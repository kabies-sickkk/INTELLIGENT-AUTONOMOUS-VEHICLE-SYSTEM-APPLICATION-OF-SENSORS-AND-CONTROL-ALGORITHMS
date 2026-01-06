#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

// ================= CONFIG =================
#define LED_PIN        2
#define AWS_PORT       8883
#define CLIENT_ID      "ESP32_AWS_CLIENT"

// ================= WIFI =================
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ================= AWS IOT =================
const char* aws_endpoint = "xxxxxxxxxxxxx-ats.iot.ap-southeast-1.amazonaws.com";

#define PUB_TOPIC "esp32/pub"
#define SUB_TOPIC "esp32/sub"

// ================= TIME =================
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 3600;   // GMT+7 (VN)
const int   daylightOffset_sec = 0;

// ================= CERTIFICATES =================
// Amazon Root CA 1
const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_AMAZON_ROOT_CA_1
-----END CERTIFICATE-----
)EOF";

// Device Certificate
const char* client_cert = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_DEVICE_CERTIFICATE
-----END CERTIFICATE-----
)EOF";

// Private Key
const char* private_key = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
YOUR_PRIVATE_KEY
-----END RSA PRIVATE KEY-----
)EOF";

// ================= OBJECTS =================
WiFiClientSecure net;
PubSubClient client(net);

// ================= MQTT CALLBACK =================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println(msg);

  if (String(topic) == SUB_TOPIC) {
    if (msg == "ON") {
      digitalWrite(LED_PIN, HIGH);
      client.publish(PUB_TOPIC, "LED ON");
    } 
    else if (msg == "OFF") {
      digitalWrite(LED_PIN, LOW);
      client.publish(PUB_TOPIC, "LED OFF");
    }
  }
}

// ================= SETUP TIME (BẮT BUỘC TLS) =================
void setupTime() {
  Serial.print("[TIME] Syncing time");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  time_t now;
  while (time(&now) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n[TIME] Time synchronized");
}

// ================= CONNECT AWS =================
void connectAWS() {
  while (!client.connected()) {
    Serial.print("[AWS] Connecting to AWS IoT...");
    if (client.connect(CLIENT_ID)) {
      Serial.println(" SUCCESS");
      client.subscribe(SUB_TOPIC);
      Serial.println("[AWS] Subscribed to topic");
      client.publish(PUB_TOPIC, "ESP32 Connected");
    } else {
      Serial.print(" FAILED, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 2s");
      delay(2000);
    }
  }
}

// ================= SETUP =================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[SYS] ESP32 Booting...");

  // WiFi
  Serial.print("[WiFi] Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n[WiFi] Connected successfully");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  // Time (BẮT BUỘC)
  setupTime();

  // TLS
  net.setCACert(ca_cert);
  net.setCertificate(client_cert);
  net.setPrivateKey(private_key);

  // MQTT
  client.setServer(aws_endpoint, AWS_PORT);
  client.setCallback(callback);

  connectAWS();
}

// ================= LOOP =================
void loop() {
  if (!client.connected()) {
    connectAWS();
  }
  client.loop();
}
