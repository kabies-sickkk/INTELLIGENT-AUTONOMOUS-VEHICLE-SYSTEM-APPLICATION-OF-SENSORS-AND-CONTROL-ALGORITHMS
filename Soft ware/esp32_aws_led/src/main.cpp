/************************************************************
 * ESP32 + AWS IoT Core + Node-RED
 * Robot Car (L298N + Encoder) + LED Control
 * FIX: TLS/MQTT ổn định + tránh PADLOCK alignment + tránh ISR phá handshake
 ************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include "driver/gpio.h"

// ================= WIFI =================
const char* ssid     = "Phong Tro";
const char* password = "a123456a";

// ================= AWS IOT =================
const char* AWS_ENDPOINT = "a3bfjjwy5d1vo8-ats.iot.us-east-1.amazonaws.com";
static const uint16_t AWS_PORT = 8883;
#define CLIENT_ID "esp32_robot_full"

// ================= MQTT TOPICS =================
#define TOPIC_CMD        "iot/esp32/cmd"
#define TOPIC_STATUS     "iot/esp32/status"
#define TOPIC_ENC_A      "iot/esp32/encA"
#define TOPIC_ENC_B      "iot/esp32/encB"
#define TOPIC_LED_CMD    "iot/esp32/led/cmd"
#define TOPIC_LED_STATUS "iot/esp32/led/status"

// ================= LED =================
#define LED_PIN 25

// ================= MOTOR (L298N) =================
const int IN1 = 26;
const int IN2 = 27;
const int ENA_PIN = 4;

const int IN3 = 12;
const int IN4 = 13;
const int ENB_PIN = 14;

// ================= ENCODER =================
#define ENCL_C1 18
#define ENCL_C2 19
#define ENCR_C1 32
#define ENCR_C2 33

volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// PPR hiệu dụng của bạn
const float PPR_EFF = 40.0f;

// debounce ISR (microseconds) - giảm nhiễu phá TLS
static volatile uint32_t lastIsrUsL = 0;
static volatile uint32_t lastIsrUsR = 0;
static const uint32_t ISR_DEBOUNCE_US = 50;

// ================= PWM =================
const int freq = 5000;
const int resolution = 8;
const int ledChannelA = 0;
const int ledChannelB = 1;
uint8_t speedA = 255;
uint8_t speedB = 255;

// ================= CERT =================
// DÁN ROOT CA / CERT / KEY CỦA BẠN VÀO 3 KHỐI DƯỚI ĐÂY
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char AWS_CLIENT_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIURsrSPKENW7v9OK7WcjYbfclb9NAwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDEwNDE5NDA0
MloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKBmgQSm8bq67ewoIc13
ra50vwLRdLzZUdig/wNysq/oop63zsPTlolBBN3/4isKpU4nWcmiw/pmBA8LPQVq
CP9e0MP1ArniEMjzdfGRdCGOWhA+Enw31tl+OpoYrxu6ZVZb83GDMx8A+6ZoHlHO
xWCOdOTv16576kLV3sOtJh363yJnvnmzNoAS3Ntm+x/EzDHbZN/vg/i7EGnTBRpn
RkVvaihxseun2cM1GgWGqrXXuM89MdUTaf0/RqnOYR+b2SghhyFcaWzJwVG4Iv5O
z5I4CDpZiV2VxPOkZxkKb9W0kYokEHTrBuQaK0wJDctPkRogQ2sTz6UJJZc+NQMj
khECAwEAAaNgMF4wHwYDVR0jBBgwFoAUfNYre7zh5bqEyfLgqKa7GRa41j0wHQYD
VR0OBBYEFA8mQI5Y1x2/wWqazGukFI5NpGD+MAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBidIV+OKTIYEqn0dHXI8WoPcbY
YaVOt8xpZCaQYqlF1SJxP3CVSymAmXG7UUuZXUfqNy68EmhaaYh3Er4Pp6arDOAa
uBagLLokSsdnP7OcQrmVRvjTkPXjTaB/U34rlvaAMTRfHvKsMk6Uu9zeq2A4Jzdt
x5Q9mZ1t8oMRif23o3wMuup1sINCsijn6tSPjRftUgPQtDD5dtDb40p/QN1GCJPG
O9tte1W/1QBrjjJglYXIuVOjyxKHmXVmRvlDIcujWw4O/bIRLHT1NjftJdvOlNkg
ypWqLUTLLqz6c7z0TOsaoWA4CtHZSM1/GmCJS9Jqv8psXWqB6NgoHyypsCEf
-----END CERTIFICATE-----

)EOF";

static const char AWS_PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAoGaBBKbxurrt7CghzXetrnS/AtF0vNlR2KD/A3Kyr+iinrfO
w9OWiUEE3f/iKwqlTidZyaLD+mYEDws9BWoI/17Qw/UCueIQyPN18ZF0IY5aED4S
fDfW2X46mhivG7plVlvzcYMzHwD7pmgeUc7FYI505O/XrnvqQtXew60mHfrfIme+
ebM2gBLc22b7H8TMMdtk3++D+LsQadMFGmdGRW9qKHGx66fZwzUaBYaqtde4zz0x
1RNp/T9Gqc5hH5vZKCGHIVxpbMnBUbgi/k7PkjgIOlmJXZXE86RnGQpv1bSRiiQQ
dOsG5BorTAkNy0+RGiBDaxPPpQkllz41AyOSEQIDAQABAoIBAAbYeRcmvNhYVAEp
MVAglHQGnYRURiQtLjYNoPSXt5KUXiRh+dwXsqwG71s+KXvmx2lPeqCFZtHP+spd
k2wgu2OSu1wxn70pLcg1w8nciFVKSwO4cneEtpGvtnaPKSII0XxfDkPA8rSXCSZv
+SNKrNfTHKAMsGrM9RGHKSu2vxK2afROT5lLwfsOOV4U02niHozL/DCJk1nUZknM
3CnJxirqsEeJTDi2wW1dCTJMRfYgbkuwzmGAaFfakst7HrhyOSxIyCXGbNqaeq3o
7zq0xrcLkkjlsreYJ4PpL1IOnghe3fxlX27Zzx3KiawWxrUFqwgBSXtjZ4nMNp/i
wdUQx1UCgYEAz3TVG/Dimz5NTsYNtYRF+9Jdkm6jFHIg+SuU/RgWcxLhd6VFqC/A
AC7S0EjRw5B0XYaXjAYfDLZc82FWctmAsHCGj5Q2LN0JulHsFyFL/8qWd88XatVg
+Y8ytDaJ0BABoPOhmb+tiauUT7MmK2ru+ajCXeUv9GR12xQS0RQBgN8CgYEAxe7m
Bj4wIozldYa7hrWfinRPgsQ+Ql22XJofGT+Ngt2KFdIyvdt8BMTxM46gMbQiPhNe
hmL49HR408gQfLqf6FPYbvFRkAf8UUiBKjEILFRfc6L/46dWSIDYR2kNYkR4+IbT
Y471vDpLJZW7usOOzsE9iwhq3W4tqGjdzTS6mw8CgYEAjkKSxOdJraVqhANCcV/0
UbwvGHQv9pxQyi09rNUwx9aBsHcwLiYYZse1Hslgtte/REdAcu/bqO8BDJCKJJXM
LrHrRFghUmB9BDybRj06DX1R83SanVMXferZnUTs+jWPuFx5S6JqkUaMnJ7AJPPe
TmuQZaz9VnzxaLr6ME+kvEcCgYBBTV5KtP3AiGez7K219foa/eC1rfU0hCKETrzv
4Y3YnSrPpDfld6JlGGzTq7yiaWHaxR0sjT7BuCOdC6aU4CWdyvvfLnLQaDIQxzEG
MIMXzQZo6mAuRakr5oZ0mO43e4vFwwW/FQ1x+nGMhugW6CoXgJ3DXxWS1KTvqRl0
eG8QdQKBgC3paUXXgd0XI6w+Y7ooYnj7snwAFg5OkBTRSlcVE/2AqtKlL0RZJYyR
GDh2sp0MVLD9eoURfF0DYOy24wE0e7XDFYkdkZHzGAabDD/U1BEPqXQfAgeYDsj6
aICqwMGporjqhiivCOXm+X1N2trQrJn++xxZA+MVC02LN5OsCMQi
-----END RSA PRIVATE KEY-----

)EOF";

// ================= Aligned TLS client (FIX PADLOCK alignment) =================
// Copy buffer vào vùng aligned 16 trước khi TLS encrypt/decrypt.
class AlignedWiFiClientSecure : public WiFiClientSecure {
public:
  size_t write(const uint8_t* buf, size_t size) override {
    if (!buf || size == 0) return 0;

    // aligned 16 => đi thẳng
    if ((((uintptr_t)buf) & 0x0F) == 0) {
      return WiFiClientSecure::write(buf, size);
    }

    static uint8_t tmp[1024] __attribute__((aligned(16)));
    size_t sent = 0;

    while (size) {
      size_t n = (size > sizeof(tmp)) ? sizeof(tmp) : size;
      memcpy(tmp, buf, n);
      size_t w = WiFiClientSecure::write(tmp, n);
      sent += w;
      if (w != n) break;
      buf += n;
      size -= n;
    }
    return sent;
  }

  int read(uint8_t* buf, size_t size) override {
    if (!buf || size == 0) return 0;

    // Nếu user buffer đã aligned 16 => đọc thẳng
    if ((((uintptr_t)buf) & 0x0F) == 0) {
      return WiFiClientSecure::read(buf, size);
    }

    static uint8_t tmp[1024] __attribute__((aligned(16)));
    size_t n = (size > sizeof(tmp)) ? sizeof(tmp) : size;

    int r = WiFiClientSecure::read(tmp, n);
    if (r > 0) memcpy(buf, tmp, r);
    return r;
  }
};

AlignedWiFiClientSecure net;
PubSubClient mqtt(net);

// ================= Helpers =================
static void printTLSError(WiFiClientSecure& c) {
  char sslerr[256];
  int err = c.lastError(sslerr, sizeof(sslerr));
  Serial.print("[TLS] lastError code="); Serial.println(err);
  Serial.print("[TLS] lastError msg=");  Serial.println(sslerr);
}

static const char* mqttStateToString(int state) {
  switch (state) {
    case -4: return "MQTT_CONNECTION_TIMEOUT";
    case -3: return "MQTT_CONNECTION_LOST";
    case -2: return "MQTT_CONNECT_FAILED";
    case -1: return "MQTT_CONNECTION_ERROR";
    case  0: return "MQTT_CONNECTED";
    case  1: return "MQTT_CONNECT_BAD_PROTOCOL";
    case  2: return "MQTT_CONNECT_BAD_CLIENT_ID";
    case  3: return "MQTT_CONNECT_UNAVAILABLE";
    case  4: return "MQTT_CONNECT_BAD_CREDENTIALS";
    case  5: return "MQTT_CONNECT_UNAUTHORIZED";
    default: return "MQTT_UNKNOWN_STATE";
  }
}

static bool isTimeReady() {
  time_t now = time(nullptr);
  return now >= 1609459200; // >= 2021-01-01
}

static void ensureTimeSyncOnce() {
  static bool requested = false;
  if (requested) return;
  requested = true;

  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("[TIME] NTP request sent...");

  // chờ tối đa 15s
  uint32_t start = millis();
  while (!isTimeReady() && millis() - start < 15000) {
    delay(200);
  }

  time_t now = time(nullptr);
  struct tm t;
  if (localtime_r(&now, &t)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    Serial.print("[TIME] "); Serial.print(buf);
    Serial.print(" (epoch="); Serial.print((long)now); Serial.println(")");
  } else {
    Serial.println("[TIME] not available");
  }
}

// ================= ROBOT STATE =================
enum MoveCmd { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
volatile MoveCmd currentCmd = STOP;

volatile bool streamEncA = false;
volatile bool streamEncB = false;
const uint32_t ENC_PUB_MS = 200;

// Connection management
unsigned long lastWiFiAttempt = 0;
const unsigned long WIFI_RETRY_MS = 5000;

unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_MS = 3000;

bool mqttWasConnected = false;

// heartbeat
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_MS = 10000;

// Encoder IRQ attach after MQTT connected
bool encIrqAttached = false;

// ================= INTERRUPTS =================
void IRAM_ATTR encoderLeftISR() {
  uint32_t nowUs = (uint32_t)esp_timer_get_time();
  if (nowUs - lastIsrUsL < ISR_DEBOUNCE_US) return;
  lastIsrUsL = nowUs;

  int a = gpio_get_level((gpio_num_t)ENCL_C1);
  int b = gpio_get_level((gpio_num_t)ENCL_C2);

  portENTER_CRITICAL_ISR(&mux);
  encoderLeftCount += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderRightISR() {
  uint32_t nowUs = (uint32_t)esp_timer_get_time();
  if (nowUs - lastIsrUsR < ISR_DEBOUNCE_US) return;
  lastIsrUsR = nowUs;

  int a = gpio_get_level((gpio_num_t)ENCR_C1);
  int b = gpio_get_level((gpio_num_t)ENCR_C2);

  portENTER_CRITICAL_ISR(&mux);
  encoderRightCount += (a == b) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&mux);
}

static void attachEncoderIRQs() {
  if (encIrqAttached) return;

  // chỉ attach C1 (A) để giảm ISR; đọc B trong ISR để biết chiều
  attachInterrupt(digitalPinToInterrupt(ENCL_C1), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCR_C1), encoderRightISR, CHANGE);

  encIrqAttached = true;
  Serial.println("[ENC] IRQ attached");
}

static void detachEncoderIRQs() {
  if (!encIrqAttached) return;

  detachInterrupt(digitalPinToInterrupt(ENCL_C1));
  detachInterrupt(digitalPinToInterrupt(ENCR_C1));

  encIrqAttached = false;
  Serial.println("[ENC] IRQ detached");
}

// ================= MOTOR =================
void motorsStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ledChannelA, 0);
  ledcWrite(ledChannelB, 0);
}

void motorsForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ledChannelA, speedA);
  ledcWrite(ledChannelB, speedB);
}

void motorsBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(ledChannelA, speedA);
  ledcWrite(ledChannelB, speedB);
}

void motorsLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ledChannelA, speedA);
  ledcWrite(ledChannelB, speedB);
}

void motorsRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(ledChannelA, speedA);
  ledcWrite(ledChannelB, speedB);
}

void applyCommand(MoveCmd cmd) {
  switch (cmd) {
    case FORWARD:  motorsForward();  break;
    case BACKWARD: motorsBackward(); break;
    case LEFT:     motorsLeft();     break;
    case RIGHT:    motorsRight();    break;
    default:       motorsStop();     break;
  }
}

// ================= MQTT CALLBACK =================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  static char msg[128];
  unsigned int n = (length >= sizeof(msg)) ? (sizeof(msg) - 1) : length;
  memcpy(msg, payload, n);
  msg[n] = '\0';

  Serial.print("[MQTT RX] Topic="); Serial.print(topic);
  Serial.print(" Payload="); Serial.println(msg);

  // LED
  if (strcmp(topic, TOPIC_LED_CMD) == 0) {
    if (strcmp(msg, "ON") == 0) {
      digitalWrite(LED_PIN, HIGH);
      mqtt.publish(TOPIC_LED_STATUS, "ON", true);
    } else if (strcmp(msg, "OFF") == 0) {
      digitalWrite(LED_PIN, LOW);
      mqtt.publish(TOPIC_LED_STATUS, "OFF", true);
    }
    return;
  }

  // Encoder stream toggles
  if      (strcmp(msg, "ENA")  == 0) streamEncA = true;
  else if (strcmp(msg, "ENAS") == 0) streamEncA = false;
  else if (strcmp(msg, "ENB")  == 0) streamEncB = true;
  else if (strcmp(msg, "ENBS") == 0) streamEncB = false;
  else {
    char c = msg[0];
    if      (c == 'H') currentCmd = FORWARD;
    else if (c == 'B') currentCmd = BACKWARD;
    else if (c == 'L') currentCmd = LEFT;
    else if (c == 'R') currentCmd = RIGHT;
    else               currentCmd = STOP;

    applyCommand(currentCmd);
  }
}

// ================= AWS CONNECT =================
static void connectAWS() {
  if (mqtt.connected()) return;

  if (WiFi.status() != WL_CONNECTED) return;

  if (!isTimeReady()) return;

  if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
  lastMqttAttempt = millis();

  // quan trọng: ngắt encoder IRQ trong lúc handshake để tránh “đập” WiFi/TLS
  detachEncoderIRQs();

  // đảm bảo socket cũ đóng sạch
  net.stop();

  Serial.print("[AWS] Attempt connecting... ");

  bool ok = mqtt.connect(
    CLIENT_ID,
    TOPIC_STATUS, 1, true, "OFFLINE"   // LWT
  );

  if (ok) {
    Serial.println("OK");
    mqtt.subscribe(TOPIC_CMD);
    mqtt.subscribe(TOPIC_LED_CMD);

    mqtt.publish(TOPIC_STATUS, "ONLINE", true);
    mqtt.publish(TOPIC_LED_STATUS, digitalRead(LED_PIN) ? "ON" : "OFF", true);

    mqttWasConnected = true;

    // connect xong mới bật IRQ encoder
    attachEncoderIRQs();
  } else {
    int st = mqtt.state();
    Serial.print("FAIL state="); Serial.print(st);
    Serial.print(" ("); Serial.print(mqttStateToString(st)); Serial.println(")");
    printTLSError(net);
  }
}

// ================= ENCODER TASK =================
static void encoderTask() {
  static uint32_t t = 0;
  uint32_t nowMs = millis();
  if (nowMs - t < ENC_PUB_MS) return;
  uint32_t dt = nowMs - t;
  t = nowMs;

  long dL, dR;
  portENTER_CRITICAL(&mux);
  dL = encoderLeftCount;  encoderLeftCount = 0;
  dR = encoderRightCount; encoderRightCount = 0;
  portEXIT_CRITICAL(&mux);

  float rpmL = (PPR_EFF > 0) ? ((dL / PPR_EFF) * (60000.0f / (float)dt)) : 0.0f;
  float rpmR = (PPR_EFF > 0) ? ((dR / PPR_EFF) * (60000.0f / (float)dt)) : 0.0f;

  if (!mqtt.connected()) return;

  if (streamEncA) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"d\":%ld,\"rpm\":%.2f}", dL, rpmL);
    mqtt.publish(TOPIC_ENC_A, buf);
  }
  if (streamEncB) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"d\":%ld,\"rpm\":%.2f}", dR, rpmR);
    mqtt.publish(TOPIC_ENC_B, buf);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENCL_C1, INPUT_PULLUP);
  pinMode(ENCL_C2, INPUT_PULLUP);
  pinMode(ENCR_C1, INPUT_PULLUP);
  pinMode(ENCR_C2, INPUT_PULLUP);

  ledcSetup(ledChannelA, freq, resolution);
  ledcAttachPin(ENA_PIN, ledChannelA);

  ledcSetup(ledChannelB, freq, resolution);
  ledcAttachPin(ENB_PIN, ledChannelB);

  motorsStop();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); // giảm rớt MQTT
  WiFi.setHostname("esp32_robot");

  Serial.println("[WIFI] Connecting...");
  WiFi.begin(ssid, password);

  // TLS
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CLIENT_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);
  net.setHandshakeTimeout(15);

  mqtt.setServer(AWS_ENDPOINT, AWS_PORT);
  mqtt.setCallback(mqttCallback);

  // giữ mặc định buffer (tránh đổi malloc alignment lung tung)
  // mqtt.setBufferSize(256);

  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);
}

// ================= LOOP =================
void loop() {
  // WiFi reconnect (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiAttempt > WIFI_RETRY_MS) {
      lastWiFiAttempt = millis();
      Serial.println("[WIFI] retry...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  } else {
    // once time
    ensureTimeSyncOnce();
  }

  // MQTT connect + loop
  connectAWS();
  mqtt.loop();

  // Detect disconnect transition
  if (!mqtt.connected() && mqttWasConnected) {
    Serial.print("[AWS] disconnected, state=");
    Serial.print(mqtt.state());
    Serial.print(" ("); Serial.print(mqttStateToString(mqtt.state())); Serial.println(")");
    mqttWasConnected = false;
  }

  encoderTask();

  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = millis();
    Serial.print("[HB] RSSI="); Serial.print(WiFi.RSSI());
    Serial.print(" dBm, FreeHeap="); Serial.print(ESP.getFreeHeap());

    time_t now = time(nullptr);
    struct tm t;
    if (localtime_r(&now, &t)) {
      char buf[32];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
      Serial.print(", Time="); Serial.print(buf);
    } else {
      Serial.print(", Time=not-synced");
    }
    Serial.println();
  }
}
