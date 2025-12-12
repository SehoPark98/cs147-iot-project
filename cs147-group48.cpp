#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_CAP1188.h>
#include <DHT20.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// just manually disable the brownout error message
// such that we can send our data to the cloud successfully
#define DISABLE_BROWNOUT 1
#if DISABLE_BROWNOUT
  #include "soc/soc.h"
  #include "soc/rtc_cntl_reg.h"$
#endif

#define WIFI_SSID     "UCInet Mobile Access"
#define WIFI_PASSWORD ""

// we tried a lot of ways to solve the brownout error, so we also tried redues wifi peak level
static const wifi_power_t WIFI_TX_POWER = WIFI_POWER_2dBm;
static const char* IOTHUB_NAME = "cs147-hub-48";
static const char* DEVICE_ID   = "147esp32";

#define SAS_TOKEN "SharedAccessSignature sr=cs147-hub-48.azure-devices.net%2Fdevices%2F147esp32&sig=K8behOVPSq%2Ffh4SmqQ%2BDbEwY5qDdU2vUoZYDZTvNBUQ%3D&se=1765511447"

const char* root_ca =
"-----BEGIN CERTIFICATE-----\n"
"MIIEtjCCA56gAwIBAgIQCv1eRG9c89YADp5Gwibf9jANBgkqhkiG9w0BAQsFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
"MjAeFw0yMjA0MjgwMDAwMDBaFw0zMjA0MjcyMzU5NTlaMEcxCzAJBgNVBAYTAlVT\n"
"MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xGDAWBgNVBAMTD01TRlQg\n"
"UlMyNTYgQ0EtMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMiJV34o\n"
"eVNHI0mZGh1Rj9mdde3zSY7IhQNqAmRaTzOeRye8QsfhYFXSiMW25JddlcqaqGJ9\n"
"GEMcJPWBIBIEdNVYl1bB5KQOl+3m68p59Pu7npC74lJRY8F+p8PLKZAJjSkDD9Ex\n"
"mjHBlPcRrasgflPom3D0XB++nB1y+WLn+cB7DWLoj6qZSUDyWwnEDkkjfKee6ybx\n"
"SAXq7oORPe9o2BKfgi7dTKlOd7eKhotw96yIgMx7yigE3Q3ARS8m+BOFZ/mx150g\n"
"dKFfMcDNvSkCpxjVWnk//icrrmmEsn2xJbEuDCvtoSNvGIuCXxqhTM352HGfO2JK\n"
"AF/Kjf5OrPn2QpECAwEAAaOCAYIwggF+MBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n"
"VR0OBBYEFAyBfpQ5X8d3on8XFnk46DWWjn+UMB8GA1UdIwQYMBaAFE4iVCAYlebj\n"
"buYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcD\n"
"AQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQGCCsGAQUFBzABhhhodHRwOi8v\n"
"b2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKGNGh0dHA6Ly9jYWNlcnRzLmRp\n"
"Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcnQwQgYDVR0fBDswOTA3\n"
"oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0R2xvYmFsUm9v\n"
"dEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwCATAHBgVngQwBATAIBgZngQwB\n"
"AgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG9w0BAQsFAAOCAQEAdYWmf+AB\n"
"klEQShTbhGPQmH1c9BfnEgUFMJsNpzo9dvRj1Uek+L9WfI3kBQn97oUtf25BQsfc\n"
"kIIvTlE3WhA2Cg2yWLTVjH0Ny03dGsqoFYIypnuAwhOWUPHAu++vaUMcPUTUpQCb\n"
"eC1h4YW4CCSTYN37D2Q555wxnni0elPj9O0pymWS8gZnsfoKjvoYi/qDPZw1/TSR\n"
"penOgI6XjmlmPLBrk4LIw7P7PPg4uXUpCzzeybvARG/NIIkFv1eRYIbDF+bIkZbJ\n"
"QFdB9BjjlA4ukAg2YkOyCiB8eXTBi2APaceh3+uBLIgLk8ysy52g2U3gP7Q26Jlg\n"
"q/xKzj3O9hFh/g==\n"
"-----END CERTIFICATE-----\n";

String url = String("https://") + IOTHUB_NAME +
             ".azure-devices.net/devices/" + DEVICE_ID +
             "/messages/events?api-version=2021-04-12";

// LED and LDR (photo register)
const int LED_PIN = 33;
const int LDR_PIN = 35;

// Sensor
Adafruit_CAP1188 cap;
DHT20 dht20;

static const int SAMPLE_N = 8;
static const unsigned long REPORT_MS = 500;
unsigned long lastReport = 0;
uint8_t lastTouched = 0;
unsigned long lastTouchTime = 0;
static const unsigned long TOUCH_DEBOUNCE_MS = 300;
bool userEnabled = false;
float temperature = NAN;
float humidity    = NAN;

// sending data with 15secoonds cycle
static const unsigned long SEND_INTERVAL_MS = 15000;
unsigned long lastSend = 0;

// TLS client
WiFiClientSecure tlsClient;

// helper for ldr
int readLDRAvg() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_N; i++) {
    sum += analogRead(LDR_PIN);
    delayMicroseconds(500);
  }
  return (int)(sum / SAMPLE_N);
}

// connect wifi
bool connectWiFiOnce(unsigned long timeoutMs = 12000) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setTxPower(WIFI_TX_POWER);

  Serial.print("Connecting WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP=");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("WiFi timeout -> backoff");
  WiFi.disconnect(true);
  return false;
}

// sending data
bool sendTelemetry(float t, float h, int ldr, bool isDark, bool ledOn, bool userEn) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (isnan(t) || isnan(h)) return false;

  // create a JSON
  char payload[220];
  snprintf(payload, sizeof(payload),
           "{\"temperature\":%.2f,\"humidity\":%.2f,\"ldr\":%d,"
           "\"isDark\":%s,\"ledOn\":%s,\"userEnabled\":%s}",
           t, h, ldr,
           isDark ? "true" : "false",
           ledOn  ? "true" : "false",
           userEn ? "true" : "false");

  HTTPClient http;
  http.setTimeout(8000);

  if (!http.begin(tlsClient, url)) {
    Serial.println("http.begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", SAS_TOKEN);

  Serial.print("POST => ");
  Serial.println(payload);

  int code = http.POST((uint8_t*)payload, strlen(payload));
  Serial.print("HTTP code = ");
  Serial.println(code);

  if (code != 204) {
    String body = http.getString();
    if (body.length()) {
      Serial.print("Azure resp = ");
      Serial.println(body);
    }
  } else {
    Serial.println("Telemetry OK (204)");
  }

  http.end();
  return (code == 204);
}

void setup() {
// we just manally deleted brownout error to send our data into the cloud
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(9600);
  delay(1000);
  setCpuFrequencyMhz(80);

  // set up the led, ldr, touch sensor, humidity sensor, and connect the wifi
  pinMode(LED_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  Wire.begin();

  dht20.begin();
  Serial.println("DHT20 started.");

  if (!cap.begin()) {
    Serial.println("CAP1188 Error");
    while (1) delay(1000);
  }
  Serial.println("CAP1188 Initialized.");

  tlsClient.setCACert(root_ca);
  connectWiFiOnce();

  lastReport = millis();
  lastSend = millis();
}

void loop() {
  unsigned long now = millis();

  // 1) LDR
  int v = readLDRAvg();
  bool isDark = (v < 3000);

  // 2) Touch toggle
  uint8_t currTouched = cap.touched();
  if (currTouched != 0 && lastTouched == 0 &&
      (now - lastTouchTime) > TOUCH_DEBOUNCE_MS) {
    lastTouchTime = now;
    userEnabled = !userEnabled;
    Serial.print("Touch! userEnabled -> ");
    Serial.println(userEnabled ? "true" : "false");
  }
  lastTouched = currTouched;

  // 3) DHT20 read
  bool dhtOk = false;
  if (dht20.read() == 0) {
    temperature = dht20.getTemperature();
    humidity    = dht20.getHumidity();
    dhtOk = true;
  }

  // 4) LED
  bool ledOn = (userEnabled || isDark);
  digitalWrite(LED_PIN, ledOn ? HIGH : LOW);

  // 5) Serial report
  if (now - lastReport >= REPORT_MS) {
    Serial.print("LDR=");
    Serial.print(v);
    Serial.print(" isDark=");
    Serial.print(isDark ? "Y" : "N");
    Serial.print(" userEnabled=");
    Serial.print(userEnabled ? "Y" : "N");
    Serial.print(" LED=");
    Serial.print(ledOn ? "ON" : "OFF");
    Serial.print(" | Temp=");
    Serial.print(temperature, 1);
    Serial.print("C Hum=");
    Serial.print(humidity, 1);
    Serial.println("%");
    lastReport = now;
  }

  // Sending data to the Azure cloud
  if (dhtOk && (now - lastSend >= SEND_INTERVAL_MS)) {
    if (WiFi.status() != WL_CONNECTED) {
      if (!connectWiFiOnce()) {
        lastSend = now;
        delay(500);
        return;
      }
    }

    sendTelemetry(temperature, humidity, v, isDark, ledOn, userEnabled);
    lastSend = now;
  }

  delay(200);
}
