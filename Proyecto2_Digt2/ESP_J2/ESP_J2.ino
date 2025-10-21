/* M2 cliente UDP - cached VRX (ADC2) */
#include <WiFi.h>
#include <WiFiUdp.h>

#define WIFI_SSID "BAD_ICE_CREAM"
#define WIFI_PASS "Heladito2005"
#define UDP_PORT 5000
#define SERVER_IP "192.168.4.1"

WiFiUDP Udp;

#define SW   32
#define VRY  33
#define VRX  25   // ADC2 stuck pin (no tocar)
#define BTN_A 27
#define BTN_B 14

const unsigned long READ_VRX_INTERVAL_MS = 800; // leer VRX solo cada 800 ms
unsigned long lastVrxRead = 0;
int cachedX = 2048; // valor por defecto centro

void setup() {
  Serial.begin(115200);
  pinMode(SW, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);

  Serial.println("\nConectando a BAD_ICE_CREAM...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nConectado al AP. IP:");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long now = millis();

  // 1) Actualizar cachedX solo cada X ms (apagar radio brevemente para leer ADC2)
  if (now - lastVrxRead >= READ_VRX_INTERVAL_MS) {
    // Apagar radio, leer ADC2, volver a activar
    WiFi.mode(WIFI_OFF);
    delayMicroseconds(300); // estabiliza el radio/apagado
    int x = analogRead(VRX);
    cachedX = x;
    lastVrxRead = now;
    WiFi.mode(WIFI_STA);
    // Dar un pequeño tiempo para que la interfaz retome (no mucho)
    delay(50);
  }

  // 2) Lecturas ADC1 y botones habituales
  int yValue = analogRead(VRY);          // ADC1, seguro con WiFi
  int swState = !digitalRead(SW);
  int aState  = !digitalRead(BTN_A);
  int bState  = !digitalRead(BTN_B);

  String paquete = "M2:X" + String(cachedX) +
                   "Y" + String(yValue) +
                   "SW" + String(swState) +
                   "A" + String(aState) +
                   "B" + String(bState);

  // 3) enviar UDP (si estamos conectados)
  if (WiFi.status() == WL_CONNECTED) {
    Udp.beginPacket(SERVER_IP, UDP_PORT);
    Udp.print(paquete);
    Udp.endPacket();
    Serial.println("Enviado: " + paquete);
  } else {
    Serial.println("WiFi no conectado, reintentando...");
    WiFi.reconnect();
  }

  delay(50); // ~20 Hz
}