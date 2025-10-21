/*
 * ESP32-CAM — Wi-Fi UDP → UART (100 Hz, no bloqueante)
 * Emite: M1:...;M2:... cada 10 ms aprox.
 */

#include <WiFi.h>
#include <WiFiUdp.h>

#define SSID_AP   "BAD_ICE_CREAM"
#define PASS_AP   "Heladito2005"
#define UDP_PORT  5000

// Frecuencia de envío (puedes cambiarla)
// 100 Hz = 10 ms; 200 Hz = 5 ms; 50 Hz = 20 ms
#define SEND_HZ         100
#define SEND_PERIOD_MS  (1000 / SEND_HZ)

WiFiUDP Udp;

// Estados recientes
String dataM1 = "M1:X0Y0SW0A0B0";
String dataM2 = "M2:X0Y0SW0A0B0";

// Watchdog
unsigned long lastM1 = 0;
unsigned long lastM2 = 0;
const unsigned long TIMEOUT_MS = 1500; // 1.5 s

// Temporizador de envío
unsigned long t_last_send = 0;

void setup() {
  Serial.begin(115200);       // Mantengo 115200 para compatibilidad
  delay(200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_AP, PASS_AP);
  Udp.begin(UDP_PORT);

  unsigned long now = millis();
  lastM1 = now;
  lastM2 = now;
  t_last_send = now;
}

void loop() {
  // Leer todos los paquetes disponibles (no bloqueante)
  while (Udp.parsePacket()) {
    char incoming[128];
    int len = Udp.read(incoming, sizeof(incoming) - 1);
    if (len <= 0) continue;
    incoming[len] = '\0';
    String msg = String(incoming);

    unsigned long now = millis();
    if (msg.startsWith("M1:")) {
      dataM1 = msg;
      lastM1 = now;
    } else if (msg.startsWith("M2:")) {
      dataM2 = msg;
      lastM2 = now;
    }
  }

  // Watchdog: conservar último valor (sin prints)
  unsigned long now = millis();
  if (now - lastM1 > TIMEOUT_MS) lastM1 = now;
  if (now - lastM2 > TIMEOUT_MS) lastM2 = now;

  // Envío cronometrado (no usar delay largo para no frenar WiFi)
  if (now - t_last_send >= SEND_PERIOD_MS) {
    t_last_send += SEND_PERIOD_MS; // evita deriva si hay jitter
    Serial.println(dataM1 + ";" + dataM2);
  }

  // Ceder CPU a la pila WiFi/RTOS sin frenar el bucle
  delay(1);
}