/*********************************************************************************************
 * ESP32 - I2C SLAVE (0x30) + Web UI (AP) + DEBUG + I2C SCAN on-demand
 * Protocolo de registros 8-bit:
 *   0x01 REG_UP   (W): P1..P4  (b0..b3, 1=ocupado)
 *   0x02 REG_DN   (W): P5..P8  (b0..b3)
 *   0x03 REG_ALL  (R): (DN<<4)|UP   (P8..P1)
 *   0x04 REG_FREE (R): libres 0..8
 *   0x05 REG_FLAGS(R/W opc)
 *
 * Web AP: SSID=Parqueo-matic  PASS=12345678  IP=192.168.4.1
 * Endpoints:
 *   "/"             -> UI
 *   "/api/state"    -> {"free":N,"spots":[bits 0/1]}
 *   "/api/i2cscan"  -> {"found":["0x08","0x21","0x30",...]}
 *********************************************************************************************/
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

/* ---------------- I2C config ---------------- */
#define I2C_SLAVE_ADDR   0x30       // Dirección 7-bit del ESP32 (esclavo)
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define I2C_FREQ_HZ      100000

enum : uint8_t { REG_UP=0x01, REG_DN=0x02, REG_ALL=0x03, REG_FREE=0x04, REG_FLAGS=0x05 };

volatile uint8_t regs[8] = {0};     // banco simple
volatile uint8_t reg_ptr = REG_UP;

/* ---------------- WiFi + Web ---------------- */
const char* ap_ssid="Parqueo-matic"; 
const char* ap_pass="12345678";
IPAddress ip(192,168,4,1), gw(192,168,4,1), mask(255,255,255,0);
WebServer server(80);

/* ---------------- Helpers ---------------- */
static inline void debug_bits(const char* tag, uint8_t v) {
  Serial.print(tag); Serial.print(" = 0b");
  for (int i=3;i>=0;--i) Serial.print((v>>i)&1);
  Serial.print(" (0x"); Serial.print(v,HEX); Serial.println(")");
}

static inline void recompute() {
  uint8_t up = regs[REG_UP] & 0x0F;
  uint8_t dn = regs[REG_DN] & 0x0F;
  uint8_t occ = __builtin_popcount(up) + __builtin_popcount(dn);
  regs[REG_ALL]  = (uint8_t)((dn<<4) | up);
  regs[REG_FREE] = (occ>8)?0:(8-occ);

  Serial.print("[I2C][RECOMPUTE] ");
  debug_bits("UP", up);
  debug_bits("DN", dn);
  Serial.print("    -> REG_ALL=0x"); Serial.print(regs[REG_ALL],HEX);
  Serial.print("  REG_FREE="); Serial.println(regs[REG_FREE]);
}

/* ---------------- I2C Callbacks (SLAVE) ---------------- */
void onReceiveCB(int len) {
  if (len <= 0) return;

  uint8_t first = Wire.read(); 
  reg_ptr = first; 
  len--;

  if (len >= 1 && reg_ptr < sizeof(regs)) {
    uint8_t data = Wire.read(); 
    regs[reg_ptr] = data; 
    len--;

    Serial.print("[I2C][RX] WRITE reg 0x");
    Serial.print(reg_ptr, HEX);
    Serial.print(" <= 0x");
    Serial.println(data, HEX);

    if (reg_ptr==REG_UP || reg_ptr==REG_DN) recompute();

    /* Si vienen más bytes secuenciales, se almacenan y se loguean */
    uint8_t rp = reg_ptr + 1;
    while (len-- > 0 && Wire.available() && rp < sizeof(regs)) {
      uint8_t d = Wire.read();
      regs[rp] = d;
      Serial.print("[I2C][RX] WRITE reg 0x");
      Serial.print(rp, HEX);
      Serial.print(" <= 0x");
      Serial.println(d, HEX);
      rp++;
    }
  } else {
    Serial.print("[I2C][RX] PTR=0x"); Serial.print(reg_ptr,HEX);
    Serial.println(" (sin data)");
    while (len-- > 0 && Wire.available()) (void)Wire.read(); // flush
  }
}

void onRequestCB() {
  uint8_t v = regs[reg_ptr];
  Wire.write(&v,1);

  Serial.print("[I2C][TX] READ reg 0x");
  Serial.print(reg_ptr, HEX);
  Serial.print(" -> 0x");
  Serial.println(v, HEX);
}

/* ---------------- Web HTML ---------------- */
static String html() {
  String h; h.reserve(12000);
  h += F("<!DOCTYPE html><html lang='es'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>Parqueo-matic</title><style>"
         ":root{--slot:#eaeef3;--lane:#f7c64b;--gray:#9aa3ad}*{box-sizing:border-box}"
         "body{margin:0;padding:18px;font-family:Inter,system-ui,-apple-system,Segoe UI,Roboto,Arial}"
         "h1{text-align:center;margin:0 0 10px}.wrap{max-width:980px;margin:0 auto}"
         ".grid{display:grid;grid-template-columns:repeat(4,1fr);gap:16px;margin-top:10px}"
         "@media(max-width:820px){.grid{grid-template-columns:repeat(2,1fr)}}"
         "@media(max-width:480px){.grid{grid-template-columns:1fr}}"
         ".slot{background:#fff;border-radius:14px;box-shadow:0 3px 14px rgba(0,0,0,.08);padding:14px}"
         ".bay{position:relative;background:var(--slot);height:140px;border-radius:10px;overflow:hidden}"
         ".laneL,.laneR{position:absolute;top:0;bottom:0;width:10px;background:var(--lane)}.laneL{left:0}.laneR{right:0}"
         ".bump{position:absolute;left:50%;transform:translateX(-50%);bottom:14px;width:70%;height:10px;background:var(--lane);border-radius:6px}"
         ".carE{position:absolute;left:50%;top:18px;transform:translateX(-50%);font-size:100px;line-height:1}"
         ".lights{display:flex;gap:8px;justify-content:center;margin:12px 0 6px}"
         ".led{width:22px;height:22px;border-radius:50%;box-shadow:inset 0 0 0 3px rgba(0,0,0,.08)}"
         ".red{background:#cc3344}.green{background:#39b54a}.off{filter:grayscale(100%);opacity:.35}"
         ".id{display:block;text-align:center;color:var(--gray);font-size:12px}"
         ".meta{max-width:980px;margin:8px auto 0;color:#555}"
         ".btn{display:inline-block;padding:8px 12px;border-radius:8px;border:1px solid #999;cursor:pointer}"
         "</style></head><body><div class='wrap'><h1>Parqueo-matic</h1>"
         "<div class='meta'>Libres: <span id='free'>—</span> "
         "<button class='btn' onclick='scan()'>I2C Scan</button> <span id='scan'></span></div>"
         "<div class='grid' id='grid'></div></div>"
         "<script>const grid=document.getElementById('grid');let prev=[];"
         "function card(i,occ){const icon=occ?'🚗':'✅';return `<div class='slot' data-i='${i}'>"
         "<div class='bay'><div class='laneL'></div><div class='laneR'></div><div class='bump'></div><div class='carE'>${icon}</div></div>"
         "<div class='lights'><div class='led red ${occ?'':'off'}'></div><div class='led green ${occ?'off':''}'></div></div>"
         "<span class='id'>Espacio #${i+1}</span></div>`}"
         "function ensureGrid(n){if(grid.children.length===n)return;let html='';for(let i=0;i<n;i++)html+=card(i,false);grid.innerHTML=html;prev=new Array(n).fill(null);} "
         "function patch(spots){ensureGrid(spots.length);for(let i=0;i<spots.length;i++){const occ=!!spots[i];if(prev[i]===occ)continue;const node=grid.children[i];"
         "if(!node)continue;node.querySelector('.carE').textContent=occ?'🚗':'✅';"
         "const r=node.querySelector('.led.red'),g=node.querySelector('.led.green');"
         "r.classList.toggle('off',!occ);g.classList.toggle('off',occ);prev[i]=occ;}}"
         "async function tick(){try{const r=await fetch('/api/state');const js=await r.json();"
         "document.getElementById('free').textContent=js.free??'—';patch(js.spots||[]);}catch(e){console.log(e)}}"
         "async function scan(){document.getElementById('scan').textContent='...';"
         "try{const r=await fetch('/api/i2cscan');const js=await r.json();document.getElementById('scan').textContent='found: '+(js.found||[]).join(', ');}catch(e){document.getElementById('scan').textContent='error';}}"
         "ensureGrid(8);tick();setInterval(tick,500);</script></body></html>");
  return h;
}

static void handleRoot(){ server.send(200,"text/html", html()); }

static void handleState(){
  uint8_t up=regs[REG_UP]&0x0F, dn=regs[REG_DN]&0x0F, freec=regs[REG_FREE];
  String js="{\"free\":"+String(freec)+",\"spots\":[";
  for(int i=0;i<4;i++){ js += ((up>>i)&1)?'1':'0'; js += ','; }
  for(int i=0;i<4;i++){ js += ((dn>>i)&1)?'1':'0'; if(i<3) js+=','; }
  js += "]}"; 
  server.send(200,"application/json",js);
}

/* ---------------- I2C Scanner on-demand ----------------
   Pausa modo esclavo, inicia maestro en los mismos pines,
   escanea 0x01..0x7E, devuelve lista, y restaura el modo esclavo.  */
static void i2c_begin_slave() {
  Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.onReceive(onReceiveCB);
  Wire.onRequest(onRequestCB);
}

static void i2c_end() {
  Wire.end(); // disponible en ESP32
}

/* Devuelve las direcciones encontradas en 'out_addrs' y cuenta en '*out_count' */
static void i2c_scan_once(uint8_t* out_addrs, uint8_t max_len, uint8_t* out_count) {
  if (!out_addrs || !out_count) return;
  *out_count = 0;

  i2c_end();
  delay(2);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ); // Maestro
  delay(2);

  Serial.println("[SCAN] Iniciando escaneo I2C...");
  for (uint8_t addr=1; addr<127; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err==0) {
      if (*out_count < max_len) {
        out_addrs[*out_count] = addr;
        (*out_count)++;
      }
      Serial.print("[SCAN] Dispositivo en 0x"); Serial.println(addr, HEX);
    } else if (err==4) {
      Serial.print("[SCAN] Error desconocido en 0x"); Serial.println(addr, HEX);
    }
  }
  if (*out_count==0) Serial.println("[SCAN] No se encontró ningún dispositivo.");

  // Restaurar SLAVE
  i2c_begin_slave();
  Serial.println("[SCAN] Fin. Modo SLAVE restaurado.");
}

static void handleScan(){
  uint8_t found[16]; uint8_t n=0;
  i2c_scan_once(found, 16, &n);

  String js = "{\"found\":[";
  for (uint8_t i=0;i<n;i++){
    js += "\"0x";
    if (found[i]<16) js += "0";
    js += String(found[i],HEX);
    js += "\"";
    if (i+1<n) js += ",";
  }
  js += "]}";
  server.send(200,"application/json", js);
}

/* ---------------- Setup / Loop ---------------- */
void setup(){
  Serial.begin(115200);
  delay(50);

  // I2C esclavo
  i2c_begin_slave();
  // Estado inicial coherente
  regs[REG_UP] = regs[REG_UP] & 0x0F;
  regs[REG_DN] = regs[REG_DN] & 0x0F;
  recompute();

  // WiFi AP + Web
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip,gw,mask);
  WiFi.softAP(ap_ssid,ap_pass);
  server.on("/",            handleRoot);
  server.on("/api/state",   handleState);
  server.on("/api/i2cscan", handleScan);
  server.begin();

  Serial.print("[I2C] Slave @0x"); Serial.println(I2C_SLAVE_ADDR, HEX);
  Serial.print("[I2C] SDA="); Serial.print(I2C_SDA_PIN);
  Serial.print("  SCL="); Serial.println(I2C_SCL_PIN);
  Serial.print("[WiFi] AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.println("Comandos Serial: 'SCAN' -> escanear I2C (pausa SLAVE, escanea, restaura).");
}

void loop(){
  server.handleClient();

  // Comando de consola para escaneo
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.equalsIgnoreCase("SCAN")){
      uint8_t found[16]; uint8_t n=0;
      i2c_scan_once(found, 16, &n);
      Serial.print("[SCAN] Encontrados: ");
      if (n==0) Serial.println("(ninguno)");
      else {
        for (uint8_t i=0;i<n;i++){
          Serial.print("0x"); Serial.print(found[i],HEX);
          if (i+1<n) Serial.print(", ");
        }
        Serial.println();
      }
    }
  }
}



