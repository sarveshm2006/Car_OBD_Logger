#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <mcp2515.h>

// --- Wi-Fi Credentials ---
const char* ssid = "Aspire-Diagnostics";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
MCP2515 mcp2515(5);

const uint32_t OBD_REQUEST_ID = 0x7DF;
const uint32_t ECU_RESPONSE_ID_MIN = 0x7E8;
const uint32_t ECU_RESPONSE_ID_MAX = 0x7EF;

// --- Vehicle Data Variables (Now 10 items) ---
int rpm = 0;
int speed = 0;
int coolantTemp = 0;
int intakeTemp = 0;
float mafFlow = 0.0;
int engineLoad = 0;
int throttle = 0;
int fuelLevel = 0;
int mapPressure = 0;
float voltage = 0.0;

// --- Polling Logic ---
unsigned long lastPollTime = 0;
const int POLL_INTERVAL = 100; // ms
int currentPidIndex = 0;

// Expanded PID List
const uint8_t healthPIDs[] = {
  0x0C, // RPM
  0x0D, // Speed
  0x05, // Coolant Temp
  0x0F, // Intake Temp
  0x10, // MAF
  0x04, // Engine Load
  0x11, // Throttle Position
  0x2F, // Fuel Level
  0x0B, // Intake MAP
  0x42  // Module Voltage
};
const int NUM_PIDS = sizeof(healthPIDs) / sizeof(healthPIDs[0]);

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Ford Aspire Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background-color: #121212; color: #fff; text-align: center; padding: 20px; margin: 0; }
    h2 { color: #00d2ff; letter-spacing: 2px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 15px; max-width: 900px; margin: 0 auto; }
    .card { background: #1e1e1e; padding: 20px; border-radius: 12px; border-left: 4px solid #00d2ff; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    .value { font-size: 2.5em; font-weight: bold; margin: 10px 0; color: #fff; }
    .label { font-size: 0.9em; color: #aaa; text-transform: uppercase; }
    .unit { font-size: 0.4em; color: #888; }
    #status { margin-bottom: 20px; font-size: 0.9em; color: #ff4444; }
  </style>
</head>
<body>
  <h2>ASPIRE LIVE TELEMETRY</h2>
  <div id="status">Connecting to vehicle...</div>
  <div class="grid">
    <div class="card"><div class="label">Engine Speed</div><div class="value" id="rpm">0<span class="unit"> RPM</span></div></div>
    <div class="card"><div class="label">Vehicle Speed</div><div class="value" id="speed">0<span class="unit"> KM/H</span></div></div>
    <div class="card"><div class="label">Coolant Temp</div><div class="value" id="coolant">0<span class="unit"> &deg;C</span></div></div>
    <div class="card"><div class="label">Intake Temp</div><div class="value" id="intake">0<span class="unit"> &deg;C</span></div></div>
    <div class="card"><div class="label">Mass Air Flow</div><div class="value" id="maf">0.0<span class="unit"> g/s</span></div></div>
    <div class="card"><div class="label">Engine Load</div><div class="value" id="load">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Throttle Pos</div><div class="value" id="throttle">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Fuel Level</div><div class="value" id="fuel">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Intake MAP</div><div class="value" id="map">0<span class="unit"> kPa</span></div></div>
    <div class="card"><div class="label">ECU Voltage</div><div class="value" id="volt">0.0<span class="unit"> V</span></div></div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    
    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = function(e) { document.getElementById("status").innerHTML = "<span style='color:#00ff00'>Live Connection Established</span>"; };
      websocket.onclose = function(e) { document.getElementById("status").innerHTML = "Connection Lost. Reconnecting..."; setTimeout(initWebSocket, 2000); };
      websocket.onmessage = function(e) {
        var data = JSON.parse(e.data);
        document.getElementById("rpm").innerHTML = data.rpm + '<span class="unit"> RPM</span>';
        document.getElementById("speed").innerHTML = data.speed + '<span class="unit"> KM/H</span>';
        document.getElementById("coolant").innerHTML = data.coolant + '<span class="unit"> &deg;C</span>';
        document.getElementById("intake").innerHTML = data.intake + '<span class="unit"> &deg;C</span>';
        document.getElementById("maf").innerHTML = data.maf.toFixed(1) + '<span class="unit"> g/s</span>';
        document.getElementById("load").innerHTML = data.load + '<span class="unit"> %</span>';
        document.getElementById("throttle").innerHTML = data.thr + '<span class="unit"> %</span>';
        document.getElementById("fuel").innerHTML = data.fuel + '<span class="unit"> %</span>';
        document.getElementById("map").innerHTML = data.map + '<span class="unit"> kPa</span>';
        document.getElementById("volt").innerHTML = data.volt.toFixed(1) + '<span class="unit"> V</span>';
      };
    }
    window.addEventListener('load', initWebSocket);
  </script>
</body>
</html>
)rawliteral";

void notifyClients() {
  String json = "{\"rpm\":" + String(rpm) + 
                ",\"speed\":" + String(speed) + 
                ",\"coolant\":" + String(coolantTemp) +
                ",\"intake\":" + String(intakeTemp) +
                ",\"maf\":" + String(mafFlow) +
                ",\"load\":" + String(engineLoad) +
                ",\"thr\":" + String(throttle) +
                ",\"fuel\":" + String(fuelLevel) +
                ",\"map\":" + String(mapPressure) +
                ",\"volt\":" + String(voltage) + "}";
  ws.textAll(json);
}

void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&ws);
  server.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  ws.cleanupClients();

  if (millis() - lastPollTime > POLL_INTERVAL) {
    lastPollTime = millis();
    
    struct can_frame tx_frame;
    tx_frame.can_id = OBD_REQUEST_ID;
    tx_frame.can_dlc = 8;
    tx_frame.data[0] = 0x02;
    tx_frame.data[1] = 0x01;
    tx_frame.data[2] = healthPIDs[currentPidIndex];
    for(int i = 3; i < 8; i++) tx_frame.data[i] = 0x00;
    
    mcp2515.sendMessage(&tx_frame);
    currentPidIndex = (currentPidIndex + 1) % NUM_PIDS;
  }

  struct can_frame rx_frame;
  if (mcp2515.readMessage(&rx_frame) == MCP2515::ERROR_OK) {
    if (rx_frame.can_id >= ECU_RESPONSE_ID_MIN && rx_frame.can_id <= ECU_RESPONSE_ID_MAX) {
      if (rx_frame.data[1] == 0x41) {
        uint8_t pid = rx_frame.data[2];
        uint8_t A = rx_frame.data[3];
        uint8_t B = rx_frame.data[4];
        
        bool dataUpdated = false;

        switch (pid) {
          case 0x0C: rpm = ((A * 256) + B) / 4; dataUpdated = true; break;
          case 0x0D: speed = A; dataUpdated = true; break;
          case 0x05: coolantTemp = A - 40; dataUpdated = true; break;
          case 0x0F: intakeTemp = A - 40; dataUpdated = true; break;
          case 0x10: mafFlow = ((A * 256.0) + B) / 100.0; dataUpdated = true; break;
          case 0x04: engineLoad = (A * 100) / 255; dataUpdated = true; break;
          case 0x11: throttle = (A * 100) / 255; dataUpdated = true; break;
          case 0x2F: fuelLevel = (A * 100) / 255; dataUpdated = true; break;
          case 0x0B: mapPressure = A; dataUpdated = true; break;
          case 0x42: voltage = ((A * 256.0) + B) / 1000.0; dataUpdated = true; break;
        }

        if (dataUpdated) {
          notifyClients();
        }
      }
    }
  }
}
