#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <mcp2515.h>

// --- Configuration ---
const char* ssid = "Aspire-Diagnostics";
const char* password = "password123";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
MCP2515 mcp2515(5);

const uint32_t OBD_REQUEST_ID = 0x7DF;
const uint32_t ECU_RESPONSE_ID_MIN = 0x7E8;
const uint32_t ECU_RESPONSE_ID_MAX = 0x7EF;

// --- Telemetry Variables (18 Total) ---
int rpm = 0, speed = 0, coolantTemp = 0, intakeTemp = 0;
int engineLoad = 0, throttle = 0, fuelLevel = 0, mapPressure = 0;
float mafFlow = 0.0, voltage = 0.0;
// NEW VARIABLES
float shortFuelTrim = 0.0, longFuelTrim = 0.0, timingAdvance = 0.0;
float o2Voltage = 0.0;
int runTime = 0, baroPressure = 0, ambientTemp = 0, pedalPos = 0;

// --- State Tracking ---
unsigned long lastPollTime = 0;
const int POLL_INTERVAL = 100; // ms
int currentPidIndex = 0;
bool scanningDtc = false;
unsigned long dtcScanTimeout = 0;

// Expanded Polling Array (18 PIDs)
const uint8_t healthPIDs[] = {
  0x0C, 0x0D, 0x05, 0x0F, 0x10, 0x04, 0x11, 0x2F, 0x0B, 0x42, // Original 10
  0x06, 0x07, 0x0E, 0x14, 0x1F, 0x33, 0x46, 0x5A              // New 8
};
const int NUM_PIDS = sizeof(healthPIDs) / sizeof(healthPIDs[0]);

// --- UI (HTML/JS/CSS) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Ford Aspire Ultimate Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background-color: #121212; color: #fff; text-align: center; padding: 20px; margin: 0; }
    h2 { color: #00d2ff; letter-spacing: 2px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 10px; max-width: 1000px; margin: 0 auto; }
    .card { background: #1e1e1e; padding: 15px; border-radius: 10px; border-left: 4px solid #00d2ff; box-shadow: 0 4px 8px rgba(0,0,0,0.5); }
    .value { font-size: 2em; font-weight: bold; margin: 8px 0; color: #fff; }
    .label { font-size: 0.8em; color: #aaa; text-transform: uppercase; }
    .unit { font-size: 0.4em; color: #888; }
    #status { margin-bottom: 20px; font-size: 0.9em; color: #ff4444; }
    
    .control-section { max-width: 1000px; margin: 15px auto; background: #1a1a1f; padding: 15px; border-radius: 10px; border: 1px solid #333; display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; align-items: center;}
    button { color: white; border: none; padding: 10px 20px; font-size: 0.9em; font-weight: bold; border-radius: 6px; cursor: pointer; transition: 0.2s; }
    button:disabled { background: #555 !important; cursor: not-allowed; }
    #dtcBtn { background: #ff3344; } #dtcBtn:hover { background: #e02434; }
    #recordBtn { background: #4CAF50; } #recordBtn:hover { background: #45a049; }
    #downloadBtn { background: #008CBA; } #downloadBtn:hover { background: #007bb5; }
    .dtc-results { width: 100%; margin-top: 10px; font-size: 1em; }
    .dtc-pill { display: inline-block; background: #ff3344; color: #fff; padding: 5px 12px; border-radius: 5px; margin: 4px; font-weight: bold; }
    .dtc-none { color: #00ff88; font-weight: bold; }
    .log-status { font-size: 0.9em; color: #aaa; }
  </style>
</head>
<body>
  <h2>ASPIRE TELEMETRY</h2>
  <div id="status">Connecting to vehicle...</div>

  <div class="control-section">
    <button id="dtcBtn" onclick="requestDtcScan()">Scan Fault Codes</button>
    <button id="recordBtn" onclick="toggleRecording()">Start Recording Data</button>
    <button id="downloadBtn" onclick="downloadCSV()" disabled>Download CSV</button>
    <div class="log-status" id="logCount">Logged: 0 rows</div>
    <div class="dtc-results" id="dtcList"></div>
  </div>

  <div class="grid">
    <div class="card"><div class="label">Engine Speed</div><div class="value" id="rpm">0<span class="unit"> RPM</span></div></div>
    <div class="card"><div class="label">Vehicle Speed</div><div class="value" id="speed">0<span class="unit"> KM/H</span></div></div>
    <div class="card"><div class="label">Coolant Temp</div><div class="value" id="coolant">0<span class="unit"> &deg;C</span></div></div>
    <div class="card"><div class="label">Intake Temp</div><div class="value" id="intake">0<span class="unit"> &deg;C</span></div></div>
    <div class="card"><div class="label">Mass Air Flow</div><div class="value" id="maf">0.0<span class="unit"> g/s</span></div></div>
    <div class="card"><div class="label">Engine Load</div><div class="value" id="load">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Throttle Pos</div><div class="value" id="throttle">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Accel Pedal</div><div class="value" id="pedal">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Fuel Level</div><div class="value" id="fuel">0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Intake MAP</div><div class="value" id="map">0<span class="unit"> kPa</span></div></div>
    <div class="card"><div class="label">Baro Pressure</div><div class="value" id="baro">0<span class="unit"> kPa</span></div></div>
    <div class="card"><div class="label">ECU Voltage</div><div class="value" id="volt">0.0<span class="unit"> V</span></div></div>
    <div class="card"><div class="label">Short Fuel Trim</div><div class="value" id="sft">0.0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Long Fuel Trim</div><div class="value" id="lft">0.0<span class="unit"> %</span></div></div>
    <div class="card"><div class="label">Timing Advance</div><div class="value" id="timing">0.0<span class="unit"> &deg;</span></div></div>
    <div class="card"><div class="label">O2 Bank 1</div><div class="value" id="o2">0.0<span class="unit"> V</span></div></div>
    <div class="card"><div class="label">Ambient Temp</div><div class="value" id="ambient">0<span class="unit"> &deg;C</span></div></div>
    <div class="card"><div class="label">Run Time</div><div class="value" id="runtime">0<span class="unit"> s</span></div></div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    var isRecording = false;
    var logData = [];
    var csvHeaders = "Timestamp,RPM,Speed,Coolant,IntakeTemp,MAF,Load,Throttle,Pedal,FuelLevel,MAP,Baro,Voltage,ShortTrim,LongTrim,Timing,O2Volts,Ambient,RunTime\n";

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = function(e) { document.getElementById("status").innerHTML = "<span style='color:#00ff00'>Live Connection Established</span>"; };
      websocket.onclose = function(e) { document.getElementById("status").innerHTML = "Connection Lost. Reconnecting..."; setTimeout(initWebSocket, 2000); };
      
      websocket.onmessage = function(e) {
        try {
          var msg = JSON.parse(e.data);
          
          if (msg.type === "telemetry") {
            document.getElementById("rpm").innerHTML = msg.rpm + '<span class="unit"> RPM</span>';
            document.getElementById("speed").innerHTML = msg.speed + '<span class="unit"> KM/H</span>';
            document.getElementById("coolant").innerHTML = msg.coolant + '<span class="unit"> &deg;C</span>';
            document.getElementById("intake").innerHTML = msg.intake + '<span class="unit"> &deg;C</span>';
            document.getElementById("maf").innerHTML = msg.maf.toFixed(1) + '<span class="unit"> g/s</span>';
            document.getElementById("load").innerHTML = msg.load + '<span class="unit"> %</span>';
            document.getElementById("throttle").innerHTML = msg.thr + '<span class="unit"> %</span>';
            document.getElementById("pedal").innerHTML = msg.pedal + '<span class="unit"> %</span>';
            document.getElementById("fuel").innerHTML = msg.fuel + '<span class="unit"> %</span>';
            document.getElementById("map").innerHTML = msg.map + '<span class="unit"> kPa</span>';
            document.getElementById("baro").innerHTML = msg.baro + '<span class="unit"> kPa</span>';
            document.getElementById("volt").innerHTML = msg.volt.toFixed(1) + '<span class="unit"> V</span>';
            document.getElementById("sft").innerHTML = msg.sft.toFixed(1) + '<span class="unit"> %</span>';
            document.getElementById("lft").innerHTML = msg.lft.toFixed(1) + '<span class="unit"> %</span>';
            document.getElementById("timing").innerHTML = msg.timing.toFixed(1) + '<span class="unit"> &deg;</span>';
            document.getElementById("o2").innerHTML = msg.o2.toFixed(2) + '<span class="unit"> V</span>';
            document.getElementById("ambient").innerHTML = msg.ambient + '<span class="unit"> &deg;C</span>';
            document.getElementById("runtime").innerHTML = msg.runtime + '<span class="unit"> s</span>';

            if (isRecording) {
              var ts = new Date().toLocaleTimeString();
              var row = `${ts},${msg.rpm},${msg.speed},${msg.coolant},${msg.intake},${msg.maf.toFixed(1)},${msg.load},${msg.thr},${msg.pedal},${msg.fuel},${msg.map},${msg.baro},${msg.volt.toFixed(1)},${msg.sft.toFixed(1)},${msg.lft.toFixed(1)},${msg.timing.toFixed(1)},${msg.o2.toFixed(2)},${msg.ambient},${msg.runtime}\n`;
              logData.push(row);
              document.getElementById("logCount").innerText = "Logged: " + (logData.length - 1) + " rows";
            }
          } else if (msg.type === "dtc") {
            document.getElementById("dtcBtn").disabled = false;
            var container = document.getElementById("dtcList");
            container.innerHTML = "";
            if (msg.codes.length === 0) {
              container.innerHTML = "<span class='dtc-none'>No Fault Codes Found (System Clean)</span>";
            } else {
              msg.codes.forEach(function(code) { container.innerHTML += "<span class='dtc-pill'>" + code + "</span>"; });
            }
          }
        } catch (err) {}
      };
    }

    function requestDtcScan() {
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        document.getElementById("dtcBtn").disabled = true;
        document.getElementById("dtcList").innerHTML = "<span style='color:#00d2ff'>Querying ECU...</span>";
        websocket.send(JSON.stringify({ action: "scan_dtc" }));
      }
    }

    function toggleRecording() {
      isRecording = !isRecording;
      var btn = document.getElementById("recordBtn");
      var dBtn = document.getElementById("downloadBtn");
      if (isRecording) {
        btn.innerText = "Stop Recording"; btn.style.background = "#ff9900"; dBtn.disabled = true;
        if (logData.length === 0) logData.push(csvHeaders); 
      } else {
        btn.innerText = "Start Recording Data"; btn.style.background = "#4CAF50"; dBtn.disabled = (logData.length <= 1); 
      }
    }

    function downloadCSV() {
      if (logData.length <= 1) return;
      var blob = new Blob(logData, { type: 'text/csv' });
      var url = window.URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.setAttribute('hidden', ''); a.setAttribute('href', url); a.setAttribute('download', 'Aspire_Full_Log.csv');
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
    }
    window.addEventListener('load', initWebSocket);
  </script>
</body>
</html>
)rawliteral";

String decodeDtc(uint8_t highByte, uint8_t lowByte) {
  if (highByte == 0x00 && lowByte == 0x00) return "";
  char category;
  switch (highByte >> 6) { case 0: category = 'P'; break; case 1: category = 'C'; break; case 2: category = 'B'; break; case 3: category = 'U'; break; }
  char codeBuffer[6];
  snprintf(codeBuffer, sizeof(codeBuffer), "%c%X%X%X%X", category, (highByte >> 4) & 0x03, highByte & 0x0F, lowByte >> 4, lowByte & 0x0F);
  return String(codeBuffer);
}

void notifyTelemetry() {
  String json = "{\"type\":\"telemetry\""
                ",\"rpm\":" + String(rpm) + ",\"speed\":" + String(speed) + ",\"coolant\":" + String(coolantTemp) +
                ",\"intake\":" + String(intakeTemp) + ",\"maf\":" + String(mafFlow, 2) + ",\"load\":" + String(engineLoad) +
                ",\"thr\":" + String(throttle) + ",\"pedal\":" + String(pedalPos) + ",\"fuel\":" + String(fuelLevel) +
                ",\"map\":" + String(mapPressure) + ",\"baro\":" + String(baroPressure) + ",\"volt\":" + String(voltage, 2) + 
                ",\"sft\":" + String(shortFuelTrim, 2) + ",\"lft\":" + String(longFuelTrim, 2) + 
                ",\"timing\":" + String(timingAdvance, 2) + ",\"o2\":" + String(o2Voltage, 2) + 
                ",\"ambient\":" + String(ambientTemp) + ",\"runtime\":" + String(runTime) + "}";
  ws.textAll(json);
}

void triggerDtcQuery() {
  scanningDtc = true; dtcScanTimeout = millis() + 2000;
  struct can_frame tx_frame; tx_frame.can_id = OBD_REQUEST_ID; tx_frame.can_dlc = 8;
  tx_frame.data[0] = 0x01; tx_frame.data[1] = 0x03;
  for (int i = 2; i < 8; i++) tx_frame.data[i] = 0x00;
  mcp2515.sendMessage(&tx_frame);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; if (strstr((char*)data, "\"scan_dtc\"") != NULL) triggerDtcQuery();
    }
  }
}

void setup() {
  Serial.begin(115200); SPI.begin();
  WiFi.mode(WIFI_AP); WiFi.softAP(ssid, password);
  
  ws.onEvent(onWsEvent); server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
  server.begin();

  mcp2515.reset();
  while (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) { delay(500); }
  mcp2515.setNormalMode();
}

void loop() {
  ws.cleanupClients();
  if (scanningDtc && millis() > dtcScanTimeout) { scanningDtc = false; ws.textAll("{\"type\":\"dtc\",\"codes\":[]}"); }

  if (!scanningDtc && (millis() - lastPollTime > POLL_INTERVAL)) {
    lastPollTime = millis();
    struct can_frame tx_frame; tx_frame.can_id = OBD_REQUEST_ID; tx_frame.can_dlc = 8;
    tx_frame.data[0] = 0x02; tx_frame.data[1] = 0x01; tx_frame.data[2] = healthPIDs[currentPidIndex];
    for (int i = 3; i < 8; i++) tx_frame.data[i] = 0x00;
    mcp2515.sendMessage(&tx_frame);
    currentPidIndex = (currentPidIndex + 1) % NUM_PIDS;
  }

  struct can_frame rx_frame;
  if (mcp2515.readMessage(&rx_frame) == MCP2515::ERROR_OK) {
    if (rx_frame.can_id >= ECU_RESPONSE_ID_MIN && rx_frame.can_id <= ECU_RESPONSE_ID_MAX && rx_frame.can_dlc >= 4) {
      if (rx_frame.data[1] == 0x43) {
        scanningDtc = false; String dtcJson = "{\"type\":\"dtc\",\"codes\":["; bool hasCode = false;
        if (rx_frame.can_dlc >= 8) {
          for (int i = 2; i <= 6; i += 2) {
            String code = decodeDtc(rx_frame.data[i], rx_frame.data[i + 1]);
            if (code.length() > 0) { if (hasCode) dtcJson += ","; dtcJson += "\"" + code + "\""; hasCode = true; }
          }
        }
        dtcJson += "]}"; ws.textAll(dtcJson);
      }
      else if (rx_frame.data[1] == 0x41) {
        uint8_t pid = rx_frame.data[2]; uint8_t A = rx_frame.data[3]; 
        uint8_t B = (rx_frame.can_dlc >= 5) ? rx_frame.data[4] : 0x00;
        bool dataUpdated = true;

        switch (pid) {
          case 0x0C: rpm = ((A * 256) + B) / 4; break;
          case 0x0D: speed = A; break;
          case 0x05: coolantTemp = A - 40; break;
          case 0x0F: intakeTemp = A - 40; break;
          case 0x10: mafFlow = ((A * 256.0) + B) / 100.0; break;
          case 0x04: engineLoad = (A * 100) / 255; break;
          case 0x11: throttle = (A * 100) / 255; break;
          case 0x2F: fuelLevel = (A * 100) / 255; break;
          case 0x0B: mapPressure = A; break;
          case 0x42: voltage = ((A * 256.0) + B) / 1000.0; break;
          // New Cases
          case 0x06: shortFuelTrim = (A / 1.28) - 100.0; break;
          case 0x07: longFuelTrim = (A / 1.28) - 100.0; break;
          case 0x0E: timingAdvance = (A / 2.0) - 64.0; break;
          case 0x14: o2Voltage = A / 200.0; break;
          case 0x1F: runTime = (A * 256) + B; break;
          case 0x33: baroPressure = A; break;
          case 0x46: ambientTemp = A - 40; break;
          case 0x5A: pedalPos = (A * 100) / 255; break;
          default: dataUpdated = false; break;
        }
        if (dataUpdated) notifyTelemetry();
      }
    }
  }
}
