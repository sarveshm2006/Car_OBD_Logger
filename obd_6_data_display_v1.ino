#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// --- Configuration ---
const char* ssid = "Aspire-Diagnostics";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
MCP2515 mcp2515(5);

// MCP2515 INT pin -> lets TaskCAN block instead of polling, and react to a
// new frame in microseconds instead of waiting up to 10ms for the next tick.
// GPIO4 is free on the WROOM-32 DevKit (no strapping/flash conflicts) --
// rewire this #define if you've wired INT elsewhere.
#define CAN_INT_PIN 4
static SemaphoreHandle_t canIntSemaphore;

void IRAM_ATTR canISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(canIntSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

const uint32_t OBD_REQUEST_ID = 0x7DF;
const uint32_t ECU_RESPONSE_ID_MIN = 0x7E8;
const uint32_t ECU_RESPONSE_ID_MAX = 0x7EF;

// --- Shared Telemetry ---
// One struct instead of 18 loose globals: TaskWebBroadcast takes the mutex
// once, does a single memcpy-style struct assignment (a few microseconds),
// and releases it -- TaskCAN is never kept waiting.
struct TelemetryData {
  int rpm, speed, coolantTemp, intakeTemp;
  int engineLoad, throttle, fuelLevel, mapPressure;
  float mafFlow, voltage;
  float shortFuelTrim, longFuelTrim, timingAdvance;
  float o2Voltage;
  int runTime, baroPressure, ambientTemp, pedalPos;
};

static TelemetryData telemetry = {0};
static SemaphoreHandle_t dataMutex;

// --- DTC state ---
// bool writes are inherently atomic on the ESP32 (word-aligned), so
// scanningDtc doesn't need the mutex; the payload buffer does.
static volatile bool scanningDtc = false;
static volatile bool dtcDataReady = false;
static char dtcJsonPayload[160] = "{\"type\":\"dtc\",\"codes\":[]}";

const uint8_t healthPIDs[] = {
  0x0C, 0x0D, 0x05, 0x0F, 0x10, 0x04, 0x11, 0x2F, 0x0B, 0x42,
  0x06, 0x07, 0x0E, 0x14, 0x1F, 0x33, 0x46, 0x5A
};
const int NUM_PIDS = sizeof(healthPIDs) / sizeof(healthPIDs[0]);

// --- Raw CAN sniffer ---
// Every frame that hits the bus gets pushed here, regardless of ID --
// unlike the OBD decode path above, this doesn't filter to 0x7E8-0x7EF,
// so you also see broadcast traffic from other modules (ABS, BCM, etc.
// if they share this physical bus). A FreeRTOS queue decouples capture
// (TaskCAN, must never block) from broadcast (TaskWebBroadcast).
struct RawFrame {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t ts; // millis() at capture
};
static QueueHandle_t rawFrameQueue;

// --- HTML Dashboard (unchanged) ---
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
    .raw-panel { max-width: 1000px; margin: 15px auto; background: #1a1a1f; border-radius: 10px; border: 1px solid #333; padding: 15px; text-align: left; }
    .raw-hint { font-size: 0.85em; color: #aaa; margin-bottom: 8px; }
    .raw-table { width: 100%; border-collapse: collapse; font-family: monospace; font-size: 0.85em; }
    .raw-table th { color: #00d2ff; text-align: left; padding: 4px 8px; }
    .raw-table td { padding: 4px 8px; border-bottom: 1px solid #2a2a2a; transition: color 0.3s; }
    .raw-table tr.fresh td { color: #00ff88; }
  </style>
</head>
<body>
  <h2>ASPIRE TELEMETRY (RTOS)</h2>
  <div id="status">Connecting to vehicle...</div>

  <div class="control-section">
    <button id="dtcBtn" onclick="requestDtcScan()">Scan Fault Codes</button>
    <button id="recordBtn" onclick="toggleRecording()">Start Recording Data</button>
    <button id="downloadBtn" onclick="downloadCSV()" disabled>Download CSV</button>
    <div class="log-status" id="logCount">Logged: 0 rows</div>
    <div class="dtc-results" id="dtcList"></div>
  </div>

  <div class="control-section">
    <button id="rawToggleBtn" onclick="toggleRawView()">Show Raw CAN Monitor</button>
    <button id="rawRecordBtn" onclick="toggleRawRecording()">Start Raw Log</button>
    <button id="rawDownloadBtn" onclick="downloadRawCSV()" disabled>Download Raw CSV</button>
    <div class="log-status" id="rawLogCount">Raw logged: 0 rows</div>
  </div>

  <div id="rawPanel" class="raw-panel" style="display:none;">
    <div class="raw-hint">Every ID seen on the bus, updated live -- watch which row changes color when you toggle something in the car (brake pedal, headlights, turn signal) to figure out what it means.</div>
    <table class="raw-table">
      <thead><tr><th>ID</th><th>DLC</th><th>Data (hex)</th><th>Count</th><th>Last seen</th></tr></thead>
      <tbody id="rawTableBody"></tbody>
    </table>
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

    var rawIdMap = {};       // CAN ID -> {count, tr} for the live sniffer table
    var isRawRecording = false;
    var rawLogData = [];
    var rawCsvHeaders = "Timestamp,ID,DLC,Data\n";

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
          } else if (msg.type === "raw") {
            handleRawFrames(msg.frames);
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

    function toggleRawView() {
      var panel = document.getElementById("rawPanel");
      var btn = document.getElementById("rawToggleBtn");
      var show = panel.style.display === "none";
      panel.style.display = show ? "block" : "none";
      btn.innerText = show ? "Hide Raw CAN Monitor" : "Show Raw CAN Monitor";
    }

    function handleRawFrames(frames) {
      var tbody = document.getElementById("rawTableBody");
      frames.forEach(function(f) {
        var ts = new Date().toLocaleTimeString();
        if (isRawRecording) rawLogData.push(`${ts},${f.id},${f.dlc},${f.data}\n`);

        var entry = rawIdMap[f.id];
        if (!entry) {
          var tr = document.createElement("tr");
          tr.innerHTML = "<td>" + f.id + "</td><td></td><td></td><td></td><td></td>";
          tbody.appendChild(tr);
          entry = { count: 0, tr: tr };
          rawIdMap[f.id] = entry;
        }
        entry.count++;
        var cells = entry.tr.children;
        cells[1].innerText = f.dlc;
        cells[2].innerText = f.data;
        cells[3].innerText = entry.count;
        cells[4].innerText = ts;
        entry.tr.classList.add("fresh");
        setTimeout((function(rowTr){ return function(){ rowTr.classList.remove("fresh"); }; })(entry.tr), 400);
      });
      if (isRawRecording) {
        document.getElementById("rawLogCount").innerText = "Raw logged: " + (rawLogData.length - 1) + " rows";
      }
    }

    function toggleRawRecording() {
      isRawRecording = !isRawRecording;
      var btn = document.getElementById("rawRecordBtn");
      var dBtn = document.getElementById("rawDownloadBtn");
      if (isRawRecording) {
        btn.innerText = "Stop Raw Log"; btn.style.background = "#ff9900"; dBtn.disabled = true;
        if (rawLogData.length === 0) rawLogData.push(rawCsvHeaders);
      } else {
        btn.innerText = "Start Raw Log"; btn.style.background = "#4CAF50"; dBtn.disabled = (rawLogData.length <= 1);
      }
    }

    function downloadRawCSV() {
      if (rawLogData.length <= 1) return;
      var blob = new Blob(rawLogData, { type: 'text/csv' });
      var url = window.URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.setAttribute('hidden', ''); a.setAttribute('href', url); a.setAttribute('download', 'Aspire_Raw_CAN_Log.csv');
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
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

// --- Helpers ---
// Writes a DTC code like "P0301" into a caller-owned buffer. No String,
// no heap allocation -- safe to call every scan without fragmenting the heap.
void decodeDtc(uint8_t highByte, uint8_t lowByte, char* out, size_t outLen) {
  out[0] = '\0';
  if (highByte == 0x00 && lowByte == 0x00) return;
  char category;
  switch (highByte >> 6) {
    case 0: category = 'P'; break;
    case 1: category = 'C'; break;
    case 2: category = 'B'; break;
    default: category = 'U'; break;
  }
  snprintf(out, outLen, "%c%X%X%X%X", category, (highByte >> 4) & 0x03, highByte & 0x0F, lowByte >> 4, lowByte & 0x0F);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      if (strstr((char*)data, "\"scan_dtc\"") != NULL) {
        scanningDtc = true; // Signal the CAN task to perform a DTC scan
      }
    }
  }
}

// ==============================================================================
// RTOS TASK 1: CAN BUS COMMUNICATION (Runs on Core 1 / APP_CPU)
// Isolated from the WiFi driver's own internal tasks, which live on Core 0.
// ==============================================================================
void TaskCAN(void *pvParameters) {
  TickType_t lastPoll = 0;
  int pidIndex = 0;
  TickType_t dtcTimeout = 0;
  char codeBuf[6];

  for (;;) {
    struct can_frame rx_frame;
    while (mcp2515.readMessage(&rx_frame) == MCP2515::ERROR_OK) {

      // Sniffer tap: every frame off the bus goes here, unfiltered -- this
      // runs before (and independent of) the OBD-response check below, so
      // it never affects the existing decode path.
      RawFrame rf;
      rf.id = rx_frame.can_id;
      rf.dlc = rx_frame.can_dlc;
      memcpy(rf.data, rx_frame.data, 8);
      rf.ts = millis();
      xQueueSend(rawFrameQueue, &rf, 0); // non-blocking: best-effort, drop if full rather than ever stalling CAN reads

      if (rx_frame.can_id >= ECU_RESPONSE_ID_MIN && rx_frame.can_id <= ECU_RESPONSE_ID_MAX && rx_frame.can_dlc >= 4) {

        // Mode 03 (DTC) response
        if (rx_frame.data[1] == 0x43) {
          scanningDtc = false;
          char localBuf[160];
          int pos = snprintf(localBuf, sizeof(localBuf), "{\"type\":\"dtc\",\"codes\":[");
          bool hasCode = false;
          if (rx_frame.can_dlc >= 8) {
            for (int i = 2; i <= 6; i += 2) {
              decodeDtc(rx_frame.data[i], rx_frame.data[i + 1], codeBuf, sizeof(codeBuf));
              if (codeBuf[0] != '\0') {
                pos += snprintf(localBuf + pos, sizeof(localBuf) - pos, "%s\"%s\"", hasCode ? "," : "", codeBuf);
                hasCode = true;
              }
            }
          }
          snprintf(localBuf + pos, sizeof(localBuf) - pos, "]}");

          // Only the final buffer copy needs the lock
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          strncpy(dtcJsonPayload, localBuf, sizeof(dtcJsonPayload) - 1);
          dtcJsonPayload[sizeof(dtcJsonPayload) - 1] = '\0';
          dtcDataReady = true;
          xSemaphoreGive(dataMutex);
        }

        // Mode 01 (telemetry) response
        else if (rx_frame.data[1] == 0x41) {
          uint8_t pid = rx_frame.data[2];
          uint8_t A = rx_frame.data[3];
          uint8_t B = (rx_frame.can_dlc >= 5) ? rx_frame.data[4] : 0x00;

          xSemaphoreTake(dataMutex, portMAX_DELAY);
          switch (pid) {
            case 0x0C: telemetry.rpm = ((A * 256) + B) / 4; break;
            case 0x0D: telemetry.speed = A; break;
            case 0x05: telemetry.coolantTemp = A - 40; break;
            case 0x0F: telemetry.intakeTemp = A - 40; break;
            case 0x10: telemetry.mafFlow = ((A * 256.0f) + B) / 100.0f; break;
            case 0x04: telemetry.engineLoad = (A * 100) / 255; break;
            case 0x11: telemetry.throttle = (A * 100) / 255; break;
            case 0x2F: telemetry.fuelLevel = (A * 100) / 255; break;
            case 0x0B: telemetry.mapPressure = A; break;
            case 0x42: telemetry.voltage = ((A * 256.0f) + B) / 1000.0f; break;
            case 0x06: telemetry.shortFuelTrim = (A / 1.28f) - 100.0f; break;
            case 0x07: telemetry.longFuelTrim = (A / 1.28f) - 100.0f; break;
            case 0x0E: telemetry.timingAdvance = (A / 2.0f) - 64.0f; break;
            case 0x14: telemetry.o2Voltage = A / 200.0f; break;
            case 0x1F: telemetry.runTime = (A * 256) + B; break;
            case 0x33: telemetry.baroPressure = A; break;
            case 0x46: telemetry.ambientTemp = A - 40; break;
            case 0x5A: telemetry.pedalPos = (A * 100) / 255; break;
          }
          xSemaphoreGive(dataMutex);
        }
      }
    }

    // Timeout recovery for DTC scans
    if (scanningDtc && (xTaskGetTickCount() > dtcTimeout)) {
      if (dtcTimeout == 0) {
        struct can_frame tx_frame;
        tx_frame.can_id = OBD_REQUEST_ID;
        tx_frame.can_dlc = 8;
        tx_frame.data[0] = 0x01; tx_frame.data[1] = 0x03;
        for (int i = 2; i < 8; i++) tx_frame.data[i] = 0x00;
        mcp2515.sendMessage(&tx_frame);
        dtcTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
      } else {
        scanningDtc = false;
        dtcTimeout = 0;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        strcpy(dtcJsonPayload, "{\"type\":\"dtc\",\"codes\":[]}");
        dtcDataReady = true;
        xSemaphoreGive(dataMutex);
      }
    }

    // Transmit telemetry request
    if (!scanningDtc && (xTaskGetTickCount() - lastPoll > pdMS_TO_TICKS(100))) {
      lastPoll = xTaskGetTickCount();
      struct can_frame tx_frame;
      tx_frame.can_id = OBD_REQUEST_ID;
      tx_frame.can_dlc = 8;
      tx_frame.data[0] = 0x02; tx_frame.data[1] = 0x01; tx_frame.data[2] = healthPIDs[pidIndex];
      for (int i = 3; i < 8; i++) tx_frame.data[i] = 0x00;
      mcp2515.sendMessage(&tx_frame);
      pidIndex = (pidIndex + 1) % NUM_PIDS;
    }

    // Block until either the MCP2515 INT line fires (new frame waiting --
    // wakes almost instantly) or 10ms elapses (keeps the 100ms TX poll and
    // DTC-timeout checks running exactly as before, even with no traffic).
    // Still yields the core each iteration, so the watchdog stays fed.
    xSemaphoreTake(canIntSemaphore, pdMS_TO_TICKS(10));
  }
}

// ==============================================================================
// RTOS TASK 2: NETWORK & WEBSOCKET UPDATES (Runs on Core 0 / PRO_CPU)
// Snapshot under lock, format + send outside it -- TaskCAN is never blocked
// waiting on JSON string building.
// ==============================================================================
void TaskWebBroadcast(void *pvParameters) {
  static char jsonBuf[380]; // static: allocated once, not re-pushed onto the stack every loop

  for (;;) {
    // 1. Send DTC result if ready
    if (dtcDataReady) {
      char localDtc[160];
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      strncpy(localDtc, dtcJsonPayload, sizeof(localDtc) - 1);
      localDtc[sizeof(localDtc) - 1] = '\0';
      dtcDataReady = false;
      xSemaphoreGive(dataMutex);
      ws.textAll(localDtc);
    }

    // 2. Snapshot telemetry -- one struct copy, held for microseconds
    TelemetryData snap;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    snap = telemetry;
    xSemaphoreGive(dataMutex);

    // 3. Format + broadcast OUTSIDE the lock
    snprintf(jsonBuf, sizeof(jsonBuf),
      "{\"type\":\"telemetry\",\"rpm\":%d,\"speed\":%d,\"coolant\":%d,"
      "\"intake\":%d,\"maf\":%.2f,\"load\":%d,\"thr\":%d,\"pedal\":%d,"
      "\"fuel\":%d,\"map\":%d,\"baro\":%d,\"volt\":%.2f,\"sft\":%.2f,"
      "\"lft\":%.2f,\"timing\":%.2f,\"o2\":%.2f,\"ambient\":%d,\"runtime\":%d}",
      snap.rpm, snap.speed, snap.coolantTemp, snap.intakeTemp, snap.mafFlow,
      snap.engineLoad, snap.throttle, snap.pedalPos, snap.fuelLevel,
      snap.mapPressure, snap.baroPressure, snap.voltage, snap.shortFuelTrim,
      snap.longFuelTrim, snap.timingAdvance, snap.o2Voltage, snap.ambientTemp,
      snap.runTime);

    ws.textAll(jsonBuf);

    // 4. Drain the raw-frame sniffer queue, batching ~20 frames per WS
    // message so a chatty bus doesn't spam one message per frame. Sends
    // up to 5 batches per cycle so a backlog gets cleared instead of
    // growing unbounded.
    {
      static char rawBuf[1400];
      RawFrame rf;
      for (int batch = 0; batch < 5; batch++) {
        int pos = snprintf(rawBuf, sizeof(rawBuf), "{\"type\":\"raw\",\"frames\":[");
        int count = 0;
        while (count < 20 && xQueueReceive(rawFrameQueue, &rf, 0) == pdTRUE) {
          char dataHex[17];
          for (int i = 0; i < rf.dlc && i < 8; i++) snprintf(dataHex + i * 2, 3, "%02X", rf.data[i]);
          dataHex[rf.dlc <= 8 ? rf.dlc * 2 : 16] = '\0';
          pos += snprintf(rawBuf + pos, sizeof(rawBuf) - pos,
                           "%s{\"id\":\"%03X\",\"dlc\":%d,\"data\":\"%s\",\"ts\":%lu}",
                           count ? "," : "", (unsigned int)rf.id, rf.dlc, dataHex, (unsigned long)rf.ts);
          count++;
        }
        if (count == 0) break; // queue empty
        snprintf(rawBuf + pos, sizeof(rawBuf) - pos, "]}");
        ws.textAll(rawBuf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // Steady 5 Hz dashboard refresh
  }
}

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  SPI.begin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false); // disable modem-sleep power saving -> lower, steadier WS latency

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
  server.begin();

  mcp2515.reset();
  while (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) { delay(500); }
  mcp2515.setNormalMode();

  dataMutex = xSemaphoreCreateMutex();
  rawFrameQueue = xQueueCreate(64, sizeof(RawFrame)); // holds ~64 sniffed frames between broadcasts

  // MCP2515 INT is active-low, open-drain by default -> pull-up + FALLING edge
  canIntSemaphore = xSemaphoreCreateBinary();
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);

  // Core 1 (APP_CPU): CAN/SPI only
  xTaskCreatePinnedToCore(TaskCAN, "CAN_Task", 4096, NULL, 5, NULL, 1);

  // Core 0 (PRO_CPU): sits beside the WiFi/AsyncTCP internals it talks to
  xTaskCreatePinnedToCore(TaskWebBroadcast, "Web_Task", 4096, NULL, 2, NULL, 0);
}

// --- Main Loop (background maintenance only) ---
void loop() {
  ws.cleanupClients();
  vTaskDelay(pdMS_TO_TICKS(500));
}#include <WiFi.h>
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
