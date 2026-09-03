#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// --- Wi-Fi Credentials ---
// Set these to your own network before flashing.
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Create Async Web Server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Data variables (packed types)
int16_t x = 0, y = 0;
uint16_t pressure = 0;

// Rate limiting & heartbeat
unsigned long lastSend = 0;   // last send timestamp (ms)
unsigned long lastPing = 0;   // last cleanup timestamp (ms)

// Throttle settings
const unsigned long MIN_SEND_INTERVAL_MS = 5;  // super fast test ~200Hz

// Random bounds for dummy data
const int X_MIN = 0, X_MAX = 1920;
const int Y_MIN = 0, Y_MAX = 1080;
const int P_MIN = 0, P_MAX = 1023;

// ------------------------------------------------------------
// HTML page served by ESP32 (binary receiver JS)
// ------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
        <meta charset="utf-8">
        <title>XP-Pen Real-time Stream (Binary)</title>
        <style>
        body { font-family: Arial, sans-serif; background: #0f172a; color: #e2e8f0;
                display:flex; flex-direction:column; justify-content:center; align-items:center; height:100vh; margin:0; }
        h2 { color:#38bdf8; margin-bottom:1rem; }
        p { font-size:1.2rem; background:#1e293b; padding:1rem 2rem; border-radius:.5rem;
            box-shadow: 0 0 10px rgba(56,189,248,0.3); }
        small { color:#94a3b8; margin-top:0.5rem; }
        </style>
    </head>
    <body>
        <h2>XP-Pen Real-time Stream</h2>
        <p id="d">Waiting for data...</p>
        <small id="s">Status: disconnected</small>

        <script>
        let ws;
        function connectWS() {
            const status = document.getElementById('s');
            ws = new WebSocket(`ws://${location.host}/ws`);
            ws.binaryType = "arraybuffer";

            ws.onopen = (e) => {
            console.log("[ws] open", e);
            status.innerText = "Status: connected";
            };
            ws.onerror = (e) => {
            console.warn("[ws] error", e);
            status.innerText = "Status: error";
            };
            ws.onmessage = (event) => {
            const dv = new DataView(event.data);
            const x = dv.getInt16(0, true);
            const y = dv.getInt16(2, true);
            const p = dv.getUint16(4, true);
            document.getElementById("d").innerText = `X=${x} | Y=${y} | Pressure=${p}`;
            };
            ws.onclose = (e) => {
            console.log("[ws] closed", e);
            status.innerText = "Status: disconnected — reconnecting...";
            setTimeout(connectWS, 2000);
            };
        }
        connectWS();
        </script>
    </body>
</html>
)rawliteral";

// ------------------------------------------------------------
// WebSocket Event Handler (logs connects/disconnects)
// ------------------------------------------------------------
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
               AwsEventType type, void * arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
        Serial.printf("Client #%u connected\n", client->id());
        break;
        case WS_EVT_DISCONNECT:
        Serial.printf("Client #%u disconnected\n", client->id());
        break;
        default:
        break;
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    ws.onEvent(onWsEvent);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    server.addHandler(&ws);
    server.begin();

    Serial.println("WebSocket server started with dummy fast data.");
}

void loop() {
    unsigned long now = millis();

    // Only send if at least one client is connected
    if (ws.count() > 0) {
        if (now - lastSend >= MIN_SEND_INTERVAL_MS) {
        // Generate dummy random data (simulate API)
        x = random(X_MIN, X_MAX);
        y = random(Y_MIN, Y_MAX);
        pressure = random(P_MIN, P_MAX);

        // Prepare binary packet
        uint8_t buf[6];
        memcpy(&buf[0], &x, 2);
        memcpy(&buf[2], &y, 2);
        memcpy(&buf[4], &pressure, 2);

        if (ws.availableForWriteAll() > 0) {
            ws.binaryAll(buf, sizeof(buf));
        } else {
            Serial.println("⚠️ Skipped frame: buffer full");
        }

        lastSend = now;
        }
    }

    // Cleanup clients every 10s
    if (now - lastPing >= 10000) {
        ws.cleanupClients();
        lastPing = now;
        Serial.printf("Clients: %u\n", ws.count());
    }

    yield();
}
