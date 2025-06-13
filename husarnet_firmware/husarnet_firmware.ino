#include <Husarnet.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Husarnet info
const char* husarnetJoinCode = "YOUR_JOIN_CODE_HERE";
const char* husarnetHostname = "esp32-airlock-controller";

struct AirlockStatus {
    bool gateA_open;
    bool gateB_open;
    AirlockState current_state;
    bool safety_lockout;
    unsigned long timestamp;
};

void setup() {
    // starting husarnet
    Husarnet.selfHostedSetup("default");
    Husarnet.join(husarnetJoinCode, husarnetHostname);
    //dumy function for starting hardware
    initializeAirlockHardware();
    
    //setup HTTP server for robot communication
    setupWebServer();
}
