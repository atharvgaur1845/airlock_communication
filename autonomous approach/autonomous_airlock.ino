// pins
#define PRESENCE_FRONT    18
#define PRESENCE_MIDDLE   19  
#define PRESENCE_BACK     21
#define GATE_SAFETY_A     22
#define GATE_SAFETY_B     23
#define GATE_MOVING_A     25
#define GATE_MOVING_B     26
#define GATE_REQUEST_A    32
#define GATE_REQUEST_B    33
#define LED_DATA          2
class KalmanFilter1D {
private:
  float x;        
  float P;        
  float Q;       
  float R;       
  float K;        
  
public:
  KalmanFilter1D(float processNoise = 0.01f, float measurementNoise = 0.1f) {
    x = 0.0f;    
    P = 1.0f;     
    Q = processNoise;
    R = measurementNoise;
  }
  
  bool update(bool measurement) {
    P = P + Q;
    
    K = P / (P + R);
    x = x + K * (measurement - x);
    P = (1 - K) * P;

    return (x > 0.5f);
  }
  
  float getState() { return x; }
  void reset() { x = 0.0f; P = 1.0f; }
};
enum AirlockState {
  IDLE_BOTH_CLOSED,
  FRONT_ENTRY_REQUESTED,
  FRONT_GATE_OPENING,
  ROBOT_IN_MIDDLE,
  FRONT_GATE_CLOSING,
  BACK_GATE_OPENING,
  BACK_ENTRY_REQUESTED,
  BACK_GATE_CLOSING_REVERSE,
  FRONT_GATE_OPENING_REVERSE,
  CONFLICT_DETECTED,
  EMERGENCY_STOP
};
AirlockState currentState = IDLE_BOTH_CLOSED;
unsigned long stateStartTime = 0;
unsigned long conflictStartTime = 0;
bool frontPriorityFlag = true;
const unsigned long GATE_TIMEOUT = 10000;        
const unsigned long CONFLICT_TIMEOUT = 30000;    
const unsigned long STATE_DELAY = 100;

//kalman filters for IR sensors
KalmanFilter1D frontFilter(0.01f, 0.15f); 
KalmanFilter1D middleFilter(0.01f, 0.15f);
KalmanFilter1D backFilter(0.01f, 0.15f);

void setup() {
  Serial.begin(115200);
  
  pinMode(PRESENCE_FRONT, INPUT);
  pinMode(PRESENCE_MIDDLE, INPUT);
  pinMode(PRESENCE_BACK, INPUT);
  pinMode(GATE_SAFETY_A, INPUT);
  pinMode(GATE_SAFETY_B, INPUT);
  pinMode(GATE_MOVING_A, INPUT);
  pinMode(GATE_MOVING_B, INPUT);
  
  pinMode(GATE_REQUEST_A, OUTPUT);
  pinMode(GATE_REQUEST_B, OUTPUT);
  pinMode(LED_DATA, OUTPUT);

  digitalWrite(GATE_REQUEST_A, LOW);
  digitalWrite(GATE_REQUEST_B, LOW);
  digitalWrite(LED_DATA, LOW);

  currentState = IDLE_BOTH_CLOSED;
  stateStartTime = millis();
  
  Serial.println("Airlock system initialized");
}

void loop() {
  bool rawPresenceFront = digitalRead(PRESENCE_FRONT);
  bool rawPresenceMiddle = digitalRead(PRESENCE_MIDDLE);
  bool rawPresenceBack = digitalRead(PRESENCE_BACK);
  
  bool presenceFront = frontFilter.update(rawPresenceFront);
  bool presenceMiddle = middleFilter.update(rawPresenceMiddle);
  bool presenceBack = backFilter.update(rawPresenceBack);
  
  bool gateSafetyA = digitalRead(GATE_SAFETY_A);
  bool gateSafetyB = digitalRead(GATE_SAFETY_B);
  bool gateMovingA = digitalRead(GATE_MOVING_A);
  bool gateMovingB = digitalRead(GATE_MOVING_B);
 
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.printf("Raw: F=%d M=%d B=%d | Filtered: F=%d M=%d B=%d | States: %.2f %.2f %.2f\n",
                  rawPresenceFront, rawPresenceMiddle, rawPresenceBack,
                  presenceFront, presenceMiddle, presenceBack,
                  frontFilter.getState(), middleFilter.getState(), backFilter.getState());
    lastDebug = millis();
  }
  
  switch (currentState) {
    case IDLE_BOTH_CLOSED:
      handleIdleState(presenceFront, presenceMiddle, presenceBack);
      setLEDStatus(0, 255, 0); // Green - system ready
      break;
      
    case FRONT_ENTRY_REQUESTED:
      handleFrontEntryRequested(gateSafetyA, gateMovingA, presenceMiddle);
      setLEDStatus(0, 0, 255); // Blue - waiting for gate operation
      break;
      
    case FRONT_GATE_OPENING:
      handleFrontGateOpening(gateMovingA, presenceMiddle, presenceFront);
      setLEDStatus(255, 255, 0); // Yellow - gate operation in progress
      break;
      
    case ROBOT_IN_MIDDLE:
      handleRobotInMiddle(presenceMiddle, presenceFront, gateSafetyB, gateMovingA);
      setLEDStatus(0, 0, 255); // Blue - waiting for robot movement
      break;
      
    case FRONT_GATE_CLOSING:
      handleFrontGateClosing(gateMovingA, gateSafetyB);
      setLEDStatus(255, 255, 0); // Yellow - gate operation in progress
      break;
      
    case BACK_GATE_OPENING:
      handleBackGateOpening(gateMovingB, presenceBack, presenceMiddle);
      setLEDStatus(255, 255, 0); // Yellow - gate operation in progress
      break;
      
    case BACK_ENTRY_REQUESTED:
      handleBackEntryRequested(gateSafetyB, gateMovingB, presenceMiddle);
      setLEDStatus(0, 0, 255); // Blue - waiting for gate operation
      break;
      
    case BACK_GATE_CLOSING_REVERSE:
      handleBackGateClosingReverse(gateMovingB, gateSafetyA);
      setLEDStatus(255, 255, 0); // Yellow - gate operation in progress
      break;
      
    case FRONT_GATE_OPENING_REVERSE:
      handleFrontGateOpeningReverse(gateMovingA, presenceFront, presenceMiddle);
      setLEDStatus(255, 255, 0); // Yellow - gate operation in progress
      break;
      
    case CONFLICT_DETECTED:
      handleConflictDetected(presenceFront, presenceBack);
      setLEDStatus(255, 0, 255); // Purple - conflict detected
      break;
  }
  performSafetyChecks(gateSafetyA, gateSafetyB);
  
  delay(STATE_DELAY);
}
void handleIdleState(bool presenceFront, bool presenceMiddle, bool presenceBack) {
  if (presenceFront && presenceBack) {
    currentState = CONFLICT_DETECTED;
    conflictStartTime = millis();
    stateStartTime = millis();
    Serial.println("Conflict detected");
    return;
  }
  
  if (presenceFront && !presenceMiddle) {
    currentState = FRONT_ENTRY_REQUESTED;
    stateStartTime = millis();
    Serial.println("Front entry requested");
    return;
  }
  
  if (presenceBack && !presenceMiddle) {
    currentState = BACK_ENTRY_REQUESTED;
    stateStartTime = millis();
    Serial.println("Back entry requested");
    return;
  }
}

void handleFrontEntryRequested(bool gateSafetyA, bool gateMovingA, bool presenceMiddle) {
  if (!gateSafetyA && !presenceMiddle) {
    digitalWrite(GATE_REQUEST_A, HIGH);
    currentState = FRONT_GATE_OPENING;
    stateStartTime = millis();
    Serial.println("Opening front gate A");
  } else if (gateSafetyA) {
    Serial.println("Gate A safety sensor triggered - waiting");
  }
}

void handleFrontGateOpening(bool gateMovingA, bool presenceMiddle, bool presenceFront) {
  if (!gateMovingA) {
    if (presenceMiddle) {
      currentState = ROBOT_IN_MIDDLE;
      stateStartTime = millis();
      Serial.println("Robot entered middle zone");
    } else if (!presenceFront) {
      digitalWrite(GATE_REQUEST_A, LOW);
      currentState = IDLE_BOTH_CLOSED;
      stateStartTime = millis();
      Serial.println("Robot left - closing front gate");
    }
  }
  
  if (millis() - stateStartTime > GATE_TIMEOUT) {
    digitalWrite(GATE_REQUEST_A, LOW);
    currentState = IDLE_BOTH_CLOSED;
    stateStartTime = millis();
    Serial.println("Front gate opening timeout");
  }
}

void handleRobotInMiddle(bool presenceMiddle, bool presenceFront, bool gateSafetyB, bool gateMovingA) {
  if (!presenceMiddle && !presenceFront) {
    digitalWrite(GATE_REQUEST_A, LOW);
    currentState = FRONT_GATE_CLOSING;
    stateStartTime = millis();
    Serial.println("Robot left middle zone - closing front gate");
  }
}

void handleFrontGateClosing(bool gateMovingA, bool gateSafetyB) {
  if (!gateMovingA) {
    if (!gateSafetyB) {
      digitalWrite(GATE_REQUEST_B, HIGH);
      currentState = BACK_GATE_OPENING;
      stateStartTime = millis();
      Serial.println("Front gate closed - opening back gate B");
    }
  }
}

void handleBackGateOpening(bool gateMovingB, bool presenceBack, bool presenceMiddle) {
  if (!gateMovingB) {
    if (presenceBack && !presenceMiddle) {
      digitalWrite(GATE_REQUEST_B, LOW);
      currentState = IDLE_BOTH_CLOSED;
      stateStartTime = millis();
      Serial.println("Robot transit complete - closing back gate");
    }
  }
  
  if (millis() - stateStartTime > GATE_TIMEOUT) {
    digitalWrite(GATE_REQUEST_B, LOW);
    currentState = IDLE_BOTH_CLOSED;
    stateStartTime = millis();
    Serial.println("Back gate opening timeout");
  }
}

void handleBackEntryRequested(bool gateSafetyB, bool gateMovingB, bool presenceMiddle) {
  if (!gateSafetyB && !presenceMiddle) {
    digitalWrite(GATE_REQUEST_B, HIGH);
    currentState = BACK_GATE_CLOSING_REVERSE;
    stateStartTime = millis();
    Serial.println("Opening back gate B (reverse direction)");
  } else if (gateSafetyB) {
    Serial.println("Gate B safety sensor triggered - waiting");
  }
}

void handleBackGateClosingReverse(bool gateMovingB, bool gateSafetyA) {
  if (!gateMovingB) {
    if (!gateSafetyA) {
      digitalWrite(GATE_REQUEST_A, HIGH);
      currentState = FRONT_GATE_OPENING_REVERSE;
      stateStartTime = millis();
      Serial.println("Back gate closed - opening front gate A (reverse)");
    }
  }
}

void handleFrontGateOpeningReverse(bool gateMovingA, bool presenceFront, bool presenceMiddle) {
  if (!gateMovingA) {
    if (presenceFront && !presenceMiddle) {
      digitalWrite(GATE_REQUEST_A, LOW);
      currentState = IDLE_BOTH_CLOSED;
      stateStartTime = millis();
      Serial.println("Reverse robot transit complete - closing front gate");
    }
  }
  
  if (millis() - stateStartTime > GATE_TIMEOUT) {
    digitalWrite(GATE_REQUEST_A, LOW);
    currentState = IDLE_BOTH_CLOSED;
    stateStartTime = millis();
    Serial.println("Front gate opening timeout (reverse)");
  }
}

void handleConflictDetected(bool presenceFront, bool presenceBack) {
  if (!presenceFront || !presenceBack) {
    currentState = IDLE_BOTH_CLOSED;
    stateStartTime = millis();
    Serial.println("Conflict resolved - returning to idle");
    return;
  }
  
  if (millis() - conflictStartTime > CONFLICT_TIMEOUT) {
    if (frontPriorityFlag) {
      if (!presenceBack) {
        currentState = FRONT_ENTRY_REQUESTED;
        frontPriorityFlag = false; 
      }
    } else {
      if (!presenceFront) {
        currentState = BACK_ENTRY_REQUESTED;
        frontPriorityFlag = true;
      }
    }
    stateStartTime = millis();
    Serial.println("Conflict timeout");
  }
}

void performSafetyChecks(bool gateSafetyA, bool gateSafetyB) {
  if (gateSafetyA && gateSafetyB) {
    digitalWrite(GATE_REQUEST_A, LOW);
    digitalWrite(GATE_REQUEST_B, LOW);
    setLEDStatus(255, 0, 0);
    currentState = EMERGENCY_STOP;
    Serial.println("EMERGENCY");
  }
}

void setLEDStatus(int red, int green, int blue) {
  if (red > 0 || green > 0 || blue > 0) {
    digitalWrite(LED_DATA, HIGH);
  } else {
    digitalWrite(LED_DATA, LOW);
  }
}
