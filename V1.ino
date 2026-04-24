#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// --- Pins (ESP32-S3) ---
const int motorIN1 = 16; 
const int motorIN2 = 17; 
const int motorENA = 18; 
const int feedbackPotPin = 35;  // Internal MG996R Potentiometer (Position Feedback)
const int inputPin = 4;        // External Control Signal (RC PWM or Analog Volts)

// --- PWM Hardware Settings (ESP32 v3.x API) ---
const int pwmFreq = 20000;      // 20kHz to remove audible motor whine
const int pwmResolution = 8;    // 8-bit resolution (0-255)

// --- PID & Control Variables ---
float Kp, Ki, Kd, tolerance, maxDeg, gearRatio;
float targetDeg = 0.0, currentDeg = 0.0, currentError = 0.0;
float integral = 0, lastError = 0;
bool servoEnabled = true;
int controlMode = 0; // 0: Web, 1: RC PWM, 2: Analog Input

// --- Limits & Filtering ---
float minLimitDeg = 0.0;   
float filteredFeedback = 0;
float filterAlpha = 0.15; // 0.1 (very smooth) to 0.8 (fast/noisy)

// --- RC PWM Variables ---
volatile unsigned long pulseStart = 0;
volatile int pulseWidth = 0;

Preferences prefs; 
AsyncWebServer server(80);

// --- Fast Interrupt for RC PWM Input ---
void IRAM_ATTR handlePWM() {
  if (digitalRead(inputPin) == HIGH) pulseStart = micros();
  else pulseWidth = micros() - pulseStart;
}

// --- Read Internal Servo Potentiometer ---
void updateFeedback() {
  int rawADC = analogRead(feedbackPotPin);
  
  // EMA Filter to smooth out internal potentiometer noise
  filteredFeedback = (filterAlpha * rawADC) + ((1.0 - filterAlpha) * filteredFeedback);

  // Convert to Degrees (assuming 12-bit ADC and direct drive gear ratio)
  currentDeg = (filteredFeedback / 4095.0) * maxDeg * gearRatio;
}

void resetPID() {
  integral = 0;
  lastError = 0;
}

void driveMotor(int output) {
  float error = abs(targetDeg - currentDeg);

  // 1. HARDWARE LIMIT CHECKS (Protect the internal pot)
  if ((currentDeg <= minLimitDeg && output < 0) || 
      (currentDeg >= maxDeg && output > 0)) {
    ledcWrite(motorIN1, 0); 
    ledcWrite(motorIN2, 0);
    return; 
  }

  // 2. DEADZONE: Stop motor if within tolerance to prevent jitter
  if (!servoEnabled || error < tolerance) {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, 0);
    return;
  }

  // 3. Minimum starting power (overcome DC motor static friction)
  int speed = constrain(abs(output), 45, 255); 
  
  // 4. Drive via Hardware PWM (v3.x API)
  if (output > 0) {
    ledcWrite(motorIN1, speed);
    ledcWrite(motorIN2, 0);
  } else {
    ledcWrite(motorIN1, 0);
    ledcWrite(motorIN2, speed);
  }
}

void runPID() {
  currentError = targetDeg - currentDeg;
  
  // Anti-windup for Integral
  integral += currentError;
  integral = constrain(integral, -100, 100); 
  
  float derivative = currentError - lastError;
  int output = (Kp * currentError) + (Ki * integral) + (Kd * derivative);
  
  driveMotor(output);
  lastError = currentError;
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>Pro Servo Panel</title>
<style>
  body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }
  .card { background: #2d2d2d; padding: 20px; border-radius: 15px; display: inline-block; width: 420px; border: 1px solid #444; }
  .btn { padding: 12px; margin: 5px; cursor: pointer; border: none; border-radius: 8px; font-weight: bold; width: 45%; }
  .on { background: #2ecc71; color: white; }
  .save { background: #3498db; color: white; width: 93%; }
  input, select { width: 85%; padding: 10px; margin: 10px 0; border-radius: 5px; background: #444; color: white; border: none; }
  .row { display: flex; justify-content: space-around; align-items: center; }
  .row input { width: 60px; }
  .stats { display: flex; justify-content: space-between; background: #111; padding: 10px; border-radius: 8px; margin-bottom: 15px; }
  .stat-box { width: 30%; }
  .stat-val { font-size: 1.5em; font-weight: bold; }
  .err-val { color: #e74c3c; }
</style></head>
<body>
  <div class="card">
    <h2>Smart Servo Pro</h2>
    <div class="stats">
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Target&deg;</div><div class="stat-val" id="tgt">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Current&deg;</div><div class="stat-val" id="pos" style="color:#2ecc71;">0.0</div></div>
      <div class="stat-box"><div style="font-size:12px; color:#aaa;">Error&deg;</div><div class="stat-val err-val" id="err">0.0</div></div>
    </div>
    <hr>
    <h4>Control Mode & Limits</h4>
    <select id="mode" style="width:90%;">
      <option value="0">Web Dashboard</option>
      <option value="1">External RC PWM</option>
      <option value="2">Analog Signal</option>
    </select>
    <div class="row"> Ratio: <input type="text" id="ratio"> Max Limit&deg;: <input type="text" id="mDeg"> </div>
    <hr>
    <h4>PID Tuning</h4>
    <div class="row"> P:<input type="text" id="p"> I:<input type="text" id="i"> D:<input type="text" id="d"> </div>
    Tolerance&deg;: <input type="text" id="t" style="width:60px;">
    <button class="btn save" onclick="saveSettings()">APPLY & SAVE</button>
    <hr>
    <input type="number" id="moveVal" placeholder="Degrees" style="width: 40%;">
    <button class="btn on" style="width:45%; background:#9b59b6" onclick="move()">WEB MOVE</button>
  </div>
<script>
  function move() { fetch('/move?val=' + document.getElementById('moveVal').value); }
  function saveSettings() {
    const p = document.getElementById('p').value, i = document.getElementById('i').value, d = document.getElementById('d').value;
    const t = document.getElementById('t').value, m = document.getElementById('mode').value, max = document.getElementById('mDeg').value, r = document.getElementById('ratio').value;
    fetch(`/save?p=${p}&i=${i}&d=${d}&t=${t}&m=${m}&max=${max}&r=${r}`).then(() => alert("Saved Successfully!"));
  }
  fetch('/getparams').then(r => r.json()).then(data => {
    document.getElementById('p').value = data.p; document.getElementById('i').value = data.i;
    document.getElementById('d').value = data.d; document.getElementById('t').value = data.t;
    document.getElementById('mode').value = data.m; document.getElementById('mDeg').value = data.max;
    document.getElementById('ratio').value = data.r;
  });
  setInterval(() => { 
    fetch('/getlive').then(r => r.json()).then(data => { 
      document.getElementById('pos').innerText = data.pos; 
      document.getElementById('tgt').innerText = data.tgt; 
      document.getElementById('err').innerText = data.err; 
    }); 
  }, 200);
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  
  pinMode(motorENA, OUTPUT); 
  digitalWrite(motorENA, HIGH);
  pinMode(inputPin, INPUT);
  pinMode(feedbackPotPin, INPUT);

  // --- Initialize LEDC PWM (ESP32 v3.x API) ---
  ledcAttach(motorIN1, pwmFreq, pwmResolution);
  ledcAttach(motorIN2, pwmFreq, pwmResolution);

  // Load Saved Data
  prefs.begin("servo-data", false);
  Kp = prefs.getFloat("kp", 2.0);
  Ki = prefs.getFloat("ki", 0.0);
  Kd = prefs.getFloat("kd", 0.1);
  tolerance = prefs.getFloat("tol", 1.0);
  maxDeg = prefs.getFloat("maxDeg", 180.0); // Standard MG996R range
  gearRatio = prefs.getFloat("ratio", 1.0);
  controlMode = prefs.getInt("mode", 0);

  // INITIAL POTENTIOMETER READ
  filteredFeedback = analogRead(feedbackPotPin); 
  updateFeedback();
  
  // SYNC target to prevent startup jump
  targetDeg = currentDeg; 

  if (controlMode == 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);

  WiFi.softAP("SmartServo_Pro", "");

  // API Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *f){ f->send_P(200, "text/html", index_html); });
  
  server.on("/save", [](AsyncWebServerRequest *f){
    Kp = f->getParam("p")->value().toFloat();
    Ki = f->getParam("i")->value().toFloat();
    Kd = f->getParam("d")->value().toFloat();
    tolerance = f->getParam("t")->value().toFloat();
    maxDeg = f->getParam("max")->value().toFloat();
    gearRatio = f->getParam("r")->value().toFloat();
    int newMode = f->getParam("m")->value().toInt();

    if (newMode == 1 && controlMode != 1) attachInterrupt(digitalPinToInterrupt(inputPin), handlePWM, CHANGE);
    else if (newMode != 1 && controlMode == 1) detachInterrupt(digitalPinToInterrupt(inputPin));
    
    controlMode = newMode;
    resetPID();

    prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
    prefs.putFloat("tol", tolerance); prefs.putInt("mode", controlMode);
    prefs.putFloat("maxDeg", maxDeg); prefs.putFloat("ratio", gearRatio);
    f->send(200);
  });

  server.on("/move", [](AsyncWebServerRequest *f){ 
    targetDeg = f->getParam("val")->value().toFloat(); 
    targetDeg = constrain(targetDeg, minLimitDeg, maxDeg);
    f->send(200); 
  });
  
  server.on("/getparams", [](AsyncWebServerRequest *f){
    String json = "{\"p\":"+String(Kp)+",\"i\":"+String(Ki)+",\"d\":"+String(Kd)+",\"t\":"+String(tolerance)+",\"m\":"+String(controlMode)+",\"max\":"+String(maxDeg)+",\"r\":"+String(gearRatio)+"}";
    f->send(200, "application/json", json);
  });
  
  server.on("/getlive", [](AsyncWebServerRequest *f){ 
    String json = "{\"pos\":\""+String(currentDeg, 2)+"\",\"tgt\":\""+String(targetDeg, 2)+"\",\"err\":\""+String(abs(currentError), 2)+"\"}";
    f->send(200, "application/json", json); 
  });

  server.begin();
}

void loop() {
  updateFeedback();

  float rawInput = targetDeg;

  if (controlMode == 1) { // RC PWM MODE
    if (pulseWidth > 800 && pulseWidth < 2200) {
      // Map standard RC signals (1000us - 2000us) to physical limits
      rawInput = map(pulseWidth, 1000, 2000, 0, maxDeg);
    }
  } 
  else if (controlMode == 2) { // ANALOG SIGNAL MODE
    int analogVal = 0;
    for(int i=0; i<8; i++) analogVal += analogRead(inputPin); // Multi-sample average
    rawInput = map(analogVal / 8, 0, 4095, 0, maxDeg);
  }

  // Apply EMA smoothing to external targets to prevent aggressive snapping
  if (controlMode != 0) {
    targetDeg = (filterAlpha * rawInput) + ((1.0 - filterAlpha) * targetDeg);
  }

  // Final safety clamp before running PID
  targetDeg = constrain(targetDeg, minLimitDeg, maxDeg);

  runPID();
  delay(2); 
}