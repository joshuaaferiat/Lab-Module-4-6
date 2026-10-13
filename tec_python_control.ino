// Phys 39 Module 3 - Part 6
// Arduino serial-command control sketch
//
// A0: thermistor (averaged)
// Pin 9 / Pin 10: H-bridge control signals
// No trim pot, no pin 11 switch.
//
// Receives commands from the Python GUI:
//   SET PWM 120 DIR HEAT
//   SET PWM 45 DIR COOL
//
// Prints measurement lines:
//   Temperature (C): 27.73, Time (s): 645.06, PWM: 120, Direction input: 1, Active PWM pin: 9, Heat/Cool: 1
//
// Heat/Cool = 1 only for observed HEATING.
// Heat/Cool = 0 only for observed COOLING.
// This is open-loop manual control. No feedback control.

const int THERMISTOR_PIN = A0;
const int HBRIDGE_PIN_1 = 9;
const int HBRIDGE_PIN_2 = 10;

const int ADC_SAMPLES = 1000;
const unsigned long PRINT_INTERVAL_MS = 200;

// Thermistor constants -- Module 2 values.
const float SERIES_RESISTOR = 100000.0;
const float NOMINAL_RESISTANCE = 100000.0;
const float NOMINAL_TEMPERATURE_C = 25.0;
const float BETA_COEFFICIENT = 4540.0;
const float ADC_MAX = 1023.0;

// ------------------------------------------------------------------
// EXPERIMENTALLY VERIFIED PART 3 MAPPING
// ------------------------------------------------------------------
// Set these two flags after your Part 3 experiment.
//
// HEAT_ACTIVE_PIN  = the Arduino pin that must output PWM when HEATING.
// COOL_ACTIVE_PIN  = the Arduino pin that must output PWM when COOLING.
//
// In the Part 3 sketch, pin 11 HIGH -> pin 9 PWM, pin 11 LOW -> pin 10 PWM.
// If your Part 3 test showed that pin 9 heated and pin 10 cooled, keep:
//   HEAT_ACTIVE_PIN = 9
//   COOL_ACTIVE_PIN = 10
// If your experiment showed the opposite, swap these two values.
const int HEAT_ACTIVE_PIN = 9;
const int COOL_ACTIVE_PIN = 10;

// ------------------------------------------------------------------
// STATE
// ------------------------------------------------------------------
int currentPwm = 0;          // 0-255, starts at 0
bool currentIsHeat = true;   // true = HEAT, false = COOL, starts as HEAT

unsigned long startTime;
unsigned long lastPrint = 0;

// Serial line buffer for incoming commands.
String commandBuffer = "";

// ------------------------------------------------------------------
// THERMISTOR MEASUREMENT (same as Part 2 / Part 3)
// ------------------------------------------------------------------
float readTemperatureC() {
  long sum = 0;

  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(THERMISTOR_PIN);
    delayMicroseconds(200);
  }

  float adc = sum / (float)ADC_SAMPLES;

  if (adc <= 0 || adc >= ADC_MAX) {
    return NAN;
  }

  float resistance = SERIES_RESISTOR * adc / (ADC_MAX - adc);

  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE_C + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

// ------------------------------------------------------------------
// SAFETY: apply PWM to the H-bridge
// ------------------------------------------------------------------
// Only one H-bridge input gets PWM at a time. The other is held LOW.
// This prevents both inputs being active at once.
void applyOutput() {
  int pwmPin;
  int lowPin;

  if (currentIsHeat) {
    pwmPin = HEAT_ACTIVE_PIN;
    lowPin = COOL_ACTIVE_PIN;
  } else {
    pwmPin = COOL_ACTIVE_PIN;
    lowPin = HEAT_ACTIVE_PIN;
  }

  // Make sure both pins are digital outputs.
  pinMode(HBRIDGE_PIN_1, OUTPUT);
  pinMode(HBRIDGE_PIN_2, OUTPUT);

  // Hold the inactive pin LOW, then write PWM on the active pin.
  digitalWrite(lowPin, LOW);
  analogWrite(pwmPin, currentPwm);
}

// ------------------------------------------------------------------
// SAFETY: zero the output
// ------------------------------------------------------------------
void zeroOutput() {
  currentPwm = 0;
  digitalWrite(HBRIDGE_PIN_1, LOW);
  digitalWrite(HBRIDGE_PIN_2, LOW);
  analogWrite(HBRIDGE_PIN_1, 0);
  analogWrite(HBRIDGE_PIN_2, 0);
}

// ------------------------------------------------------------------
// PARSER: handle one command line
// ------------------------------------------------------------------
// Accepted format (case-insensitive on keywords):
//   SET PWM <0-255> DIR HEAT
//   SET PWM <0-255> DIR COOL
//
// Any malformed or unknown command returns PWM to 0.
void handleCommand(String line) {
  line.trim();
  line.toUpperCase();

  // Reject empty lines silently.
  if (line.length() == 0) {
    return;
  }

  // Tokenize by spaces.
  // Expected tokens:
  //   0: SET
  //   1: PWM
  //   2: <number>
  //   3: DIR
  //   4: HEAT | COOL
  int t0 = line.indexOf(' ');
  if (t0 < 0) { zeroOutput(); return; }
  String tok0 = line.substring(0, t0);

  int t1 = line.indexOf(' ', t0 + 1);
  if (t1 < 0) { zeroOutput(); return; }
  String tok1 = line.substring(t0 + 1, t1);

  int t2 = line.indexOf(' ', t1 + 1);
  if (t2 < 0) { zeroOutput(); return; }
  String tok2 = line.substring(t1 + 1, t2);

  int t3 = line.indexOf(' ', t2 + 1);
  if (t3 < 0) { zeroOutput(); return; }
  String tok3 = line.substring(t2 + 1, t3);

  String tok4 = line.substring(t3 + 1);
  tok4.trim();

  // Validate keywords.
  if (tok0 != "SET") { zeroOutput(); return; }
  if (tok1 != "PWM") { zeroOutput(); return; }
  if (tok3 != "DIR") { zeroOutput(); return; }
  if (tok4 != "HEAT" && tok4 != "COOL") { zeroOutput(); return; }

  // Validate and parse PWM value.
  // The string must be all digits, else malformed.
  if (tok2.length() == 0) { zeroOutput(); return; }
  for (unsigned int i = 0; i < tok2.length(); i++) {
    if (!isDigit(tok2.charAt(i))) {
      zeroOutput();
      return;
    }
  }

  long pwmValue = tok2.toInt();

  // Clamp numeric PWM to 0-255.
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255;

  // Commit the new state.
  currentPwm = (int)pwmValue;
  currentIsHeat = (tok4 == "HEAT");

  applyOutput();
}

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  pinMode(HBRIDGE_PIN_1, OUTPUT);
  pinMode(HBRIDGE_PIN_2, OUTPUT);

  // Safety: PWM starts at zero.
  zeroOutput();

  Serial.begin(9600);

  startTime = millis();
  lastPrint = 0;
  commandBuffer.reserve(64);

  Serial.println("Part 6 serial-command TEC control");
  Serial.println("Commands: SET PWM <0-255> DIR HEAT | COOL");
}

// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {
  // ---- read incoming serial commands ----
  // We read one character at a time, accumulate into commandBuffer,
  // and process on newline.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        handleCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      if (commandBuffer.length() < 64) {
        commandBuffer += c;
      } else {
        // Line too long: drop it and zero output.
        commandBuffer = "";
        zeroOutput();
      }
    }
  }

  // ---- print the measurement line on a timer ----
  unsigned long now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    float tempC = readTemperatureC();
    float elapsed = (now - startTime) / 1000.0;

    int activePin = currentIsHeat ? HEAT_ACTIVE_PIN : COOL_ACTIVE_PIN;
    int heatCool = currentIsHeat ? 1 : 0;
    int dirInput = currentIsHeat ? 1 : 0;

    Serial.print("Temperature (C): ");
    if (isnan(tempC)) {
      Serial.print("nan");
    } else {
      Serial.print(tempC, 2);
    }

    Serial.print(", Time (s): ");
    Serial.print(elapsed, 2);

    Serial.print(", PWM: ");
    Serial.print(currentPwm);

    Serial.print(", Direction input: ");
    Serial.print(dirInput);

    Serial.print(", Active PWM pin: ");
    Serial.print(activePin);

    Serial.print(", Heat/Cool: ");
    Serial.println(heatCool);
  }
}
