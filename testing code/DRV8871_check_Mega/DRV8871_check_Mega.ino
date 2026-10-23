// DRV8871 health check — Arduino Mega. IN1 = D9, IN2 = D10 (PWM pins), GND common with the driver.
// VM from the bench supply (7.5 V, current limit 0.1 A for steps 1-3, 0.5 A for the motor step).
// Serial Monitor 115200, "Newline". Commands:
//   f = forward (IN1 HIGH, IN2 LOW)     r = reverse (IN1 LOW, IN2 HIGH)
//   c = coast  (both LOW, outputs Hi-Z) b = brake  (both HIGH, outputs both LOW)
//   p NNN = forward PWM 0-255 on IN1 (IN2 LOW)    s = slow sweep 0 -> 255 forward
// Expected OUT voltages with NO motor (DMM, black on GND), VM = 7.5 V:
//   f: OUT1 ~VM, OUT2 ~0 V | r: OUT1 ~0 V, OUT2 ~VM | b: both ~0 V | c: both floating (DMM may read anything)
const uint8_t IN1 = 9, IN2 = 10;

void show(const char* what, const char* expect) {
  Serial.print("> "); Serial.print(what); Serial.print("   expect: "); Serial.println(expect);
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);      // coast at start
  Serial.println("DRV8871 check. f r c b | pNNN | s   (starts in COAST)");
}


void loop() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  if (c == "f")      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  show("FORWARD", "OUT1 ~VM, OUT2 ~0 V"); }
  else if (c == "r") { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); show("REVERSE", "OUT1 ~0 V, OUT2 ~VM"); }
  else if (c == "c") { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  show("COAST", "outputs off (floating)"); }
  else if (c == "b") { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); show("BRAKE", "OUT1 ~0 V, OUT2 ~0 V"); }
  else if (c[0] == 'p') {
    int d = constrain(c.substring(1).toInt(), 0, 255);
    digitalWrite(IN2, LOW); analogWrite(IN1, d);
    Serial.print("> PWM forward duty "); Serial.print(d); Serial.print(" / 255, expect OUT1 average ~");
    Serial.print(7.5 * d / 255.0, 1); Serial.println(" V (at VM 7.5 V)");
  }
  else if (c == "s") {
    digitalWrite(IN2, LOW);
    for (int d = 0; d <= 255; d += 15) { analogWrite(IN1, d); Serial.print("duty "); Serial.println(d); delay(800); }
    digitalWrite(IN1, LOW); Serial.println("> sweep done, coast");
  }
}
