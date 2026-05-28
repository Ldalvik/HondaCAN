// TODO:
// - Add screens that change via touch
// - Screen for temps- do color changing bar similar to RPM.
// - Add saved data over time, average temp, highest temp reached,
// - last warm up time for engine temp
// - Screen for fuel- fuel percent, fuel gallons, fuel bar, Distance-to-Empty
// - Screen for

void lcd_setup() {
  /* device specific setup */
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  lcd.init();
  lcd.initDMA();  // need both inits..?

  frame.setColorDepth(16);
  frame.createSprite(240, 240);

  // Startup screen
  frame.fillSprite(TFT_BLACK);

  frame.setTextDatum(MC_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(2);
  frame.setTextColor(TFT_WHITE);

  frame.drawString("Engine: " + String(CAN.CanData.PCM.EngineTemp) + " F", 120, 70);
  frame.drawString("Intake: " + String(CAN.CanData.PCM.IntakeTemp) + " F", 120, 110);
  frame.drawString("Fuel: " + String(CAN.CanData.BCM.FuelLevel) + "%", 120, 150);

  frame.pushSprite(0, 0);
  delay(3000);
  // lcd.startWrite();
  // lcd.writeCommand(0x36);
  // lcd.writeData(0x80 | 0x08);  // Mirror for windshield HUD
  // //lcd.writeData(0xC0 | 0x08);  // If horizontal flip needed
  // lcd.endWrite();
}

void drawMainScreen() {
  frame.fillSprite(TFT_BLACK);

  draw_rpm();
  draw_speed();

  draw_mpg();
  draw_gear();
  update_speed_limit();

  frame.pushSprite(0, 0);
}

void drawTemperatureScreen() {
  frame.fillSprite(TFT_BLACK);

  frame.drawString("Engine: " + String(CAN.CanData.PCM.EngineTemp) + " F", 120, 70);
  frame.drawString("Intake: " + String(CAN.CanData.PCM.IntakeTemp) + " F", 120, 110);

  frame.pushSprite(0, 0);
}

void drawFuelScreen() {
  frame.fillSprite(TFT_BLACK);

  draw_fuel_left();

  frame.pushSprite(0, 0);
}

/*
  CRUISE MAIN must be disabled for this to work.
  Press the CRUISE CANCEL button to toggle this feature.
  Use the SET AND RES buttons to increase and decrease the set "speed limit".
  When enabled, the color of your speed reading will change and also flash or fade depending on your speed.
*/

int SPEED_LIMIT = 55;
bool SPEED_LIMIT_MODE_ON = false;  // Tracks toggle state of CRUISE_CANCEL when CRUISE MAIN is disabled

void update_speed_limit() {
  static unsigned long lastCruiseChange = 0;
  static unsigned long lastCancelToggle = 0;
  static unsigned long showChangeUntil = 0;

  unsigned long now = millis();
  ScmButtons button = (ScmButtons)CAN.CanData.SCM.ScmButtons;
  if (!CAN.CanData.SCM.CruiseMainOn) {
    if (button == ScmButtons::CRUISE_CANCEL && now - lastCancelToggle >= 300) {
      SPEED_LIMIT_MODE_ON = !SPEED_LIMIT_MODE_ON;
      lastCancelToggle = now;
    }

    if (SPEED_LIMIT_MODE_ON && now - lastCruiseChange >= 100) {
      int change = 0;
      if (button == ScmButtons::CRUISE_RES) change = 5;
      if (button == ScmButtons::CRUISE_SET) change = -5;

      if (change) {
        SPEED_LIMIT = constrain(SPEED_LIMIT + change, 5, 100);
        lastCruiseChange = now;
        showChangeUntil = now + 500;
      }
    }
  }

  if (now < showChangeUntil) {
    frame.setTextDatum(MC_DATUM);
    frame.setTextFont(7);
    frame.setTextSize(1);
    frame.setTextColor(TFT_WHITE, TFT_BLACK);
    frame.drawString(String(SPEED_LIMIT), 120, 120);
  }
}

void draw_rpm() {
  int cx = 120, cy = 120;
  int startAngle = 180, endAngle = 360;
  int outer = 122;
  int inner = outer - 18;

  // switch (lcd.getRotation()) {
  //   case 0: startAngle = 180, endAngle = 360; break;
  //   case 1: startAngle = 270, endAngle = 450; break;
  //   case 2: startAngle = 0,   endAngle = 180; break;
  //   case 3: startAngle = 90,  endAngle = 270; break;
  // }

  static bool lastVtec = false;
  static int vtecStartRPM = 0;

  int rpm = CAN.CanData.PCM.EngineRPM;
  bool vtec = CAN.CanData.PCM.VTEC;
  const int maxRPM = 6800;  // K24W (9.5th gen Accord)

  if (vtec && !lastVtec) vtecStartRPM = rpm;
  if (!vtec) vtecStartRPM = 0;
  lastVtec = vtec;

  float p = constrain((float)rpm / maxRPM, 0.0f, 1.0f);
  int fillEnd = startAngle + (endAngle - startAngle) * p;

  uint16_t color;
  if (vtec) {
    float t = constrain((float)(rpm - vtecStartRPM) / (maxRPM - vtecStartRPM), 0.0f, 1.0f);
    color = lcd.color565(255, 165 * (1.0f - t), 0);
  } else {
    color = lcd.color565(255, 255 - 90 * p, 255 - 255 * p);
  }

  if (rpm >= maxRPM - 900 && (millis() / 100) % 2 == 0)
    color = TFT_RED;

  frame.fillArc(cx, cy, outer, inner, startAngle, endAngle, TFT_BLACK);
  frame.fillArc(cx, cy, outer, inner, startAngle, fillEnd, color);
}

void draw_speed() {
  int speed = CAN.CanData.PCM.Speed;

  static unsigned long lastBlinkTime = 0;
  static bool blinkOn = true;

  unsigned long now = millis();

  // Make 3-digit speeds fit better on screen
  int textSize = 3;
  int x = 120;
  if (speed > 99) {
    textSize = 2;
    x = 100;
  }

  uint16_t color = TFT_WHITE;

  if (SPEED_LIMIT_MODE_ON) {
    int delta = speed - SPEED_LIMIT;

    if (delta > 0) {
      int overSpeedLimit = 20;  // With a value of 20 and a speed limit of 55mph, at 75mph it will be max blink speed and color
      int over = min(delta, overSpeedLimit);
      float t = over / (float)overSpeedLimit;

      // white -> red as speed goes over the limit
      uint8_t gb = 255 * (1.0f - t);
      color = frame.color565(255, gb, gb);

      // Blink faster the farther over the speed limit you are
      // min = minimum blink time
      // blinkSpeed = how long the number blinks, e.g. higher values give a slower blink
      // blinkAccel = how quickly the number blinks based on speed (over the limit), higher values make blinking aggresive faster
      unsigned long blinkTime = 0;
      if (blinkOn) {  // visible
        int min = 100;
        int blinkSpeed = 500;
        int blinkAccel = 15;
        blinkTime = max(min, blinkSpeed - over * blinkAccel);
      } else {  // not visible
        int min = 50;
        int blinkSpeed = 500;
        int blinkAccel = 10;
        blinkTime = max(min, blinkSpeed - over * blinkAccel);
      }

      if (now - lastBlinkTime >= blinkTime) {
        blinkOn = !blinkOn;
        lastBlinkTime = now;
      }

      if (!blinkOn) color = TFT_BLACK;
    } else {
      blinkOn = true;
      lastBlinkTime = now;

      if (delta < 0) {
        int underSpeedLimit = 20;  // With a value of 20 and a speed limit of 55mph, at 35mph it will be max pulse effect
        int underMax = min(underSpeedLimit, SPEED_LIMIT);
        int under = min(-delta, underMax);
        float t = under / (float)underMax;

        // white -> red as speed drops below the limit
        uint8_t gb = 255 * (1.0f - t);

        // pulse when under the limit
        int pulseLength = 400.0f;     // lower value for faster pulse, higher value for longer pulse
        float pulseIntensity = 0.5f;  // higher value for subtle pulse, lower value for intense pulse

        float pulse = (sin(now / pulseLength) + 1.0f) * 0.5f;
        gb *= (1.0f - pulseIntensity * pulse);
        color = frame.color565(255, gb, gb);
      }
    }
  }

  frame.setTextDatum(MC_DATUM);
  frame.setTextFont(7);
  frame.setTextSize(textSize);
  frame.setTextColor(color);
  frame.drawString(String(speed), x, 120);
}

void draw_mpg() {

  frame.setTextDatum(TC_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(2);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);

  float mpg = CAN.CanData.PCM.TripDistance / CAN.CanData.PCM.TripFuelConsumed;
  if (CAN.CanData.PCM.TripFuelConsumed <= 0.0) mpg = 0.0;

  char mpgStr[16];
  sprintf(mpgStr, "%04.1fmpg", mpg);
  frame.drawString(mpgStr, 120, 30);
}

void draw_gear() {
  static int lastGear = -1;
  static int lastManualGear = -1;
  int gear = CAN.CanData.PCM.GearShifter;
  int manualGear = CAN.CanData.PCM.ManualGear;

  if (gear == lastGear || manualGear == lastManualGear) return;
  lastGear = gear;
  lastManualGear = manualGear;

  frame.setTextDatum(MR_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(3);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);

  switch (gear) {
    case GearShifter::GEAR_P: frame.drawString("P", 120, 220); break;
    case GearShifter::GEAR_R: frame.drawString("R", 120, 220); break;
    case GearShifter::GEAR_N: frame.drawString("N", 120, 220); break;
    case GearShifter::GEAR_D: frame.drawString("D", 120, 220); break;
    case GearShifter::GEAR_S: frame.drawString("S", 120, 220); break;
    case GearShifter::GEAR_L: frame.drawString("L", 120, 220); break;
    default: frame.drawString("-", 120, 220); break;
  }

  if (manualGear != 0)
    frame.drawString(String(manualGear), 128, 220);
}

void draw_fuel_left() {  // implement kallmans
  static float initialFuel = 0.0f;
  float currentFuel = (CAN.CanData.BCM.FuelLevel / 105.0f) * 17.2f;
  if (initialFuel == 0.0f) initialFuel = currentFuel;  // Initial fuel on boot

  if (CAN.CanData.PCM.EngineRPM == 0) initialFuel = currentFuel;  // Engine on, acc on. Allows live refueling

  float fuelConsumed = CAN.CanData.PCM.TripFuelConsumed;  // gallons
  float fuelLeft = constrain(initialFuel - fuelConsumed, 0.0f, 17.2f);
  float fuelPercent = (fuelLeft / 17.2f) * 100.0f;

  frame.setTextDatum(BC_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(2);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  frame.drawString(String(fuelPercent, 1) + "%", 120, 220);
}
