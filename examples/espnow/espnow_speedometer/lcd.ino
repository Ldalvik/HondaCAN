LGFX lcd;
LGFX_Sprite frame(&lcd);

void lcd_setup() {
  /* device specific setup */
  pinMode(3, OUTPUT); 
  digitalWrite(3, HIGH); 
  lcd.init(); 
  lcd.initDMA(); // need both inits..?

  frame.setColorDepth(16);
  frame.createSprite(240, 240);

  lcd.startWrite();
  lcd.writeCommand(0x36);
  lcd.writeData(0x80 | 0x08);  // Mirror for windshield HUD
  //lcd.writeData(0xC0 | 0x08);  // If horizontal flip needed
  lcd.endWrite();
}

void update() {
  frame.fillSprite(TFT_BLACK);

  draw_rpm();
  draw_speed();
  draw_mpg();
  //draw_current_gear();
  draw_fuel_left();
  update_speed_limit();

  frame.pushSprite(0, 0);
}

int speedLimit = 55;
bool speedLimitModeEnabled = false;  // Tracks toggle state of CRUISE_CANCEL

void update_speed_limit() {
  static unsigned long lastCruiseChange = 0;
  static unsigned long lastCancelToggle = 0;
  static unsigned long showChangeUntil = 0;
  unsigned long now = millis();

  int change = 0;

  if (now - lastCruiseChange >= 100) {
    if (!CAN.telemetry.cruise_main_on) {
      if (CAN.telemetry.scm_buttons == ScmButtons::CRUISE_RES) change = +5;
      if (CAN.telemetry.scm_buttons == ScmButtons::CRUISE_SET) change = -5;

      if (CAN.telemetry.scm_buttons == ScmButtons::CRUISE_CANCEL && (now - lastCancelToggle >= 300)) {
        speedLimitModeEnabled = !speedLimitModeEnabled;
        lastCancelToggle = now;
      }

      if (change != 0) {
        speedLimit = constrain(speedLimit + change, 5, 100);
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
    frame.drawString(String(speedLimit), 120, 120);
  }
}

void draw_rpm() {
  int cx = 120;
  int cy = 120;
  int outer = 122;
  int inner = outer - 14;
  int startAngle = 180;
  int endAngle = 360;

  // switch (lcd.getRotation()) {
  //   case 0: startAngle = 180, endAngle = 360; break;
  //   case 1: startAngle = 270, endAngle = 450; break;
  //   case 2: startAngle = 0,   endAngle = 180; break;
  //   case 3: startAngle = 90,  endAngle = 270; break;
  // }

  int MAX_RPM = 6800;
  int VTEC_ENGAGE = 4000;
  float progress = (float)CAN.telemetry.rpm / MAX_RPM;
  int fillEnd = startAngle + (int)((endAngle - startAngle) * progress);

  // Orange at VTEC engagement, turns red as it gets close to max RPM
  // For now VTEC is hardcoded to 2016 Accord (K24W1) need access to F-CAN for CAN ID
  uint16_t barColor = TFT_WHITE;
  if (CAN.telemetry.rpm > VTEC_ENGAGE) {
    float t = constrain((float)(CAN.telemetry.rpm - VTEC_ENGAGE) / (MAX_RPM - VTEC_ENGAGE), 0.0f, 1.0f);
    uint8_t g = 165 * (1.0f - t);
    barColor = lcd.color565(255, g, 0);
  }

  frame.fillArc(cx, cy, outer, inner, startAngle, endAngle, TFT_BLACK);
  frame.fillArc(cx, cy, outer, inner, startAngle, fillEnd, barColor);
}

unsigned long lastBlinkTime = 0;
bool blinkOn = true;

void draw_speed() {
  // Resize for 3 digit numbers
  int textSize = 3, x = 120;
  if (CAN.telemetry.speed > 99) {
    textSize = 2;
    x = 100;
  }

  uint16_t color = TFT_WHITE;
  if (speedLimitModeEnabled) {
    int delta = CAN.telemetry.speed - speedLimit;

    unsigned long currentMillis = millis();
    unsigned long onTime, offTime;

    if (delta > 0) {
      // white -> red, full red at 20 over
      int over = min(delta, 20);
      float t = over / 20.0f;
      uint8_t r = 255;
      uint8_t g = 255 * (1.0f - t);
      uint8_t b = 255 * (1.0f - t);
      color = frame.color565(r, g, b);

      // blink when over speed limit
      onTime = 500 - (over * 15);  // blinks faster as you go farther over the limit
      offTime = 500 - (over * 10);
      if (onTime < 100) onTime = 100;
      if (offTime < 50) offTime = 50;

      if (blinkOn && currentMillis - lastBlinkTime >= onTime) {
        blinkOn = false;
        lastBlinkTime = currentMillis;
      } else if (!blinkOn && currentMillis - lastBlinkTime >= offTime) {
        blinkOn = true;
        lastBlinkTime = currentMillis;
      }

      if (!blinkOn) color = TFT_BLACK;

    } else if (delta < 0) {
      // white -> red, full red at 20 under
      int underMax = min(20, speedLimit);  // cap at 0 if speedLimit < 20
      int under = min(-delta, underMax);
      float t = under / float(underMax);

      uint8_t r = 255;
      uint8_t g = 255 * (1.0f - t);
      uint8_t b = 255 * (1.0f - t);

      // Smooth pulse (sinusoidal)
      float pulse = (sin(currentMillis / 400.0) + 1.0f) / 2.0f;  
      g = g * (1.0f - 0.5f * pulse);
      b = b * (1.0f - 0.5f * pulse);

      color = frame.color565(r, g, b);
    } else  color = TFT_WHITE;
    
  }

  frame.setTextDatum(MC_DATUM);
  frame.setTextFont(7);
  frame.setTextSize(textSize);
  frame.setTextColor(color);
  frame.drawString(String(CAN.telemetry.speed), x, 120);
}

void draw_mpg() {
  frame.setTextDatum(TC_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(2);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);

  float mpg = CAN.telemetry.trip_distance / CAN.telemetry.fuel_consumed;
  if(CAN.telemetry.fuel_consumed <= 0.0) mpg = 0.0;

  char mpgStr[16];
  sprintf(mpgStr, "%04.1fmpg", mpg);
  frame.drawString(mpgStr, 120, 30);
}

void draw_current_gear() {
  frame.setTextDatum(MR_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(3);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);

  switch (CAN.telemetry.current_gear) {
    case GearShifter::GEAR_P: frame.drawString("P", 233, 150); break;
    case GearShifter::GEAR_R: frame.drawString("R", 233, 150); break;
    case GearShifter::GEAR_N: frame.drawString("N", 233, 150); break;
    case GearShifter::GEAR_D: frame.drawString("D", 233, 150); break;
    case GearShifter::GEAR_S: frame.drawString("S", 233, 150); break;
    case GearShifter::GEAR_L: frame.drawString("L", 233, 150); break;
  }
}

void draw_fuel_left() { // implement kallmans
  float currentFuel = (CAN.telemetry.fuel_level / 105.0f) * 17.2f;
  static float initialFuel = 0.0f;
  if (initialFuel == 0.0f) initialFuel = currentFuel;  // Initial fuel on boot

  if (CAN.telemetry.rpm < 400) initialFuel = currentFuel;  // Engine on, acc on. Allows live refueling

  float fuelConsumed = CAN.telemetry.fuel_consumed / 3785411.784f;
  float fuelLeft = constrain(initialFuel - fuelConsumed, 0.0f, 17.2f);
  float fuelPercent = (fuelLeft / 17.2f) * 100.0f;

  frame.setTextDatum(BC_DATUM);
  frame.setTextFont(1);
  frame.setTextSize(2);
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  frame.drawString(String(fuelPercent, 1) + "%", 120, 220);
}
