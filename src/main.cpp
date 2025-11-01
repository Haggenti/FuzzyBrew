// #pragma GCC optimize("Os") // code optimisation controls - "O2" & "O3" code performance, "Os" code size
#include <Arduino.h>
#include <Wire.h>
#include "I2C_LCD.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "TM1637.h"
#include <Blinkenlight.h>
#include <EEPROM.h>
//#include "EMAFilter.h"
#include <Adafruit_NeoPixel.h>

inline void swap_vals(double &x, double &y)
{
  double tmp = x;
  x = y;
  y = tmp;
}

#define test_mode 0 // Set to 1 to enable test mode, 0 for normal operation
//  ==================== LCD & DISPLAY SETTINGS ====================
#define LCD_SPACE_SYMBOL 0x20 // Space symbol from LCD ROM (GDM2004D datasheet p.9)
#define LCD_COLS 20
#define LCD_ROWS 4
#define I2C_BUS_SPEED 400000 // I2C bus speed 400000Hz
#define ARROW_LEFT 0x7F
#define ARROW_RIGHT 0x7E

#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

// ==================== LED DEFINITIONS ====================
constexpr uint8_t LED_PIN = 11;
#define NUM_LEDS 1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ==================== HARDWARE PIN DEFINITIONS ====================
constexpr uint8_t PIN_BTN = 4;
constexpr uint8_t CLK = 5;          // TM1637 CLK
constexpr uint8_t DIO = 6;          // TM1637 DIO
constexpr uint8_t ONE_WIRE_BUS = 7; // Temperature sensor data wire
constexpr uint8_t BUZZER_PIN = 10;
constexpr uint8_t SSR = 9;

// ==================== COMMUNICATION SETTINGS ====================
constexpr uint16_t SERIAL_BAUDRATE = 9600;

// ==================== TEMPERATURE SETTINGS ====================
constexpr uint8_t TEMPERATURE_PRECISION = 11;
float offset_temp = 0.0; // Calibration offset

// Temperature control limits
constexpr float MIN_SETPOINT = 10, MAX_SETPOINT = 99;
float Setpoint = 45;
float prev_temp = 0.0, temperature = 0.0;
float calculated_power = 0.0;
float delta = 0.0, old_delta = 0.0, pwm = 0.0, old_pwm = -1.0;

// Sampling settings
constexpr uint32_t SAMPLE_PERIOD = 500;
unsigned lastTempRequest = 0;
uint32_t lastSampleTime = 0;
unsigned delayInMillis = 500; // Wait time for temp reading
double T_filtre, dT_dt, dT_dt_prev = 0.0;
double dt = 0.75; // intervalle entre mesures

// ==================== PWM & CONTROL SETTINGS ====================
float OVERSHOOT_X = -0.1;
float NEAR_LIMIT = 0.1;
float FAR_LIMIT = 1.5;
float PWM_FAR = 90.0, PWM_NEAR_OFFSET = 0.0, PWM_N = 0.0;
uint32_t PWM_PERIOD = 5000;
bool pwmstate = false;
uint32_t onPWM_PERIOD = 0;

float MAINT_A = 0.01455f; // Coefficient A for maintenance power
float MAINT_B = 0.08593f; // Coefficient B for maintenance power
float MAINT_C = 6.8386f;  // Coefficient C for maintenance power

// ==================== ADAPTIVE MAINTENANCE CORRECTION ====================
// Per-temperature-band correction (approx. per 10°C from ~40°C to ~110°C)
float pwm_corr[8] = {0};
// Snapshot of last-saved corrections for autosave delta tracking
float pwm_corr_saved[8] = {0};
// Last time we autosaved adaptive corrections
unsigned long last_adapt_save = 0;
bool adaptive_enable = true; // can be toggled in settings
inline uint8_t band_index_for_setpoint(float sp)
{
  int idx = int((sp + 5.0f) / 10.0f) - 3; // 35–45 -> 0, 45–55 -> 1, ...
  if (idx < 0)
    idx = 0;
  if (idx > 7)
    idx = 7;
  return (uint8_t)idx;
}

// ==================== MENU & UI SETTINGS ====================
uint8_t menu_select = 0;
uint8_t encbutton_state;
unsigned long buttontick = 0;
const char *menu1[] = {"BREW", "SETTINGS", "MEMORY", NULL};
const char *menu2[] = {"LOAD", "SAVE", "DEFAULTS", "RESET ADAPT", "BACK", NULL};
// const char *menu3[] = {"START", "BACK", "EXIT", NULL}; // Unused
const char *confirm_menu[] = {"YES", "NO", NULL};

// ==================== TIMER VARIABLES ====================
uint32_t chronostart = 0, timelapse = 0;
uint8_t h = 0, m = 0, s = 0, old_s, old_m, old_h;
uint32_t startime = 0;
bool timer_active = false, querry_temp = false, error = false;
unsigned long stabilityStartTime = 0;
bool stabilityCheck = false;
float last_temp_for_rate = 0.0;
unsigned long last_rate_time = 0;
float heat_rate = 0.0;

// ==================== STATE VARIABLES ====================
bool emafilter = true;
bool click_prev = true;
float push_time = 0.0;
bool fineajust = 0;
byte mash_mode = 3;

// ==================== OBJECT INITIALIZATION ====================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
I2C_LCD lcd(39);
//EMAFilter tempFilter(0.2); // Initial alpha = 0.2
TM1637 tm;
Blinkenlight buzz(BUZZER_PIN);

// ==================== FUNCTION PROTOTYPES ====================
void buttonstate();
void red();
void black();

// ==================== CUSTOM CHARACTERS ====================
uint8_t delta_char[8] = {
    B00000, B00000, B00100, B01010, B11111, B00000, B00000, B00000};

uint8_t arrowUp[8] = {
    0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};

uint8_t arrowDown[8] = {
    0b00100, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100, 0b00000};

// ==================== EEPROM MANAGEMENT ====================
void writeEEPROM()
{
  int address = 0;
  EEPROM.put(address, offset_temp);
  address += sizeof(offset_temp);
  EEPROM.put(address, OVERSHOOT_X);
  address += sizeof(OVERSHOOT_X);
  EEPROM.put(address, NEAR_LIMIT);
  address += sizeof(NEAR_LIMIT);
  EEPROM.put(address, FAR_LIMIT);
  address += sizeof(FAR_LIMIT);
  EEPROM.put(address, PWM_FAR);
  address += sizeof(PWM_FAR);
  EEPROM.put(address, PWM_NEAR_OFFSET);
  address += sizeof(PWM_NEAR_OFFSET);
  EEPROM.put(address, PWM_PERIOD);
  address += sizeof(PWM_PERIOD);
  EEPROM.put(address, emafilter);
  address += sizeof(emafilter);
  EEPROM.put(address, MAINT_A);
  address += sizeof(MAINT_A);
  EEPROM.put(address, MAINT_B);
  address += sizeof(MAINT_B);
  EEPROM.put(address, MAINT_C);
  address += sizeof(MAINT_C);
  // Save adaptive settings
  for (int i = 0; i < 8; i++)
  {
    EEPROM.put(address, pwm_corr[i]);
    address += sizeof(pwm_corr[i]);
  }
  EEPROM.put(address, adaptive_enable);
}

void readEEPROM()
{
  int address = 0;
  EEPROM.get(address, offset_temp);
  address += sizeof(offset_temp);
  EEPROM.get(address, OVERSHOOT_X);
  address += sizeof(OVERSHOOT_X);
  EEPROM.get(address, NEAR_LIMIT);
  address += sizeof(NEAR_LIMIT);
  EEPROM.get(address, FAR_LIMIT);
  address += sizeof(FAR_LIMIT);
  EEPROM.get(address, PWM_FAR);
  address += sizeof(PWM_FAR);
  EEPROM.get(address, PWM_NEAR_OFFSET);
  address += sizeof(PWM_NEAR_OFFSET);
  EEPROM.get(address, PWM_PERIOD);
  address += sizeof(PWM_PERIOD);
  EEPROM.get(address, emafilter);
  address += sizeof(emafilter);
  EEPROM.get(address, MAINT_A);
  address += sizeof(MAINT_A);
  EEPROM.get(address, MAINT_B);
  address += sizeof(MAINT_B);
  EEPROM.get(address, MAINT_C);
  address += sizeof(MAINT_C);
  // Read adaptive settings (sanitize if EEPROM not yet initialized)
  for (int i = 0; i < 8; i++)
  {
    EEPROM.get(address, pwm_corr[i]);
    address += sizeof(pwm_corr[i]);
    if (isnan(pwm_corr[i]) || pwm_corr[i] < -30.0f || pwm_corr[i] > 30.0f)
      pwm_corr[i] = 0.0f;
  }
  EEPROM.get(address, adaptive_enable);
}

struct TempFilter
{
  // Paramètres EMA adaptatif
  double alpha_min = 0.08;
  double alpha_max = 0.35;
  double k = 0.054;      // pente EMA
  // Limitation de pente exprimée en vitesse (°C/s). Ex-équivalent ~0.20 °C par 0.75 s -> 0.27 °C/s
  double maxRate = 0.27; // °C/s

  // Paramètre EMA pour la dérivée
  double alpha_d = 0.12;

  // État interne
  double a = NAN, b = NAN, c = NAN; // tampon médiane
  double ema_y = NAN;
  double prev_rl = NAN;
  double prev_Tf = NAN;
  double deriv_v = 0.0;
  int filled = 0; // nombre d'échantillons valides accumulés (<=3)

  inline double median3(double x, double y, double z)
  {
    if (x > y)
      swap_vals(x, y);
    if (y > z)
      swap_vals(y, z);
    if (x > y)
      swap_vals(x, y);

    return y;
  }

  // Limitation de pente dépendante de dt (°C/s * dt)
  inline double rateLimitDt(double prev, double now, double dt)
  {
    if (isnan(prev))
      return now;
    double d = now - prev;
    double maxStep = maxRate * (dt > 0 ? dt : 0.75);
    if (d > maxStep)
      return prev + maxStep;
    if (d < -maxStep)
      return prev - maxStep;
    return now;
  }

  // Appel à chaque nouvelle mesure
  void process(double T_raw, double T_set, double dt,
               double &T_filtre, double &dT_dt)
  {
    // 0) Remplissage initial et anti-glitch (médiane dès 3 valeurs)
    if (filled == 0)
    {
      a = b = c = T_raw;
      filled = 1;
    }
    else
    {
      a = b;
      b = c;
      c = T_raw;
      if (filled < 3)
        filled++;
    }
    double T_med = (filled < 3) ? T_raw : median3(a, b, c);

    // 1) Limitation de pente dépendante de dt
    double T_rl = rateLimitDt(prev_rl, T_med, dt);
    prev_rl = T_rl;

    // 2) EMA adaptatif
    double e = T_set - T_rl;
    double alpha = alpha_min + k * fabs(e);
    if (alpha < alpha_min)
      alpha = alpha_min;
    if (alpha > alpha_max)
      alpha = alpha_max;

    if (isnan(ema_y))
      ema_y = T_rl;
    ema_y = alpha * T_rl + (1.0 - alpha) * ema_y;
    T_filtre = ema_y;

    // 3) Dérivée filtrée (avec dt mesuré)
    if (!isnan(prev_Tf) && dt > 0)
    {
      double raw_d = (T_filtre - prev_Tf) / dt;
      deriv_v = alpha_d * raw_d + (1 - alpha_d) * deriv_v;
    }
    prev_Tf = T_filtre;
    dT_dt = deriv_v;
  }
};

TempFilter filt;

// ==================== ENCODER MANAGEMENT ====================
constexpr byte pinA = 2;                 // Hardware interrupt pin (digital pin 2)
constexpr byte pinB = 3;                 // Hardware interrupt pin (digital pin 3)
volatile byte aFlag = 0;                 // Indicates rising edge on pinA (encoder detent reached)
volatile byte bFlag = 0;                 // Indicates rising edge on pinB (encoder detent reached in opposite direction)
volatile byte reading = 0;               // Stores direct values from interrupt pins before validating movement
volatile int encPos = 0;                 // Encoder position counter
volatile bool axcel = 1;                 // acceleration activation
volatile unsigned long lastTurnTime = 0; // Temps du dernier changement d'état
volatile int accelFactor = 1;            // Facteur d'accélération

void updateEncoder(int direction, unsigned long currentTime)
{
  unsigned long timeDiff = currentTime - lastTurnTime; // Calcul du temps entre deux impulsions
  unsigned long maxdif = 250;
  if (timeDiff > maxdif)
  {
    timeDiff = maxdif;
  }
  lastTurnTime = currentTime;

  // Ajuste le facteur d’accélération en fonction de la vitesse de rotation
  if (axcel)
  {
    accelFactor = maxdif / timeDiff;
  }
  else
  {
    accelFactor = 1;
  }
  encPos = direction * accelFactor;
}

void resetSensor()
{
  sensors.begin(); // Réinitialise le bus OneWire
  if (sensors.getAddress(tempDeviceAddress, 0))
  {
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    sensors.setWaitForConversion(false);
  }
}

void print_space(byte sp)
{
  for (size_t i = 0; i < sp; i++)
  {
    lcd.print(F(" "));
  }
}
void print_deg()
{
  lcd.print(F("\xDF"
              "C")); // Print °C symbol
}

void PinA()
{
  unsigned long currentTime = millis(); // Récupère le temps actuel
  reading = PIND & 0xC;

  if (reading == B00001100 && aFlag)
  {
    updateEncoder(1, currentTime);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100)
  {
    bFlag = 1;
  }
}

void PinB()
{
  unsigned long currentTime = millis();
  reading = PIND & 0xC;

  if (reading == B00001100 && bFlag)
  {
    updateEncoder(-1, currentTime);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000)
  {
    aFlag = 1;
  }
}

float calc_maintain_pow(float stp)
{
  float result = MAINT_A * expf(MAINT_B * stp) + MAINT_C;
  return roundf(result * 10) / 10.0;
}

void factory_rst()
{
  lcd.clear();
  lcd.print(F("Factory Reset"));
  offset_temp = 0.0;
  OVERSHOOT_X = -0.1;
  NEAR_LIMIT = 0.1;
  FAR_LIMIT = 1.5;
  PWM_FAR = 90;
  PWM_NEAR_OFFSET = 0.0;
  PWM_PERIOD = 5000;
  emafilter = true;
  MAINT_A = 0.01455f;
  MAINT_B = 0.08593f;
  MAINT_C = 6.8386f;
  for (int i = 0; i < 8; i++)
    pwm_corr[i] = 0.0f;
  adaptive_enable = true;
  delay(1000);
  lcd.print(F("OK!"));
}
void check_counter()
{
  static bool wasStable = false;
  // Reference setpoint for timer resets on setpoint changes (>1.0°C)
  static float sp_ref_for_timer = NAN;

  // Initialize reference on first call
  if (isnan(sp_ref_for_timer))
  {
    sp_ref_for_timer = Setpoint;
  }

  // Reset timer if setpoint changed significantly (> 1.0°C)
  if (fabs(Setpoint - sp_ref_for_timer) > 1.0f)
  {
    timer_active = false;
    chronostart = 0;
    h = m = s = 0;
    lcd.setCursor(0, 3);
    lcd.print(F("00:00:00"));
    wasStable = false;
    stabilityStartTime = millis();
    sp_ref_for_timer = Setpoint; // update reference only when we reset
    return;
  }

  // Vérifie si l'écart entre température et consigne est trop grand
  if (abs(temperature - Setpoint) > 2.0)
  {
    timer_active = false;
    chronostart = 0;
    h = m = s = 0;
    lcd.setCursor(0, 3);
    lcd.print(F("00:00:00"));
    wasStable = false;
    stabilityStartTime = millis();
    return;
  }

  // Vérifie la stabilité de la température
  if (abs(delta) < NEAR_LIMIT)
  {
    if (!wasStable)
    {
      stabilityStartTime = millis();
      wasStable = true;
    }

    // Démarre le timer après 20s de stabilité
    if (!timer_active && wasStable && (millis() - stabilityStartTime >= 20000))
    {
      timer_active = true;
      chronostart = millis();
      buzz.pattern(3, false);
    }
  }
  else
  {
    wasStable = false;
  }
}

void ssr_mgmt()
{
  if (error)
  {
    pwm = 0;
    digitalWrite(SSR, LOW);
    black();
  }
  // Clamp PWM for safety before using it to compute timing
  if (pwm < 0)
    pwm = 0;
  if (pwm > 100)
    pwm = 100;
  // Mise à jour de l'affichage PWM uniquement si changement
  if (old_pwm != pwm)
  {
    lcd.setCursor(5, 1);
    print_space(6);
  lcd.setCursor(5, 1);
  lcd.print(pwm, 1);
  lcd.print(F("%"));
    old_pwm = pwm;
    onPWM_PERIOD = (pwm * PWM_PERIOD) / 100; // Calcul direct du temps ON
  }

  // Gestion du cycle PWM
  uint32_t currentMillis = millis();

  // Début d'un nouveau cycle
  if (currentMillis - startime >= PWM_PERIOD)
  {
    startime = currentMillis;
    if (pwm > 0)
    {
      digitalWrite(SSR, HIGH);
      red();

      pwmstate = true;
    }
  }

  // Fin de la période ON
  if (pwmstate && (currentMillis - startime >= onPWM_PERIOD))
  {
    digitalWrite(SSR, LOW);
    black();

    pwmstate = false;
  }
}

float pwm_cal()
{
  float ramp_temp = (delta - NEAR_LIMIT) * ((PWM_FAR - calculated_power) / (FAR_LIMIT - NEAR_LIMIT));
  float ramp;
  ramp = roundf(ramp_temp * 10) / 10.0;

  float return_value;

  return_value = (delta <= OVERSHOOT_X)  ? 0
                 : (delta <= NEAR_LIMIT) ? PWM_N
                 : (delta < FAR_LIMIT)   ? PWM_N + ramp
                                         : PWM_NEAR_OFFSET + PWM_FAR;

  return return_value;
}

// ==================== SETPOINT MANAGEMENT ====================
void setpoint_mgmt()
{
  if (encPos != 0)
  {
    Setpoint = constrain(Setpoint + (0.1 * encPos), MIN_SETPOINT, MAX_SETPOINT);
    encPos = 0;

    lcd.setCursor(5, 0);
    lcd.print(Setpoint, 1);
    print_deg();

    // Update feedforward power based on Setpoint (not temperature)
    calculated_power = calc_maintain_pow(Setpoint);
  }

  delta = Setpoint - temperature;
  if (old_delta != delta)
  {
    // Apply adaptive correction per temperature band
    uint8_t bi = band_index_for_setpoint(Setpoint);
    PWM_N = PWM_NEAR_OFFSET + calculated_power + pwm_corr[bi];
  }
  old_delta = delta;

  pwm = pwm_cal();
}

void restore_disp_man()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("PWM: "));
  lcd.print(pwm, 1);
  lcd.print(F("%"));
  lcd.setCursor(0, 3);
  lcd.print(F("00:00:00"));
  lcd.setCursor(0, 0);
  lcd.print(F("Set: "));
  lcd.print(Setpoint, 1);
  print_deg();
}

void dis_mode()
{
  if (mash_mode == 1)
  {
    restore_disp_man();
    axcel = 1;
    lcd.setCursor(13, 1);
    lcd.print(F("MASH  "));
    lcd.setCursor(0, 2);
    lcd.clearEOL();
  }
  else if (mash_mode == 2)
  {
    axcel = 1;
    lcd.setCursor(0, 0);
    print_space(11);
    lcd.setCursor(13, 1);
    lcd.print(F("BOIL  "));
    lcd.setCursor(0, 2);
    lcd.clearEOL();
  }
  else if (mash_mode == 3)
  {
    axcel = 0;
    restore_disp_man();
    lcd.setCursor(0, 0);
    lcd.print(F("Standby... "));
    lcd.setCursor(13, 1);
    lcd.print(F("IDLE   "));
  }
}

void dis_time()
{
  old_s = s;
  old_m = m;
  old_h = h;

  timelapse = (millis() - chronostart) / 1000;
  h = timelapse / 3600;
  m = (timelapse - (h * 3600)) / 60;
  s = timelapse - h * 3600 - m * 60;
  if (old_h != h)
  {
    lcd.setCursor(0, 3);
    if (h < 10)
      lcd.print('0');
    lcd.print(h);
    lcd.print(F(":"));
  }
  if (old_m != m)
  {
    lcd.setCursor(3, 3);
    if (m < 10)
      lcd.print('0');
    lcd.print(m);
    lcd.print(F(":"));
  }

  if (old_s != s)
  {
    lcd.setCursor(6, 3);
    if (s < 10)
      lcd.print('0');
    lcd.print(s);
  }
}

void boil_mgmt()
{

  if (encPos != 0 && mash_mode != 4)
  {

    pwm = pwm + 0.1 * encPos;
    if (pwm > 100.0)
    {
      pwm = 100.0;
    }
    if (pwm < 0)
    {
      pwm = 0;
    }
    encPos = 0;
  }
}

void fine()
{

  if (encPos != 0)
  {
    PWM_NEAR_OFFSET = constrain(PWM_NEAR_OFFSET + (0.1 * encPos), -30, 30);
    PWM_N = PWM_NEAR_OFFSET + calculated_power;
    pwm = pwm_cal();

    if (pwm < 0)
    {
      pwm = 0;
    }
    if (pwm > 100)
    {
      pwm = 100;
    }
    ssr_mgmt();
    encPos = 0;
    lcd.setCursor(0, 2);
    lcd.clearEOL();
    lcd.setCursor(0, 2);
    lcd.print(F("Offset: "));
    lcd.print(PWM_NEAR_OFFSET, 1);
  }
}

void read_temp()
{
  // Track last shown adaptive bias (rounded) to avoid unnecessary LCD updates
  static float prev_bias_display = NAN; // stores the last SHOWN value (after clamp & rounding)
  if (millis() - lastTempRequest >= delayInMillis)
  {
    if (!querry_temp)
    {
      sensors.requestTemperatures();
      lastTempRequest = millis();
      querry_temp = true;
    }
    else
    {
      float rawTemp = offset_temp + sensors.getTempCByIndex(0);

      // Gestion erreur sonde
      if (rawTemp == -127.00 || rawTemp == DEVICE_DISCONNECTED_C)
      {
        if (!error)
        { // Seulement à la première détection d'erreur
          error = true;
          pwm = 0;
          static char err[] = "Err"; // Tableau de caractères modifiable
          tm.displayPChar(err);
        }
      }
      else
      { // Température valide
        if (error)
        { // Si on sort d'une erreur
          error = false;
          resetSensor(); // Réinitialise une fois que le capteur est reconnu
          tm.displayFloat(temperature, 1);
        }

        if (emafilter)
        {
          // Compute measured dt (seconds) between filter updates
          static unsigned long lastFilterTime = 0;
          unsigned long nowt = millis();
          double dt_meas = (lastFilterTime == 0) ? dt : (double)(nowt - lastFilterTime) / 1000.0;
          lastFilterTime = nowt;
          filt.process(rawTemp, Setpoint, dt_meas, T_filtre, dT_dt);
          temperature = T_filtre;
          // Bottom-right: show learned adaptive bias for current band instead of dT/dt
          uint8_t bi_disp = band_index_for_setpoint(Setpoint);
          float bias_disp = pwm_corr[bi_disp];
          // Clamp tiny values to 0.0 for display
          float display_val = (fabs(bias_disp) < 0.05f) ? 0.0f : bias_disp;
          // Round to 1 decimal for comparison (as we print with 1 decimal)
          float shown = roundf(display_val * 10.0f) / 10.0f;
          if (isnan(prev_bias_display) || fabs(shown - prev_bias_display) >= 0.05f)
          {
            lcd.setCursor(13, 3);
            // Print explicit '+' for positives
            if (shown > 0.0f)
              lcd.print('+');
            lcd.print(shown, 1);
            lcd.clearEOL();
            prev_bias_display = shown;
          }
          dT_dt_prev = dT_dt;
          // temperature = tempFilter.update(rawTemp) ;
        }
        else
          temperature = rawTemp;

        if (test_mode)
          temperature = 50;

        if (temperature != prev_temp)
        {
          tm.displayFloat(temperature, 1);
          prev_temp = temperature;
          delta = Setpoint - temperature;
          // Feedforward power is now computed on Setpoint (see setpoint_mgmt)
        }
      }
      querry_temp = false;
    }
  }
}

void rot_man()
{
  buttonstate();

  if (encbutton_state == 1 && fineajust)
  {
    fineajust = 0;
    encbutton_state = 0;
    lcd.setCursor(0, 2);
    lcd.clearEOL();
  }

  if (encbutton_state == 1 && !fineajust)
  {
    // single click
    mash_mode++;
    if (mash_mode > 3)
    {
      mash_mode = 1;
    }

    dis_mode();
    encbutton_state = 0;
  }

  if (encbutton_state == 4)
  {
    // long press
    digitalWrite(SSR, LOW);
    black();

    pwmstate = false;
    menu_select = 0;
    encbutton_state = 0;
    fineajust = 0;
    mash_mode = 3;
  }
  if (encbutton_state == 2 && mash_mode == 1)
  {
    if (!fineajust)
    {
      lcd.setCursor(0, 2);
      lcd.print(F("Offset: "));
      lcd.print(PWM_NEAR_OFFSET, 1);
      pwm = pwm_cal();
      lcd.setCursor(5, 1);
      lcd.print(pwm, 1);
    }
    // keep pressed
    fineajust = 1;
    encbutton_state = 0;
  }
}

byte menu_mode_flex(const char *menu[], const char *title)
{
  lcd.clear();
  lcd.print(title);

  byte menu_index = 0;
  int num_choices = 0;
  int scroll_offset = 0;

  // Calculer le nombre de choix
  while (menu[num_choices] != NULL)
  {
    num_choices++;
  }

  // Afficher les choix visibles
  for (int i = 0; i < LCD_ROWS - 1; i++)
  {
    int menu_item_index = i + scroll_offset;
    if (menu_item_index < num_choices)
    {
      lcd.setCursor(1, i + 1);
      lcd.print(menu[menu_item_index]);
    }
    else
    {
      lcd.setCursor(1, i + 1);
      print_space(1);
    }
  }

  // Afficher le curseur
  lcd.setCursor(0, menu_index + 1 - scroll_offset);
  lcd.write(ARROW_RIGHT);

  while (1)
  {
    buttonstate();
    read_temp();

    if (encbutton_state == 1)
    {
      // Retourner l'index sélectionné
      encbutton_state = 0;
      return menu_index;
    }

    if (encPos < 0)
    {
      encPos = 0;
      if (menu_index < num_choices - 1)
      {
        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        print_space(1);
        menu_index++;

        if (menu_index >= scroll_offset + LCD_ROWS - 1)
        {
          scroll_offset++;
          // Effacer les lignes avant de les réécrire
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            lcd.setCursor(0, i + 1);
            lcd.clearEOL();
            // print_space(19);
          }
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            int menu_item_index = i + scroll_offset;
            if (menu_item_index < num_choices)
            {
              lcd.setCursor(1, i + 1);
              lcd.print(menu[menu_item_index]);
            }
            else
            {
              lcd.setCursor(1, i + 1);
              print_space(1);
            }
          }
        }

        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        lcd.write(ARROW_RIGHT);
      }
    }

    if (encPos > 0)
    {
      encPos = 0;

      if (menu_index > 0)
      {
        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        print_space(1);
        menu_index--;

        if (menu_index < scroll_offset)
        {
          scroll_offset--;
          // Effacer les lignes avant de les réécrire
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            lcd.setCursor(0, i + 1);
            lcd.clearEOL();

            // print_space(19);
          }
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            int menu_item_index = i + scroll_offset;
            if (menu_item_index < num_choices)
            {
              lcd.setCursor(1, i + 1);
              lcd.print(menu[menu_item_index]);
            }
            else
            {
              lcd.setCursor(1, i + 1);
              print_space(1);
            }
          }
        }

        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        lcd.write(ARROW_RIGHT);
      }
    }

    // Afficher les flèches de défilement
    if (scroll_offset > 0)
    {
      lcd.setCursor(LCD_COLS - 1, 1); // Haut à droite
      lcd.write(byte(2));             // Flèche vers le haut
    }
    else
    {
      lcd.setCursor(LCD_COLS - 1, 1);
      print_space(1);
    }

    if (scroll_offset + LCD_ROWS - 1 < num_choices)
    {
      lcd.setCursor(LCD_COLS - 1, LCD_ROWS - 1); // Bas à droite
      lcd.write(byte(3));                        // Flèche vers le bas
    }
    else
    {
      lcd.setCursor(LCD_COLS - 1, LCD_ROWS - 1);
      print_space(1);
    }
  }
  encbutton_state = 0;
  return 0;
}

void setup(void)
{
  Serial.begin(SERIAL_BAUDRATE);
  tm.begin(CLK, DIO, 4);
  tm.setBrightness(3);
  tm.displayClear();
  lcd.config(39, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin, BACKLIGHT_PIN, POSITIVE);
  Wire.begin();
  // Wire.setClock(100000);
  lcd.begin(20, 4);
  lcd.createChar(1, delta_char);
  lcd.createChar(2, arrowUp);
  lcd.createChar(3, arrowDown);

  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(false);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(SSR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(pinA, INPUT_PULLUP);      // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP);      // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  strip.begin();
  strip.show();
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  black();
  readEEPROM();
  // Initialize feedforward based on current Setpoint
  calculated_power = calc_maintain_pow(Setpoint);
  // Initialize autosave tracking snapshot for adaptive corrections
  for (int i = 0; i < 8; i++)
    pwm_corr_saved[i] = pwm_corr[i];
  last_adapt_save = millis();
}

void manual_mode()
{
  rot_man();
  check_counter();
  if (timer_active)
    dis_time();

  read_temp();
  if (mash_mode == 1)
  {
    if (fineajust)
    {
      fine();
    }
    else
    {
      setpoint_mgmt();
    }

    // ==================== Adaptive learning (BREW only) ====================
    if (adaptive_enable && !error && !test_mode)
    {
      static bool learnStable = false;
      static unsigned long learnStart = 0;

      float absDelta = fabs(Setpoint - temperature);
      float absDer = fabs(dT_dt);
      bool inNear = absDelta < NEAR_LIMIT;
      bool slowSlope = absDer < 0.01; // ~0.6 °C/min
      bool pwmInformative = (pwm > 5.0f && pwm < 95.0f);

      if (inNear && slowSlope && pwmInformative)
      {
        if (!learnStable)
        {
          learnStable = true;
          learnStart = millis();
        }
        else
        {
          if (millis() - learnStart > 60000UL)
          {
            uint8_t bi = band_index_for_setpoint(Setpoint);
            float expected = PWM_NEAR_OFFSET + calculated_power + pwm_corr[bi];
            float err = pwm - expected;
            const float beta = 0.02f; // small learning rate
            pwm_corr[bi] += beta * err;
            if (pwm_corr[bi] > 20.0f)
              pwm_corr[bi] = 20.0f;
            if (pwm_corr[bi] < -20.0f)
              pwm_corr[bi] = -20.0f;
            // Rebuild PWM_N after learning update
            PWM_N = PWM_NEAR_OFFSET + calculated_power + pwm_corr[bi];
            learnStart = millis(); // space updates
          }
        }
      }
      else
      {
        learnStable = false;
      }

      // ==================== Adaptive autosave policy ====================
      // Every 15 minutes, if any learned bias changed by >= 0.1 since last save, persist to EEPROM.
      unsigned long now_ms = millis();
      const unsigned long AUTOSAVE_PERIOD = 900000UL; // 15 minutes
      if (now_ms - last_adapt_save >= AUTOSAVE_PERIOD)
      {
        float max_delta = 0.0f;
        for (int i = 0; i < 8; i++)
        {
          float d = fabs(pwm_corr[i] - pwm_corr_saved[i]);
          if (d > max_delta)
            max_delta = d;
        }
        if (max_delta >= 0.1f)
        {
          writeEEPROM();
          for (int i = 0; i < 8; i++)
            pwm_corr_saved[i] = pwm_corr[i];
        }
        last_adapt_save = now_ms;
      }
    }
  }
  else if (mash_mode == 2)
  {
    boil_mgmt();
  }
  else if (mash_mode == 3)
  {
    pwm = 0;
    boil_mgmt();
  }

  ssr_mgmt();
}

float modify(const String title, float variable, float min, float max, float inc, const char *unit, const byte decimals)
{
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print(variable, decimals);
  lcd.print(unit);

  buttonstate();
  while (encbutton_state == 0)
  {
    buttonstate();
    if (encPos > 0 && variable + inc <= max)
      variable += inc;
    if (encPos < 0 && variable - inc >= min)
      variable -= inc;

    if (encPos != 0)
    {
      lcd.setCursor(0, 1);
      print_space(8);
      lcd.setCursor(0, 1);
      lcd.print(variable, decimals);
      lcd.print(unit);
      encPos = 0;
    }
  }
  lcd.clear();
  return variable;
}

float selector(float variable, float min, float max, float inc, const byte decimals, const byte x, const byte y)
{
  buttonstate();
  lcd.setCursor(x, y);
  lcd.print(variable, decimals);
  while (encbutton_state == 0)
  {
    buttonstate();
    if (encPos > 0 && variable + inc <= max)
      variable += inc;
    if (encPos < 0 && variable - inc >= min)
      variable -= inc;

    if (encPos != 0)
    {
      lcd.setCursor(x, y);
      print_space(5);
      lcd.setCursor(x, y);
      lcd.print(variable, decimals);
      encPos = 0;
    }
  }
  return variable;
}

// ==================== SETTINGS MENU ====================
void set_m()
{
  lcd.blink();
  lcd.clear();
  offset_temp = modify("Temperature offset", offset_temp, -3, 3, 0.1, "\xDF"
                                                                      "C",
                       1);
  lcd.clear();
  PWM_PERIOD = modify("PWM period", PWM_PERIOD, 1000, 8000, 100, " ms", 0);
  lcd.clear();
  emafilter = modify("Pipeline Filter", emafilter, 0, 1, 1, " (no:0 - yes:1)", 0);
  lcd.clear();
  adaptive_enable = modify("Adaptive learn", adaptive_enable, 0, 1, 1, " (no:0 - yes:1)", 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 1:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Overshoot limit"));
  lcd.setCursor(0, 2);
  lcd.print(F("if \x01<"));
  lcd.setCursor(0, 3);
  lcd.print(F("PWM=0"));
  lcd.setCursor(0, 2);
  OVERSHOOT_X = selector(OVERSHOOT_X, -1, 3, 0.1, 1, 5, 2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 2:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Setpoint zone"));
  lcd.setCursor(0, 2);
  lcd.print(F("if "));
  lcd.print(OVERSHOOT_X);
  lcd.print(F("<\x01<"));
  lcd.print(NEAR_LIMIT, 1);
  lcd.setCursor(0, 3);
  lcd.print(F("PWM offset="));
  lcd.print(PWM_NEAR_OFFSET, 1);
  NEAR_LIMIT = selector(NEAR_LIMIT, OVERSHOOT_X, 6, 0.1, 1, 11, 2);
  delay(300);
  PWM_NEAR_OFFSET = selector(PWM_NEAR_OFFSET, -30, 30, 0.1, 1, 11, 3);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 3:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Ramp zone"));
  lcd.setCursor(0, 2);
  lcd.print(F("if "));
  lcd.print(NEAR_LIMIT);
  lcd.print(F("<\x01<"));
  lcd.print(FAR_LIMIT, 0);
  lcd.setCursor(0, 3);
  lcd.print(F("Ramp PWM"));
  FAR_LIMIT = selector(FAR_LIMIT, NEAR_LIMIT, 50, 0.1, 1, 10, 2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 4:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Far zone"));
  lcd.setCursor(0, 2);
  lcd.print(F("if "));
  lcd.print("\x01>");
  lcd.print(FAR_LIMIT, 1);
  lcd.setCursor(0, 3);
  lcd.print(F("PMW="));
  PWM_FAR = selector(PWM_FAR, 0, 100, 1, 0, 4, 3);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Power Curve Setup"));

  // Coefficient A avec plus de précision et validation
  float newA = modify("Coeff. A (x0.0001)", MAINT_A * 10000, 1, 10000, 1, "", 0);
  MAINT_A = newA / 10000.0;

  // Coefficient B avec plus de précision et validation
  float newB = modify("Coeff. B (x0.0001)", MAINT_B * 10000, 1, 10000, 1, "", 0);
  MAINT_B = newB / 10000.0;

  // Coefficient C avec validation
  MAINT_C = modify("Offset C", MAINT_C, 0.1, 20.0, 0.1, "", 1);

  // Test de validité et correction si nécessaire
  if (MAINT_A <= 0 || isnan(MAINT_A))
    MAINT_A = 0.01455;
  if (MAINT_B <= 0 || isnan(MAINT_B))
    MAINT_B = 0.08593;
  if (MAINT_C <= 0 || isnan(MAINT_C))
    MAINT_C = 6.8386;

  // Affichage des valeurs finales pour confirmation
  lcd.clear();
  lcd.print(F("Values set to:"));
  lcd.setCursor(0, 1);
  lcd.print(F("A:"));
  lcd.print(MAINT_A, 5);
  lcd.setCursor(0, 2);
  lcd.print(F("B:"));
  lcd.print(MAINT_B, 5);
  lcd.setCursor(0, 3);
  lcd.print(F("C:"));
  lcd.print(MAINT_C, 1);
  delay(2000);

  lcd.noBlink();
  menu_select = 0;
}

void memory_menu()
{
  byte selected_index = menu_mode_flex(menu2, "---|MEMORY  MENU|---");

  switch (selected_index)
  {
  case 0:
    lcd.clear();
    lcd.print(F("Loading"));
    readEEPROM();
    delay(1000);
    menu_select = 0;
    break;
  case 1:
    lcd.clear();
    lcd.print(F("Saving"));
    writeEEPROM();
    delay(1000);
    menu_select = 0;

    break;
  case 2:
    factory_rst();
    menu_select = 0;
    break;
  case 3:
  {
    byte conf = menu_mode_flex(confirm_menu, "-- Reset adapt? --");
    if (conf == 0)
    {
      lcd.clear();
      lcd.print(F("Reset adapt corr"));
      for (int i = 0; i < 8; i++)
        pwm_corr[i] = 0.0f;
      writeEEPROM();
      // Update autosave snapshot to match cleared values
      for (int i = 0; i < 8; i++)
        pwm_corr_saved[i] = pwm_corr[i];
      delay(800);
    }
    else
    {
      lcd.clear();
      lcd.print(F("Canceled"));
      delay(600);
    }
    menu_select = 0;
  }
    break;
  case 4:
    menu_select = 0;
    break;
  default:
    menu_select = 0;
    break;
  }
}

void main_menu()
{
  byte selected_index = menu_mode_flex(menu1, "---| FUZZY BREW |---");

  switch (selected_index)
  {
  case 0:
    menu_select = 1;
    restore_disp_man();
    dis_mode();
    break;
  case 1:
    menu_select = 2;
    break;
  case 2:
    menu_select = 3;
    memory_menu();
    break;
  default:
    break;
  }
}

void loop()
{

  switch (menu_select)
  {
  case 0:
    main_menu();
    break;
  case 1:
    manual_mode();
    break;
  case 2:
    set_m();
    break;
  default:
    break;
  }
}

void buttonstate()
{
  bool state = digitalRead(PIN_BTN);
  encbutton_state = 0;
  float elapsed = millis() - buttontick;
  buzz.update();
  if (click_prev && !state)
  {
    // detect fall
    buttontick = millis();
  }

  if (!click_prev && !state && elapsed > 2500)
  {
    // when continuous push
    encbutton_state = 4;
  }

  if (!click_prev && state)
  {

    if (elapsed > 400 && elapsed < 2500)
    {
      // long push
      encbutton_state = 2;
    }

    if (elapsed < 400)
    {
      // single click
      encbutton_state = 1;
    }
  }

  click_prev = state;
}
void red()
{
  strip.setPixelColor(0, strip.Color(25, 0, 0)); // Rouge
  strip.show();
}
void black()
{
  strip.setPixelColor(0, strip.Color(0, 0, 0)); // noir
  strip.show();
}
