// =================================
// MODE TEST CONFIGURATION
// =================================
// Mettre à true pour tester sans SI4735 (écran, encodeur, interface)
#define TEST_MODE false

// =================================
// INCLUDE FILES
// =================================

#include "Common.h"
#include <Wire.h>
#include "Rotary.h"
#include "Button.h"
#include "Menu.h"
#include "Draw.h"
#include "Storage.h"
#include "Themes.h"
#include "Utils.h"
#include "EIBI.h"

// SI473/5 and UI
#define MIN_ELAPSED_TIME         5
#define MIN_ELAPSED_RSSI_TIME  200
#define ELAPSED_COMMAND      10000
#define DEFAULT_VOLUME          35
#define DEFAULT_SLEEP            0
#define STRENGTH_CHECK_TIME   1500
#define RDS_CHECK_TIME         250
#define SEEK_TIMEOUT        600000
#define NTP_CHECK_TIME       60000
#define SCHEDULE_CHECK_TIME   2000
#define BACKGROUND_REFRESH_TIME 5000

// =================================
// CONSTANTS AND VARIABLES
// =================================

int8_t agcIdx = 0;
uint8_t disableAgc = 0;
int8_t agcNdx = 0;
int8_t softMuteMaxAttIdx = 4;

bool seekStop = false;
bool pushAndRotate = false;

long elapsedRSSI = millis();
long elapsedButton = millis();
long lastStrengthCheck = millis();
long lastRDSCheck = millis();
long lastNTPCheck = millis();
long lastScheduleCheck = millis();
long elapsedCommand = millis();

volatile int16_t encoderCount = 0;
volatile int16_t encoderCountAccel = 0;
uint16_t currentFrequency;

// AGC/ATTN index per mode (FM/AM/SSB)
int8_t FmAgcIdx = 0;
int8_t AmAgcIdx = 0;
int8_t SsbAgcIdx = 0;

// AVC index per mode (AM/SSB)
int8_t AmAvcIdx = 48;
int8_t SsbAvcIdx = 48;

// SoftMute index per mode (AM/SSB)
int8_t AmSoftMuteIdx = 4;
int8_t SsbSoftMuteIdx = 4;

// Menu options
uint8_t volume = DEFAULT_VOLUME;
uint8_t currentSquelch = 0;
uint8_t FmRegionIdx = 0;

uint16_t currentBrt = 130;
uint16_t currentSleep = DEFAULT_SLEEP;
long elapsedSleep = millis();
bool zoomMenu = false;
int8_t scrollDirection = 1;

// Background screen refresh
uint32_t background_timer = millis();

// Current parameters
uint16_t currentCmd  = CMD_NONE;
uint8_t  currentMode = FM;
int16_t  currentBFO  = 0;

uint8_t  rssi = 0;
uint8_t  snr  = 0;

// Devices
Rotary encoder  = Rotary(ENCODER_PIN_B, ENCODER_PIN_A);
ButtonTracker pb1 = ButtonTracker();
TFT_eSPI tft    = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
SI4735_fixed rx;

//
// Hardware initialization and setup
//
void setup()
{
  // Enable serial port
  #if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
  #endif
  
  Serial.begin(115200);
  delay(2000);
  
  // Attendre que le Serial soit prêt (max 3 secondes)
  unsigned long startSerial = millis();
  while (!Serial && (millis() - startSerial) < 3000) {
    delay(100);
  }
  
  Serial.println("\n\n=================================");
  Serial.println("    ATS-MINI STARTUP");
  Serial.println("=================================");
  
  if (TEST_MODE) {
    Serial.println("*** MODE TEST ACTIF ***");
    Serial.println("SI4735 et audio bypassés");
    Serial.println("=================================\n");
  }

  // Encoder pins. Enable internal pull-ups
  pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  Serial.println("Encodeur configuré");

  // Enable audio amplifier
  pinMode(PIN_AMP_EN, OUTPUT);
  digitalWrite(PIN_AMP_EN, LOW);
  Serial.println("Ampli audio désactivé (init)");

  // Enable SI4732 VDD
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  delay(100);
  Serial.println("Alimentation SI4735 activée");

  // I2C setup
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);
  Serial.println("I2C initialisé");

  // TFT display brightness control (PWM)
  ledcAttach(PIN_LCD_BL, 16000, 8);
  ledcWrite(PIN_LCD_BL, 0);
  Serial.println("Backlight PWM configuré");

  // TFT display setup
  Serial.println("Initialisation TFT...");
  tft.init();
  tft.setRotation(3);

  // Detect and fix the mirrored & inverted display
  uint8_t did3 = tft.readcommand8(ST7789_RDDID, 3);
  Serial.print("Display ID: 0x");
  Serial.println(did3, HEX);
  
  if(did3 == 0x93)
  {
    tft.invertDisplay(0);
    tft.writecommand(TFT_MADCTL);
    tft.writedata(TFT_MAD_MV | TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_BGR);
    Serial.println("Display: Mode mirrored/inverted corrigé");
  }
  else if(did3 == 0x85)
  {
    tft.writecommand(0x26);
    tft.writedata(8);
    tft.writecommand(0x55);
    tft.writedata(0xB1);
    Serial.println("Display: High gamma mode activé");
  }

  tft.fillScreen(TH.bg);
  spr.createSprite(320, 170);
  spr.setTextDatum(MC_DATUM);
  spr.setSwapBytes(true);
  spr.setFreeFont(&Orbitron_Light_24);
  spr.setTextColor(TH.text, TH.bg);
  Serial.println("TFT et sprite initialisés");

  // Press and hold Encoder button to force preferences reset
  if(digitalRead(ENCODER_PUSH_BUTTON)==LOW)
  {
    Serial.println("RESET DES PREFERENCES demandé");
    nvsErase();
    diskInit(true);

    ledcWrite(PIN_LCD_BL, 255);
    tft.setTextSize(2);
    tft.setTextColor(TH.text, TH.bg);
    tft.println(getVersion(true));
    tft.println();
    tft.setTextColor(TH.text_warn, TH.bg);
    tft.print("Resetting Preferences");
    while(digitalRead(ENCODER_PUSH_BUTTON) == LOW) delay(100);
    Serial.println("Préférences réinitialisées");
  }

  // Initialize flash file system
  Serial.println("Initialisation système de fichiers...");
  diskInit();

  // SI4732 SETUP (BYPASS EN MODE TEST)
  if (TEST_MODE) {
    Serial.println("\n*** BYPASS SI4735 (MODE TEST) ***");
    Serial.println("Simulation de la radio active\n");
    
    // Afficher un message à l'écran
    ledcWrite(PIN_LCD_BL, 255);
    tft.fillScreen(TH.bg);
    tft.setTextSize(2);
    tft.setTextColor(TH.text_warn, TH.bg);
    tft.setCursor(10, 60);
    tft.println("MODE TEST");
    tft.setCursor(10, 90);
    tft.setTextColor(TH.text, TH.bg);
    tft.println("SI4735 bypasse");
    tft.setCursor(10, 120);
    tft.println("Interface active");
    delay(2000);
    
  } else {
    // MODE NORMAL: Initialisation SI4735
    Serial.println("Recherche du SI4735...");
    rx.setI2CFastModeCustom(800000UL);

    int16_t si4735Addr = rx.getDeviceI2CAddress(RESET_PIN);
    if(!si4735Addr)
    {
      Serial.println("ERREUR: SI4735 non détecté!");
      ledcWrite(PIN_LCD_BL, 255);
      tft.setTextSize(2);
      tft.setTextColor(TH.text_warn, TH.bg);
      tft.println("Si4732 not detected");
      while(1) delay(1000);
    }
    Serial.println("SI4735 trouvé!");

    rx.setup(RESET_PIN, MW_BAND_TYPE);
    rx.setAudioMuteMcuPin(AUDIO_MUTE);
    Serial.println("SI4735 configuré");
  }

  // Load preferences
  Serial.println("Chargement des préférences...");
  if(!prefsLoad(SAVE_SETTINGS|SAVE_VERIFY))
  {
    prefsSave(SAVE_SETTINGS);
    spr.fillSprite(TH.bg);
    ledcWrite(PIN_LCD_BL, currentBrt);
    drawAboutHelp(0);
    while(digitalRead(ENCODER_PUSH_BUTTON)!=LOW) delay(100);
    while(digitalRead(ENCODER_PUSH_BUTTON)==LOW) delay(100);
  }

  if(!prefsLoad(SAVE_MEMORIES|SAVE_VERIFY)) prefsSave(SAVE_MEMORIES);
  if(!prefsLoad(SAVE_BANDS|SAVE_VERIFY)) prefsSave(SAVE_BANDS);

  // Enable audio amplifier (sauf en mode test)
  if (!TEST_MODE) {
    digitalWrite(PIN_AMP_EN, HIGH);
    Serial.println("Ampli audio activé");
  } else {
    Serial.println("Ampli audio désactivé (MODE TEST)");
  }

  // SI4732 STARTUP (sauf en mode test)
  if (!TEST_MODE) {
    selectBand(bandIdx, false);
    delay(50);
    rx.setVolume(volume);
    rx.setMaxSeekTime(SEEK_TIMEOUT);
  }

  // Draw display for the first time
  Serial.println("Affichage initial...");
  drawScreen();
  ledcWrite(PIN_LCD_BL, currentBrt);

  // Interrupt actions for Rotary encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);
  Serial.println("Interruptions encodeur configurées");

  // Connect WiFi, if necessary
  netInit(wifiModeIdx);

  // Start Bluetooth LE, if necessary
  bleInit(bleModeIdx);

  Serial.println("\n=================================");
  Serial.println("   INITIALISATION TERMINEE");
  if (TEST_MODE) {
    Serial.println("   MODE TEST ACTIF");
    Serial.println("Tournez l'encodeur pour tester");
  }
  Serial.println("=================================\n");
}

int16_t accelerateEncoder(int8_t dir)
{
  const uint32_t speedThresholds[] = {350, 60, 45, 35, 25};
  const uint16_t accelFactors[] =      {1,  2,  4,  8, 16};
  static uint32_t lastEncoderTime = 0;
  static uint32_t lastSpeed = speedThresholds[0];
  static uint16_t lastAccelFactor = accelFactors[0];
  static int8_t lastEncoderDir = 0;

  uint32_t currentTime = millis();
  lastSpeed = ((currentTime - lastEncoderTime) * 7 + lastSpeed * 3) / 10;

  if (lastSpeed > speedThresholds[0] || lastEncoderDir != dir) {
    lastSpeed = speedThresholds[0];
    lastAccelFactor = accelFactors[0];
  } else {
    for (int8_t i = LAST_ITEM(speedThresholds); i >= 0; i--) {
      if (lastSpeed <= speedThresholds[i] && lastAccelFactor < accelFactors[i]) {
        lastAccelFactor = accelFactors[i];
        break;
      }
    }
  }
  lastEncoderTime = currentTime;
  lastEncoderDir = dir;

  return(dir * lastAccelFactor);
}

ICACHE_RAM_ATTR void rotaryEncoder()
{
  uint8_t encoderStatus = encoder.process();
  if(encoderStatus)
  {
    int8_t delta = encoderStatus==DIR_CW? 1 : -1;
    int16_t accelDelta = accelerateEncoder(delta);

    if(abs(encoderCount) < 5)
    {
      encoderCount += delta;
      encoderCountAccel += accelDelta;
    }

    seekStop = true;
    
    // Debug en mode test
    if (TEST_MODE) {
      Serial.print("Encodeur: ");
      Serial.println(delta > 0 ? "CW" : "CCW");
    }
  }
}

uint32_t consumeEncoderCounts()
{
  int16_t encCount, encCountAccel;
  noInterrupts();
  encCount = encoderCount;
  encCountAccel = encoderCountAccel;
  encoderCount = 0;
  encoderCountAccel = 0;
  interrupts();
  return ((uint32_t)encCountAccel << 16) | ((uint16_t)encCount & 0xFFFF);
}

void useBand(const Band *band)
{
  if (TEST_MODE) {
    Serial.print("useBand() MODE TEST: ");
    Serial.println(band->bandName);
    currentFrequency = band->currentFreq;
    currentMode = band->bandMode;
    currentBFO = 0;
    return;
  }
  
  // Code normal SI4735...
  currentFrequency = band->currentFreq;
  currentMode = band->bandMode;
  currentBFO = 0;

  if(band->bandMode==FM)
  {
    rx.setFM(band->minimumFreq, band->maximumFreq, band->currentFreq, getCurrentStep()->step);
    rx.setSeekFmLimits(band->minimumFreq, band->maximumFreq);
    rx.setSeekFmRssiThreshold(5);
    rx.setSeekFmSNRThreshold(2);
    rx.setFMDeEmphasis(fmRegions[FmRegionIdx].value);
    rx.RdsInit();
    rx.setRdsConfig(1, 2, 2, 2, 2);
    rx.setGpioCtl(1, 0, 0);
    rx.setGpio(0, 0, 0);
  }
  else
  {
    if(band->bandMode==AM)
    {
      rx.setAM(band->minimumFreq, band->maximumFreq, band->currentFreq, getCurrentStep()->step);
      rx.setSeekAmRssiThreshold(10);
      rx.setSeekAmSNRThreshold(3);
    }
    else
    {
      rx.setSSB(band->minimumFreq, band->maximumFreq, band->currentFreq, 0, currentMode);
      rx.setSSBAutomaticVolumeControl(1);
      if (currentMode == USB)
        rx.setSSBBfo(-(currentBFO + band->usbCal));
      else if (currentMode == LSB)
        rx.setSSBBfo(-(currentBFO + band->lsbCal));
      else
        rx.setSSBBfo(-currentBFO);
    }
    rx.setGpioCtl(1, 0, 0);
    rx.setGpio(1, 0, 0);
    rx.setSeekAmLimits(band->minimumFreq, band->maximumFreq);
  }

  doStep(0);
  doSoftMute(0);
  doAgc(0);
  doAvc(0);
  delay(100);
  rssi = 0;
  snr  = 0;
}

bool updateBFO(int newBFO, bool wrap)
{
  if (TEST_MODE) {
    Serial.print("updateBFO() MODE TEST: ");
    Serial.println(newBFO);
    currentBFO = newBFO;
    return true;
  }
  
  Band *band = getCurrentBand();
  int newFreq = currentFrequency;

  if(!isSSB()) newBFO = 0;

  if(newBFO > MAX_BFO || newBFO < -MAX_BFO)
  {
    int fCorrect = (newBFO / MAX_BFO) * MAX_BFO;
    newFreq += fCorrect / 1000;
    newBFO  -= fCorrect;
  }

  int f = newFreq * 1000 + newBFO;
  if(f < band->minimumFreq * 1000)
  {
    if(!wrap) return false;
    newFreq = band->maximumFreq;
    newBFO  = 0;
  }
  else if(f > band->maximumFreq * 1000)
  {
    if(!wrap) return false;
    newFreq = band->minimumFreq;
    newBFO  = 0;
  }

  if(newFreq != currentFrequency)
  {
    rx.setFrequency(newFreq);
    doAgc(0);
    currentFrequency = rx.getFrequency();
  }

  currentBFO = newBFO;

  if (currentMode == USB)
    rx.setSSBBfo(-(currentBFO + band->usbCal));
  else if (currentMode == LSB)
    rx.setSSBBfo(-(currentBFO + band->lsbCal));
  else
    rx.setSSBBfo(-currentBFO);

  band->currentFreq = currentFrequency + currentBFO / 1000;
  return true;
}

bool updateFrequency(int newFreq, bool wrap)
{
  if (TEST_MODE) {
    Serial.print("updateFrequency() MODE TEST: ");
    Serial.println(newFreq);
    currentFrequency = newFreq;
    return true;
  }
  
  Band *band = getCurrentBand();

  if(newFreq < band->minimumFreq)
  {
    if(!wrap) return false; else newFreq = band->maximumFreq;
  }
  else if(newFreq > band->maximumFreq)
  {
    if(!wrap) return false; else newFreq = band->minimumFreq;
  }

  rx.setFrequency(newFreq);

  if(currentBFO) updateBFO(0, true);

  currentFrequency = rx.getFrequency();
  band->currentFreq = currentFrequency + currentBFO / 1000;
  return true;
}

bool checkStopSeeking()
{
  if(seekStop) return true;

  if(pb1.update(digitalRead(ENCODER_PUSH_BUTTON) == LOW, 0).isPressed)
  {
    while(pb1.update(digitalRead(ENCODER_PUSH_BUTTON) == LOW).isPressed)
      delay(100);
    return true;
  }

  return false;
}

void showFrequencySeek(uint16_t freq)
{
  currentFrequency = freq;
  drawScreen();
}

bool doSeek(int16_t enc, int16_t enca)
{
  if (TEST_MODE) {
    Serial.println("doSeek() MODE TEST");
    return false;
  }
  
  muteOn(MUTE_TEMP, true);
  if(seekMode() == SEEK_DEFAULT)
  {
    if(isSSB())
    {
      updateBFO(currentBFO + enca * getCurrentStep()->step, true);
    }
    else
    {
      clearStationInfo();
      rssi = snr = 0;
      seekStop = false;
      rx.seekStationProgress(showFrequencySeek, checkStopSeeking, enc>0? 1 : 0);
      updateFrequency(rx.getFrequency(), true);
    }
  }
  else if(seekMode() == SEEK_SCHEDULE && enc)
  {
    uint8_t hour, minute;
    clockGetHM(&hour, &minute);

    size_t offset = -1;
    const StationSchedule *schedule = enc > 0 ?
      eibiNext(currentFrequency + currentBFO / 1000, hour, minute, &offset) :
      eibiPrev(currentFrequency + currentBFO / 1000, hour, minute, &offset);

    if(schedule) updateFrequency(schedule->freq, false);
  }

  clearStationInfo();
  identifyFrequency(currentFrequency + currentBFO / 1000);
  muteOn(MUTE_TEMP, false);
  return(true);
}

bool doTune(int16_t enc)
{
  if (TEST_MODE) {
    Serial.print("doTune() MODE TEST: ");
    Serial.println(enc > 0 ? "UP" : "DOWN");
  }
  
  if(isSSB())
  {
    uint32_t step = getCurrentStep()->step;
    uint32_t stepAdjust = (currentFrequency * 1000 + currentBFO) % step;
    step = !stepAdjust? step : enc>0? step - stepAdjust : stepAdjust;

    updateBFO(currentBFO + enc * step, true);
  }
  else
  {
    uint16_t step = getCurrentStep()->step;
    uint16_t stepAdjust = currentFrequency % step;
    stepAdjust = (currentMode==FM) && (step==20)? (stepAdjust+10) % step : stepAdjust;
    step = !stepAdjust? step : enc>0? step - stepAdjust : stepAdjust;

    updateFrequency(currentFrequency + step * enc, true);
  }

  clearStationInfo();
  identifyFrequency(currentFrequency + currentBFO / 1000);
  return(true);
}

bool doDigit(int16_t enc)
{
  bool updated = false;

  if(isSSB())
  {
    updated = updateBFO(currentBFO + enc * getFreqInputStep(), false);
  }
  else
  {
    updated = updateFrequency(currentFrequency + enc * getFreqInputStep(), false);
  }

  if (updated) {
    clearStationInfo();
    identifyFrequency(currentFrequency + currentBFO / 1000);
  }

  return(updated);
}

bool clickFreq(bool shortPress)
{
  if (shortPress) {
    bool updated = false;

     if(isSSB()) {
       updated = updateBFO(currentBFO - (currentFrequency * 1000 + currentBFO) % getFreqInputStep(), false);
     } else {
       updated = updateFrequency(currentFrequency - currentFrequency % getFreqInputStep(), false);
     }

     if (updated) {
       clearStationInfo();
       identifyFrequency(currentFrequency + currentBFO / 1000);
     }
     return true;
  }
  return false;
}

bool processRssiSnr()
{
  if (TEST_MODE) {
    // Simulation de RSSI/SNR aléatoire
    static uint32_t simCounter = 0;
    if (!(simCounter++ % 10)) {
      rssi = 20 + (millis() / 1000) % 40;
      snr = 10 + (millis() / 500) % 20;
      return true;
    }
    return false;
  }
  
  static uint32_t updateCounter = 0;
  bool needRedraw = false;

  rx.getCurrentReceivedSignalQuality();
  int newRSSI = rx.getCurrentRSSI();
  int newSNR = rx.getCurrentSNR();

  if(currentSquelch && currentSquelch <= 127)
  {
    if(newRSSI >= currentSquelch && muteOn(MUTE_SQUELCH))
    {
      muteOn(MUTE_SQUELCH, false);
    }
    else if(newRSSI < currentSquelch && !muteOn(MUTE_SQUELCH))
    {
      muteOn(MUTE_SQUELCH, true);
    }
  }
  else if(muteOn(MUTE_SQUELCH))
  {
    muteOn(MUTE_SQUELCH, false);
  }

  if(!(updateCounter++ & 7))
  {
    if(newRSSI != rssi)
    {
      rssi = newRSSI;
      needRedraw = true;
    }
    if(newSNR != snr)
    {
      snr = newSNR;
      needRedraw = true;
    }
  }
  return needRedraw;
}

//
// Main event loop
//
void loop()
{
  uint32_t currentTime = millis();
  bool needRedraw = false;

  uint32_t encCounts = consumeEncoderCounts();
  int16_t encCount = (int16_t)(encCounts & 0xFFFF);
  int16_t encCountAccel = (int16_t)(encCounts >> 16);

  ButtonTracker::State pb1st = pb1.update(digitalRead(ENCODER_PUSH_BUTTON) == LOW);

  // Debug en mode test
  if (TEST_MODE && pb1st.wasClicked) {
    Serial.println("Bouton: Click");
  }
  if (TEST_MODE && pb1st.isLongPressed) {
    Serial.println("Bouton: Long Press");
  }

  remoteTickTime();

  if(Serial.available()>0)
  {
    int revent = remoteDoCommand(Serial.read());
    needRedraw |= !!(revent & REMOTE_CHANGED);
    pb1st.wasClicked |= !!(revent & REMOTE_CLICK);
    int direction = revent >> REMOTE_DIRECTION;
    encCount = direction? direction : encCount;
    encCountAccel = direction? direction : encCountAccel;
    if(revent & REMOTE_PREFS) prefsRequestSave(SAVE_ALL);
  }

  int ble_event = bleDoCommand(bleModeIdx);

  if(encCount && sleepOn() && sleepModeIdx==SLEEP_LOCKED) encCount = encCountAccel = 0;

  if (encCount && pb1st.isPressed) pushAndRotate = true;

  if(pushAndRotate)
  {
    if(encCount)
    {
      switch(currentCmd)
      {
        case CMD_NONE:
          currentCmd = CMD_FREQ;
          needRedraw = true;
          break;
        case CMD_FREQ:
          doSelectDigit(encCount);
          needRedraw = true;
          break;
        case CMD_SEEK:
          needRedraw |= doTune(encCount);
          prefsRequestSave(SAVE_CUR_BAND);
          break;
      }
    }
    elapsedSleep = elapsedCommand = currentTime;
  }
  else
  {
    if(encCount)
    {
      switch(currentCmd)
      {
        case CMD_NONE:
        case CMD_SCAN:
          needRedraw |= doTune(encCountAccel);
          prefsRequestSave(SAVE_CUR_BAND);
          break;
        case CMD_FREQ:
          needRedraw |= doDigit(encCount);
          prefsRequestSave(SAVE_CUR_BAND);
          break;
        case CMD_SEEK:
          needRedraw |= doSeek(encCount, encCountAccel);
          currentTime = millis();
          prefsRequestSave(SAVE_CUR_BAND);
          break;
        default:
          needRedraw |= doSideBar(currentCmd, encCount, encCountAccel);
          prefsRequestSave(SAVE_ALL);
          break;
      }

      elapsedSleep = elapsedCommand = currentTime;
    }
    else if(pb1st.isLongPressed)
    {
      sleepOn(!sleepOn());
      elapsedSleep = elapsedCommand = currentTime = millis();
    }
    else if(pb1st.wasClicked || pb1st.wasShortPressed)
    {
      elapsedSleep = elapsedCommand = currentTime;

      if(sleepOn())
      {
        if(currentSleep)
        {
          sleepOn(false);
          needRedraw = true;
        }
        else if(sleepModeIdx == SLEEP_UNLOCKED)
        {
          if(pb1st.wasShortPressed && currentCmd==CMD_NONE)
            currentCmd = CMD_VOLUME;
          else if(currentCmd==CMD_VOLUME)
            clickHandler(currentCmd, pb1st.wasShortPressed);

          needRedraw = true;
        }
      }
      else if(clickHandler(currentCmd, pb1st.wasShortPressed))
      {
        needRedraw = true;
        elapsedSleep = elapsedCommand = currentTime = millis();
      }
      else if(currentCmd != CMD_NONE)
      {
        currentCmd = CMD_NONE;
        needRedraw = true;
      }
      else if(pb1st.wasShortPressed)
      {
        currentCmd = CMD_VOLUME;
        needRedraw = true;
      }
      else
      {
        currentCmd = CMD_MENU;
        needRedraw = true;
      }
    }
  }

  if(!pb1st.isPressed && pushAndRotate)
  {
    pushAndRotate = false;
    needRedraw = true;
  }

  if((currentTime - elapsedCommand) > ELAPSED_COMMAND)
  {
    if(currentCmd != CMD_NONE && currentCmd != CMD_SEEK && currentCmd != CMD_SCAN && currentCmd != CMD_MEMORY)
    {
      currentCmd = CMD_NONE;
      needRedraw = true;
    }

    elapsedCommand = currentTime;
  }

  if(currentSleep && !sleepOn() && ((currentTime - elapsedSleep) > currentSleep * 1000))
  {
    sleepOn(true);
    elapsedSleep = elapsedCommand = currentTime = millis();
  }

  if((currentTime - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME)
  {
    needRedraw |= processRssiSnr();
    elapsedRSSI = currentTime;
  }

  if((currentTime - lastRDSCheck) > RDS_CHECK_TIME)
  {
    if (!TEST_MODE) {
      needRedraw |= (currentMode == FM) && (snr >= 12) && checkRds();
    }
    lastRDSCheck = currentTime;
  }

  if((currentTime - lastScheduleCheck) > SCHEDULE_CHECK_TIME)
  {
    needRedraw |= identifyFrequency(currentFrequency + currentBFO / 1000, true);
    lastScheduleCheck = currentTime;
  }

  if((currentTime - lastNTPCheck) > NTP_CHECK_TIME)
  {
    needRedraw |= ntpSyncTime();
    lastNTPCheck = currentTime;
  }

  prefsTickTime();
  netTickTime();
  needRedraw |= clockTickTime();

  if(needRedraw) background_timer = currentTime;
  if((currentTime - background_timer) > BACKGROUND_REFRESH_TIME)
  {
    if(currentCmd == CMD_NONE) needRedraw = true;
    background_timer = currentTime;
  }

  if(needRedraw) drawScreen();

  delay(5);
}
