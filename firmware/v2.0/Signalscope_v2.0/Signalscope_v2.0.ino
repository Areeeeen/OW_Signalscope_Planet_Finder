// Signalscope_v2.0
// Made By - Areen Phaltankar

#include <Wire.h>
#include <ICM20948_WE.h>
#include <RTClib.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceBuffer.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <PNGdec.h>

#define ASTRONOMY_ENGINE_WHOLE_SECOND   // ESP32: use 1-second resolution
#include "astronomy.h"                  // keep astronomy.c beside the sketch, compiled as a separate file
#include "loading.h" // The loading screen image data in flash array

const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

double latitude = //YOUR_LONGITUDE;
double longitude = //YOUR_LONGITUDE;

astro_time_t astroTime;                 // current time for Astronomy Engine
astro_observer_t observer = { 0, 0, 0}; // lat, lon, elevation meters (set after GPS / fallback)

struct Planet {              // Define a struct to hold planet data
  const char* name;
  const char* music;
  float dec;
  float RA;
  float Az;                  // data will be added by the code for the Azimuth and Altitude
  float Alt;
  float offset;
  float distanceAU;
};

Planet planets[] = {                                     // Create an array of 7 planets with their initial data  // UPDATE VALUES EVERY 3 DAYS
  {"Mercury", "/starwars.wav", 25.24139, 97.75},
  {"Venus",   "/starwars.wav",   11.21000, 34.93},
  {"Mars",    "/starwars.wav",    13.60028, 149.96},
  {"Jupiter", "/starwars.wav", 23.27500, 90.65},         // values accurate as of june 12th 2025
  {"Saturn",  "/starwars.wav",  -1.5161,  1.92},
  {"Uranus",  "/starwars.wav",  19.675,   56.58},
  {"Neptune", "/starwars.wav", -0.38861, 2.39}
};

PNG png;
#define MAX_IMAGE_WIDTH 240
int16_t xpos = 0;
int16_t ypos = 0;

TaskHandle_t TaskCore0Handle;
TaskHandle_t AudioTaskHandle;

ICM20948_WE myIMU = ICM20948_WE(0x69);
const float magX_offset = -21.902;
const float magY_offset = -12.483;
const float magZ_offset = 27.059;
volatile float altitude_deg;
volatile float azimuth_deg;
float closestOffset;
String closestPlanet;
float closestDistance;
volatile bool imuDataReady = false;

RTC_DS1307 rtc;
#define IST_OFFSET_SECONDS 19800
#define NTP_SERVER "pool.ntp.org"

bool wifiConnected = false;

HardwareSerial gpsSerial(2); // Serial2 for GPS
TinyGPSPlus gps;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);  // Framebuffer sprite
uint8_t radius       = 60; // Arc outer radius
uint8_t thickness    = 4;
uint8_t inner_radius = radius - thickness;
uint16_t LEFTstart_angle = 0;
uint16_t LEFTend_angle   = 180;
uint16_t RIGHTstart_angle = 180;
uint16_t RIGHTend_angle   = 0;
bool arc_end = true;
int centerX = 120;
int centerY = 160;

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 5000;

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

#define TFT_MOSI 15
#define TFT_SCLK 14
#define TFT_CS   22
#define TFT_DC   21
#define TFT_RST  -1

SPIClass mySPI(VSPI);
#define SD_CS    5
#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23

#define I2S_BCLK 26
#define I2S_LRCK 25
#define I2S_DOUT 27

#define LED_PWR 4
#define LED_LOCKOn 13

AudioGeneratorWAV *wav = nullptr;
AudioFileSourceSD *file = nullptr;
AudioOutputI2S *out = nullptr;
AudioFileSourceBuffer *buff = nullptr;

#define AA_FONT_SMALL "NotoSansBold15"
#define AA_FONT_LARGE "NotoSansBold36"

void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

String currentPlaying = "";
const float offsetThreshold = 20.0;

void handlePlanetAudio() {
  // If offset is too large, stop audio if running
  if (closestOffset > offsetThreshold) {
    if (wav && wav->isRunning()) {
      wav->stop();
      currentPlaying = "";
    }
    return;
  }

  // Find the matching planet to get the file path
  const char* filepath = nullptr;
  for (int i = 0; i < 7; i++) {
    if (String(planets[i].name) == closestPlanet) {
      filepath = planets[i].music;
      break;
    }
  }

  if (filepath == nullptr) return; // No matching file

  // Avoid restarting same track
  if (currentPlaying == String(filepath) && wav && wav->isRunning()) {
    // Update volume based on offset
    float vol = 1.0 - (closestOffset / offsetThreshold);
    vol = constrain(vol, 0.0, 1.0);
    if (out) out->SetGain(vol);
    return;
  }

  // Stop any current audio
  if (wav && wav->isRunning()) wav->stop();
  if (file) delete file;
  if (buff) delete buff;
  if (wav) delete wav;
  if (out) delete out;

  // Prepare new file and playback
  file = new AudioFileSourceSD(filepath);
  buff = new AudioFileSourceBuffer(file, 8192);
  out = new AudioOutputI2S();
  out->SetPinout(26, 25, 27);  // Replace with your actual I2S pins

  // Set initial volume based on offset
  float volume = 1.0 - (closestOffset / offsetThreshold);
  volume = constrain(volume, 0.0, 1.0);
  out->SetGain(volume);

  wav = new AudioGeneratorWAV();
  if (!wav->begin(buff, out)) {
    Serial.println("Failed to play audio.");
    currentPlaying = "";
  } else {
    Serial.print("Now playing: ");
    Serial.println(filepath);
    currentPlaying = String(filepath);
  }
}

void updateUserLocation() {
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 6);
  latitude = gps.location.lat();

  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 6);
  longitude = gps.location.lng();
}

void TaskCore0(void *pvParameters) {
  while (true) {
    xyzFloat gValue;
    xyzFloat angle;
    xyzFloat magValue;

    myIMU.readSensor();
    myIMU.getGValues(&gValue);
    myIMU.getMagValues(&magValue);
    myIMU.getAngles(&angle);

    magValue.x -= magX_offset;
    magValue.y -= magY_offset;
    magValue.z -= magZ_offset;

    // Normalize accelerometer vector
    float normAcc = sqrt(gValue.x * gValue.x + gValue.y * gValue.y + gValue.z * gValue.z);
    float ax = gValue.x / normAcc;
    float ay = gValue.y / normAcc;
    float az = gValue.z / normAcc;

    // Compute pitch (altitude) and roll
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)); // radians
    float roll  = atan2(ay, az); // radians

    // Normalize magnetometer vector
    float normMag = sqrt(magValue.x * magValue.x + magValue.y * magValue.y + magValue.z * magValue.z);
    float mx = magValue.x / normMag;
    float my = magValue.y / normMag;
    float mz = magValue.z / normMag;

    // Tilt compensation for magnetometer
    float mx2 = mx * cos(pitch) + mz * sin(pitch);
    float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    // Azimuth (heading), in radians
    float azimuth = atan2(-my2, mx2);
    float azimuth_deg_local = (azimuth * 180.0 / PI); 
    if (azimuth_deg_local < 0) azimuth_deg += 360.0; // Normalize to 0-360

    float altitude_deg_local = -(pitch * 180.0 / PI);

    azimuth_deg = azimuth_deg_local;
    altitude_deg = altitude_deg_local;
    imuDataReady = true;

    vTaskDelay(50 / portTICK_PERIOD_MS);  // run at ~20Hz
  }
}

void TaskAudio(void *pvParameters) {
  while (true) {
    handlePlanetAudio();  // Adjust volume or switch track if needed

    if (wav && wav->isRunning()) {
      wav->loop();  // Keep audio playing
    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Light sleep
    }
  }
}

int hack;

void findOffsets() {
  for (int i = 0; i < 7; i++) {
  Planet& target = planets[i];
  float az_rad = azimuth_deg * PI / 180.0;
  float alt_rad = altitude_deg * PI / 180.0;

  // Convert to 3D unit vector
  float dx = cos(alt_rad) * sin(az_rad);
  float dy = cos(alt_rad) * cos(az_rad);
  float dz = sin(alt_rad);

  // Convert planet azimuth & altitude to radians
  float p_az_rad = target.Az * PI / 180.0;
  float p_alt_rad = target.Alt * PI / 180.0;

  // Convert to 3D unit vector
  float pdx = cos(p_alt_rad) * sin(p_az_rad);
  float pdy = cos(p_alt_rad) * cos(p_az_rad);
  float pdz = sin(p_alt_rad);

  // Dot product
  float dot = dx*pdx + dy*pdy + dz*pdz;
  dot = constrain(dot, -1.0, 1.0); // Prevent acos domain errors

  float angleBetween = acos(dot) * 180.0 / PI;

  target.offset = angleBetween;
  }
  //Serial.printf("Running on core: %d\n", xPortGetCoreID());
}

void closestBody() {
  closestOffset = planets[0].offset;
  for (int i = 1; i < 7; i++) {
    if (planets[i].offset < closestOffset) {
      closestOffset = planets[i].offset;
      closestPlanet = planets[i].name;
      closestDistance = planets[i].distanceAU;
    }
  }
}
// Sync DS1307 from NTP if Wi-Fi is up (else leave RTC as-is)
void updateTime() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org");
    delay(1500);

    struct tm gmt;
    if (getLocalTime(&gmt)) {
      rtc.adjust(DateTime(gmt.tm_year + 1900, gmt.tm_mon + 1, gmt.tm_mday,
                          gmt.tm_hour, gmt.tm_min, gmt.tm_sec));
      Serial.println("RTC updated from NTP.");
    } else {
      Serial.println("NTP time not available; keeping RTC time.");
    }
  } else {
    Serial.println("Wi-Fi down; using RTC time.");
  }
}

// Read DS1307 and build astro_time_t
void refreshDateandTime() {
  DateTime now = rtc.now();
  astroTime = Astronomy_MakeTime(now.year(), now.month(), now.day(),
                                 now.hour(), now.minute(), (double)now.second());
  // (optional) show the time you’re feeding AE:
  astro_utc_t utc = Astronomy_UtcFromTime(astroTime);
  Serial.printf("UTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                utc.year, utc.month, utc.day, utc.hour, utc.minute, (int)utc.second);
}

// map Planet.name -> astro_body_t
static bool nameToBody(const char* n, astro_body_t &body) {
  if      (!strcmp(n,"Mercury")) body = BODY_MERCURY;
  else if (!strcmp(n,"Venus"))   body = BODY_VENUS;
  else if (!strcmp(n,"Mars"))    body = BODY_MARS;
  else if (!strcmp(n,"Jupiter")) body = BODY_JUPITER;
  else if (!strcmp(n,"Saturn"))  body = BODY_SATURN;
  else if (!strcmp(n,"Uranus"))  body = BODY_URANUS;
  else if (!strcmp(n,"Neptune")) body = BODY_NEPTUNE;
  else return false;
  return true;
}

// Updates a single planet's RA/Dec, Az/Alt, and Earth distance (AU)
void refreshAzAlt(Planet& p) {
  astro_body_t body;
  if (!nameToBody(p.name, body)) return;

  // apparent RA/Dec at observer’s location (equator-of-date, with aberration)
  astro_equatorial_t equ = Astronomy_Equator(body, &astroTime, observer,
                                             EQUATOR_OF_DATE, ABERRATION);
  // store RA/Dec for your struct (degrees)
  p.RA  = equ.ra * 15.0;                // RA hours -> degrees
  p.dec = equ.dec;

  // convert to horizon (Az/Alt) for this observer
  astro_horizon_t horiz = Astronomy_Horizon(&astroTime, observer,
                                            equ.ra, equ.dec, REFRACTION_NORMAL);
  p.Az  = horiz.azimuth;
  p.Alt = horiz.altitude;

  // geocentric distance in AU: vector(body) - vector(Earth) in heliocentric coords
  astro_vector_t vPlanet = Astronomy_HelioVector(body, astroTime);       // by value
  astro_vector_t vEarth  = Astronomy_HelioVector(BODY_EARTH, astroTime); // by value
  double dx = vPlanet.x - vEarth.x;
  double dy = vPlanet.y - vEarth.y;
  double dz = vPlanet.z - vEarth.z;
  p.distanceAU = sqrt(dx*dx + dy*dy + dz*dz);

  // debug print
  Serial.printf("%s  RA: %.3f°  Dec: %.3f°  Az: %.3f°  Alt: %.3f°  Dist: %.3f AU\n",
                p.name, p.RA, p.dec, p.Az, p.Alt, p.distanceAU);
}

// refresh all planets in your array
void refreshPlanetsData() {
  for (int i = 0; i < (int)(sizeof(planets)/sizeof(planets[0])); ++i) {
    refreshAzAlt(planets[i]);
  }
}


void updateDisplay() {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextFont(4);
  tft.loadFont(AA_FONT_SMALL);
  sprite.loadFont(AA_FONT_SMALL);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK); 
  sprite.drawString("Source:", 82, 220);
  sprite.drawString("Distance:", 68, 255);

  if (closestOffset <= 4) {
    sprite.drawSmoothArc(centerX, 100, radius, inner_radius, LEFTstart_angle, LEFTend_angle, TFT_BLUE, TFT_BLACK, arc_end);
    sprite.drawSmoothArc(centerX, 100, radius, inner_radius, RIGHTstart_angle, RIGHTend_angle, TFT_BLUE, TFT_BLACK, arc_end);

    // Draw closest planet name below the semicircles
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);  // Center align
    //sprite.setFreeFont(FSSB18);     // Optional: you can choose other fonts, or comment this if not using free fonts
    sprite.drawString(closestPlanet, 175, 220); // Adjust Y for spacing
    sprite.drawFloat(closestDistance, 2, 160, 255);
    hack = 500;
  } else {
    sprite.drawSmoothArc(centerX - closestOffset, 100, radius, inner_radius, LEFTstart_angle, LEFTend_angle, TFT_GREEN, TFT_BLACK, arc_end);
    sprite.drawSmoothArc(centerX + closestOffset, 100, radius, inner_radius, RIGHTstart_angle, RIGHTend_angle, TFT_GREEN, TFT_BLACK, arc_end);
    hack = 50;
  }

  Serial.println(closestOffset);
  Serial.println(closestPlanet);

  tft.unloadFont();
  sprite.unloadFont();
  sprite.pushSprite(0, 0);
  delay(hack);
}

void setup() {

  pinMode(LED_LOCKOn, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH); // turn on power LED

  Wire.begin(32, 33); // SDA-32, SCL-33
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  
  tft.init();
  tft.setRotation(0);
  int16_t rc = png.openFLASH((uint8_t *)loading, sizeof(loading), pngDraw);
  if (rc == PNG_SUCCESS) {
    Serial.println("Successfully opened png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", 
      png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());

    tft.startWrite();
    uint32_t dt = millis();
    png.decode(NULL, 0); // draw it!
    Serial.print(millis() - dt); Serial.println("ms");
    tft.endWrite();
  }

  // Set up sprite the same size as the screen (or smaller, if you want)
  sprite.setColorDepth(8);  // You can use 16 for better color but more memory
  sprite.createSprite(tft.width(), tft.height());

  xTaskCreatePinnedToCore(
    TaskCore0,
    "TaskCore0",
    8192,
    NULL,
    1,
    &TaskCore0Handle,
    1 // Core 0
  );

  xTaskCreatePinnedToCore(
    TaskAudio,
    "TaskAudio",
    4096,
    NULL,
    10,              // High priority (10 is fine for audio)
    &AudioTaskHandle,
    0                // Run on Core 0
  );
  delay(500);

  unsigned long checkGPS = millis();
  while (millis() - checkGPS < 3000) { // Run for 3 seconds
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }

  if (gps.location.isValid()) {
    updateUserLocation();
  } else {
    latitude = 18.551464451319365;
    longitude = 73.79048766793403;     // use hard-coded coordinates if there is no GPS fix
  }
  observer.latitude  = latitude;     // degrees north
  observer.longitude = longitude;    // degrees east
  observer.height    = 0.0;          // meters above sea level (set yours if known)
  gpsSerial.end();
  delay(200);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  rtc.begin();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nWiFi connected. Syncing time...");

    configTime(IST_OFFSET_SECONDS, 0, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      DateTime now(
        timeinfo.tm_year + 1900,
        timeinfo.tm_mon + 1,
        timeinfo.tm_mday,
        timeinfo.tm_hour,
        timeinfo.tm_min,
        timeinfo.tm_sec
      );
      rtc.adjust(now);
      Serial.println("RTC updated from NTP.");
    } else {
      Serial.println("Failed to get time from NTP.");
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  mySPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, mySPI, 16000000)) {
    Serial.println("SD card initialization failed!");
    while (1);
  } else {
    Serial.println("SD card initialized.");
  }

  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  file = new AudioFileSourceSD("/starwars.wav");
  buff = new AudioFileSourceBuffer(file, 8192);  // You can try 4096 or 8192 too
  wav = new AudioGeneratorWAV();

  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");  //debugging
  } else {
    Serial.println("ICM20948 is connected");
  }

  if(!myIMU.initMagnetometer()){
  Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating..."); //debugging
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");  //debugging
  
  myIMU.enableAcc(true);
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setTempDLPF(ICM20948_DLPF_6);
  myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);   
  myIMU.setAccSampleRateDivider(10);

  #ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif
    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      while(1) delay(10);
    }

    if (!rtc.isrunning()) {
      Serial.println("RTC is NOT running, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
  tft.fillScreen(TFT_BLACK);
}

void loop() {

  if (imuDataReady) {
    imuDataReady = false;
    Serial.print("Azimuth (deg): ");
    Serial.println(azimuth_deg, 2);
    Serial.print("Altitude (deg): ");
    Serial.println(altitude_deg, 2);

    // Update planet calculations and display
    refreshDateandTime();
    updateDisplay();
    findOffsets();
    closestBody();

    if (millis() - lastUpdateTime >= 3000) {
      lastUpdateTime = millis();
      refreshPlanetsData();
    }
  }
}
