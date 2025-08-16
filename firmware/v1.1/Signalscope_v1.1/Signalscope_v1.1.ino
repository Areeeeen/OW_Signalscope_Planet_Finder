#include <Wire.h>
#include <ICM20948_WE.h>
#include <RTClib.h>
#include <SkyMap.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceBuffer.h>
#include <SPI.h>
#include <SD.h>
#include <PNGdec.h>
#include "loading.h" // The loading screen image data in flash array

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

PNG png;
#define MAX_IMAGE_WIDTH 240
int16_t xpos = 0;
int16_t ypos = 0;

TaskHandle_t TaskCore0Handle;
TaskHandle_t AudioTaskHandle;

ICM20948_WE myIMU = ICM20948_WE(0x69);
const float magX_offset = -32.515;
const float magY_offset = -1.275;
const float magZ_offset = -2.015;
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

SKYMAP_skymap_t skymap;
int year, month, day, hour, minute, second; // date and time will be read from rtc
double local_timezone_offset  = 5.5; // offset for IST (UTC +5:30 hrs)
double Time_utc;
double latitude = 18.551464451319365;
double longitude = 73.79048766793403;
double j2000;                // SKYMAP_days since jan 2000  - to be calculated
double Local_sidereal_time;  // to be calculated
double Hour_angle;           // to be calculated

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
Planet planets[] = {          // Create an array of 7 planets with their initial data  // UPDATE VALUES EVERY 3 DAYS
  {"Mercury", "/starwars.wav", 25.24139, 97.75},
  {"Venus",   "/starwars.wav",   11.21000, 34.93},
  {"Mars",    "/starwars.wav",    13.60028, 149.96},
  {"Jupiter", "/starwars.wav", 23.27500, 90.65},         // values accurate as of june 12th 2025
  {"Saturn",  "/starwars.wav",  -1.5161,  1.92},
  {"Uranus",  "/starwars.wav",  19.675,   56.58},
  {"Neptune", "/starwars.wav", -0.38861, 2.39}
};

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 5000;

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

AudioGeneratorWAV *wav = nullptr;
AudioFileSourceSD *file = nullptr;
AudioOutputI2S *out = nullptr;
AudioFileSourceBuffer *buff = nullptr;

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

    //Serial.printf("Running on core: %d\n", xPortGetCoreID());

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

void setup() {
  Wire.begin(32, 33); // SDA-32, SCL-33
  Serial.begin(115200);
  while(!Serial) {}

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

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

   unsigned long startAttemptTime = millis();

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
  } else {
    Serial.println("\nWiFi not connected. Skipping time sync.");
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

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
    updateDistances();
    updateDisplay();
    findOffsets();
    closestBody();

    if (millis() - lastUpdateTime >= updateInterval) {
      lastUpdateTime = millis();
      for (int i = 0; i < 7; i++) {
        refreshAzAlt(planets[i]);
      }
    }
  }
  delay(1);
}

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

void refreshAzAlt(Planet& p) {
  Local_sidereal_time = SKYMAP_local_sidereal_time(j2000, Time_utc, longitude);
  Hour_angle = SKYMAP_hour_angle(Local_sidereal_time, p.RA);

  SKYMAP_search_result_t result = SKYMAP_search_for_object(Hour_angle, p.dec, latitude);
  p.Az = result.azimuth;
  p.Alt = result.altitude;

  Serial.print("Planet: ");
  Serial.print(p.name);
  Serial.print(" | az: ");
  Serial.print(p.Az);
  Serial.print(" | alt: ");
  Serial.println(p.Alt);
}

void refreshDateandTime () {
  DateTime now = rtc.now();
  year = now.year();
  month = now.month();
  day = now.day();
  hour = now.hour();
  minute = now.minute();
  second = now.second();
   
  SKYMAP_date_time_values_t dt;
  dt.day = day;
  dt.month = month;
  dt.year = year;
  Time_utc = SKYMAP_hh_mm_ss2UTC(&dt, hour, minute, second, local_timezone_offset);
  j2000 = SKYMAP_j2000(&dt);
}

float getMercuryDistance(float j2000) {
  float phase = fmod(j2000, 116.0) / 116.0 * 2 * PI;
  return 0.61 + 0.25 * cos(phase);  // Approx: 0.31–0.91 AU

}
float getVenusDistance(float j2000) {
  float phase = fmod(j2000, 584.0) / 584.0 * 2 * PI;
  return 0.72 + 0.28 * cos(phase);  // Approx: 0.28–1.02 AU

}
float getMarsDistance(float j2000) {
  float phase = fmod(j2000, 780.0) / 780.0 * 2 * PI;
  return 1.52 + 0.98 * cos(phase);  // Approx: 0.52–2.5 AU

}
float getJupiterDistance(float j2000) {
  float phase = fmod(j2000, 399.0) / 399.0 * 2 * PI;
  return 4.2 + 0.8 * cos(phase);   // Approx: 4.2–5 AU
}
float getSaturnDistance(float j2000) {
  float phase = fmod(j2000, 378.0) / 378.0 * 2 * PI;
  return 8.5 + 0.8 * cos(phase);   // Approx: 8.5–9.3 AU

}
float getUranusDistance(float j2000) {
  float phase = fmod(j2000, 370.0) / 370.0 * 2 * PI;
  return 18.5 + 0.2 * cos(phase);  // Approx: 18.3–18.7 AU

}
float getNeptuneDistance(float j2000) {
  float phase = fmod(j2000, 367.0) / 367.0 * 2 * PI;
  return 29.0 + 0.2 * cos(phase);  // Approx: 28.8–29.2 AU

}
void updateDistances() { 

  planets[0].distanceAU = getMercuryDistance(j2000);
  planets[1].distanceAU = getVenusDistance(j2000);
  planets[2].distanceAU = getMarsDistance(j2000);
  planets[3].distanceAU = getJupiterDistance(j2000);
  planets[4].distanceAU = getSaturnDistance(j2000);
  planets[5].distanceAU = getUranusDistance(j2000);
  planets[6].distanceAU = getNeptuneDistance(j2000);

}

void updateDisplay() {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextFont(4);
  //tft.loadFont(spacemono);
  //sprite.loadFont(spacemono);
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

  //tft.unloadFont();
  //sprite.unloadFont();
  sprite.pushSprite(0, 0);
  delay(hack);
}