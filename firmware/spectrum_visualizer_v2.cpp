#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <arduinoFFT.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "hal/adc_types.h"


// ============================================================
// GLOBAL OPTIONS
// ============================================================

// Set to false to completely bypass dynamic compression.
const bool ENABLE_DYNAMIC_COMPRESSION =
  true;


// Set to false to keep the spectrum visible at all times.
const bool ENABLE_CLOCK_SCREEN =
  true;


// Dynamic compression settings.
const float COMP_PIVOT_DB =
  -30.0f;


const float COMP_RATIO =
  4.0f;


// Use only the compressor's calculated gain, capped at +5 dB below.
const float COMP_EXTRA_QUIET_BOOST_DB =
  0.0f;


const float COMP_MAX_BOOST_DB =
  5.0f;


const float COMP_MAX_CUT_DB =
  -18.0f;


const float PROGRAM_LEVEL_SMOOTH_MS =
  350.0f;


const float COMP_ATTACK_MS =
  1500.0f;


const float COMP_RELEASE_MS =
  600.0f;


// Noise protection settings used by the compressor.
// Two 40% amplitude reductions, then a 40% increase: 20 * log10(0.6 * 0.6 * 1.4).
const float NOISE_GATE_DB =
  -47.0f - 2.0f * 4.436975f + 2.922561f;


const float FULL_BOOST_FROM_DB =
  NOISE_GATE_DB + 1.0f;


const float NOISE_GAIN_RELEASE_MS =
  250.0f;


// ============================================================
// WIFI
// ============================================================

const char *WIFI_SSID =
  "SSID";

const char *WIFI_PASSWORD =
  "pass";


// Romania timezone including daylight-saving time

const char *TZ_INFO =
  "EET-2EEST,M3.5.0/3,M10.5.0/4";


volatile bool internetTimeValid =
  false;


// Once the clock is synchronized, weather updates pause in spectrum mode.
// WiFi stays active and reconnects independently of the display mode.
volatile bool auxiliaryServicesPaused =
  true;


TaskHandle_t weatherTaskHandle =
  nullptr;


const uint32_t WIFI_RETRY_INTERVAL_MS =
  10000;


// ============================================================
// GHIMBAV WEATHER
// ============================================================
//
// Ghimbav, Brasov, Romania
//
// Approx coordinates:
// 45.6635 N
// 25.5066 E
//
// Open-Meteo:
// current 2-meter air temperature
// ============================================================

const char *WEATHER_URL =
  "https://api.open-meteo.com/v1/forecast"
  "?latitude=45.6635"
  "&longitude=25.5066"
  "&current=temperature_2m"
  "&daily=temperature_2m_max"
  "&hourly=weather_code,is_day"
  "&forecast_days=2"
  "&temperature_unit=celsius"
  "&timezone=Europe%2FBucharest";


// Refresh every 10 minutes

const uint32_t WEATHER_UPDATE_INTERVAL_MS =
  10UL * 60UL * 1000UL;


// Retry faster if an update failed

const uint32_t WEATHER_RETRY_INTERVAL_MS =
  60UL * 1000UL;


volatile float outsideTemperatureC =
  NAN;


volatile bool weatherValid =
  false;


// Increment whenever new weather arrives.
// Clock code uses it to know that it must redraw.

volatile uint32_t weatherVersion =
  0;


uint32_t displayedWeatherVersion =
  0;


struct DailyForecast
{
  float maximumC;
  int conditionCode;
  char date[11];
};

struct WeatherReading
{
  float currentC;
  DailyForecast forecasts[2];
};

WeatherReading latestWeather = {NAN, {{NAN, -1, ""}, {NAN, -1, ""}}};
portMUX_TYPE weatherMux = portMUX_INITIALIZER_UNLOCKED;

void publishWeather(const WeatherReading &reading)
{
  portENTER_CRITICAL(&weatherMux);
  latestWeather = reading;
  outsideTemperatureC = reading.currentC;
  weatherValid = true;
  weatherVersion++;
  portEXIT_CRITICAL(&weatherMux);
}


// ============================================================
// CLOCK MODE LEVEL DETECTION
// ============================================================

const float CLOCK_ENTER_DB =
  NOISE_GATE_DB;


// Raise the previous exit amplitude by 40%: add 20 * log10(1.4) dB.
// Total exit threshold is 2.1 times the noise-gate RMS amplitude.
const float CLOCK_EXIT_DB =
  NOISE_GATE_DB + 3.521825f + 2.922561f;


// Must remain quiet for 30 seconds after audio ends.

const uint32_t CLOCK_ENTER_DELAY_MS =
  30000;


// Require two uninterrupted seconds of sustained audio to reject noise spikes.
const uint32_t CLOCK_EXIT_CONFIRM_MS =
  2000;


bool clockMode =
  false;


bool clockDisplayStarted = false;
uint32_t clockDisplayedSinceMs = 0;


// Give the startup clock screen three seconds before evaluating GPIO1 audio.
const uint32_t STARTUP_AUDIO_DELAY_MS =
  3000;


bool startupAudioDelayActive = false;


uint32_t lowSignalStartMs =
  0;


uint32_t audioReturnStartMs =
  0;


int lastClockMinute =
  -1;


// ============================================================
// MATRIX
// ============================================================

#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

// Values are ESP32 GPIO numbers; comments identify the J2 HUB75 pins.
#define R1_PIN   6  // J2 pin 1
#define G1_PIN   7  // J2 pin 2
#define B1_PIN   5  // J2 pin 3

#define R2_PIN   4  // J2 pin 5
#define G2_PIN   15 // J2 pin 6
#define B2_PIN   12 // J2 pin 7

#define A_PIN    11 // J2 pin 9
#define B_PIN    16 // J2 pin 10
#define C_PIN    21 // J2 pin 11
#define D_PIN    17 // J2 pin 12
#define E_PIN   -1

#define CLK_PIN  9  // J2 pin 13
#define LAT_PIN  18 // J2 pin 14
#define OE_PIN   8  // J2 pin 15: matrix OE, controlled by DMA
// Connector pins 4, 8 and 16 are GND.

// Separate active-low SN74HCT245N /OE, pulled up to 3.3 V with 10 kohm.
constexpr gpio_num_t BUFFER_OE_PIN = GPIO_NUM_13;

#define AUDIO_PIN 1

MatrixPanel_I2S_DMA *display =
  nullptr;


// ============================================================
// MAIN FFT
//
// 1024 @ 40 kHz
//
// resolution = 39.0625 Hz/bin
// hop        = 512 samples = 12.8 ms
// ============================================================

#define HIGH_FFT_SIZE 1024
#define HIGH_HOP_SIZE 512

#define HIGH_SAMPLE_RATE 40000.0f

float highWindow[HIGH_FFT_SIZE];

float highReal[HIGH_FFT_SIZE];

float highImag[HIGH_FFT_SIZE];


ArduinoFFT<float> highFFT(
  highReal,
  highImag,
  HIGH_FFT_SIZE,
  HIGH_SAMPLE_RATE,
  true // Cache window coefficients instead of recomputing cosines each frame.
);


// ============================================================
// LOW FFT
//
// 40 kHz / 10 = 4 kHz
//
// 512 @ 4 kHz (128 ms window)
//
// resolution = 7.8125 Hz/bin; assign each detected peak to one bass column
// ============================================================

#define LOW_DECIMATION 10

#define LOW_SAMPLE_RATE \
  (HIGH_SAMPLE_RATE / LOW_DECIMATION)

#define LOW_FFT_SIZE 512


float lowRing[LOW_FFT_SIZE];

float lowReal[LOW_FFT_SIZE];

float lowImag[LOW_FFT_SIZE];


ArduinoFFT<float> lowFFT(
  lowReal,
  lowImag,
  LOW_FFT_SIZE,
  LOW_SAMPLE_RATE,
  true // Reuse window coefficients on subsequent frames.
);


uint16_t lowWriteIndex =
  0;


uint16_t lowSampleCount =
  0;


// ============================================================
// DECIMATION FIR
// ============================================================

#define FIR_TAPS 127


const float FIR_CUTOFF_HZ =
  1500.0f;


float firCoeff[FIR_TAPS];

float firHistory[FIR_TAPS];


int firWriteIndex =
  0;


int decimationCounter =
  0;


const float PI_F =
  3.14159265358979323846f;


// ============================================================
// INPUT HIGH-PASS FILTER
// ============================================================

// Fourth-order Butterworth high-pass filter. Its cutoff is -3 dB at 30 Hz
// and frequencies below that point are attenuated at 24 dB per octave.
const float INPUT_HIGH_PASS_CUTOFF_HZ =
  30.0f;


#define INPUT_HIGH_PASS_STAGES 2


const float inputHighPassQ[INPUT_HIGH_PASS_STAGES] =
{
  0.5411961f,
  1.3065630f
};


struct HighPassBiquad
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float x1;
  float x2;
  float y1;
  float y2;
};


HighPassBiquad inputHighPass[INPUT_HIGH_PASS_STAGES];


// ============================================================
// ADC DMA
// ============================================================

adc_continuous_handle_t adcHandle =
  nullptr;


adc_unit_t adcUnit;

adc_channel_t adcChannel;


const uint8_t MATRIX_BRIGHTNESS_DARK =
  20;


const uint8_t MATRIX_BRIGHTNESS_LIGHT =
  255;


uint8_t matrixBrightness =
  MATRIX_BRIGHTNESS_LIGHT;


#define ADC_FRAME_BYTES \
  (HIGH_HOP_SIZE * SOC_ADC_DIGI_RESULT_BYTES)


uint8_t adcRawBuffer[ADC_FRAME_BYTES]
  __attribute__((aligned(4)));


float newRawHop[HIGH_HOP_SIZE];


// ============================================================
// SPECTRUM
// ============================================================

#define NUM_BANDS 64
#define LOW_BANDS 32


// ============================================================
// FREQUENCY MAPPING
//
// column 0             column 32             column 63
//
// 40 Hz                  1 kHz                 16 kHz
// ============================================================

const float MIN_FREQ =
  40.0f;


const float CENTER_FREQ =
  1000.0f;


const float MAX_FREQ =
  16000.0f;


// ============================================================
// ADC REFERENCE
// ============================================================

const float ADC_REFERENCE_AMPLITUDE =
  2048.0f;


// ============================================================
// DISPLAY RANGE
// ============================================================

const float DB_MIN =
  -55.0f;


// Reference level before vertical scaling, without changing audio processing.
const float DB_MAX =
  -30.0f;


// Scale before clamping so strong frequencies can still fill all 32 rows.
const float DISPLAY_HEIGHT_SCALE =
  0.75f;


// ============================================================
// STATIC HIGH-FREQUENCY VISUAL BOOST
// ============================================================

const float HIGH_FREQ_BOOST_DB =
  0.0f;


float bandBoostDb[NUM_BANDS];


// ============================================================
// LOW FFT CALIBRATION
// ============================================================

const float LOW_BRANCH_GAIN_DB =
  0.0f;


// ============================================================
// FAST BASS ATTACK ASSIST
// ============================================================

const float FAST_ATTACK_ASSIST =
  0.0f;


// ============================================================
// COMPRESSOR STATE
// ============================================================

float frameRmsDb =
  -120.0f;


float programLevelDb =
  -120.0f;


float globalGainDb =
  0.0f;


float desiredGlobalGainDb =
  0.0f;


bool programLevelInitialized =
  false;


uint32_t lastDynamicsTimeMs =
  0;


// ============================================================
// BAND DATA
// ============================================================

float bandTargetFrequency[NUM_BANDS];


float lowBandEdges[LOW_BANDS + 1];

// Frequency hysteresis only: no extra audio window or frame confirmation.
const float LOW_PEAK_HYSTERESIS = 0.15f; // Fraction of the assigned band's width.
const float LOW_PEAK_RELATIVE_FLOOR = 0.025f; // -32 dB; reject Hamming sidelobes.
int16_t previousLowPeakBand[LOW_FFT_SIZE / 2];
float previousLowPeakPosition[LOW_FFT_SIZE / 2];
bool lowPeakTrackingReady = false;
float lowPeakMagnitude[LOW_BANDS];


uint16_t fastAssistBin[LOW_BANDS];


bool fastAssistEnabled[LOW_BANDS];


uint16_t highBandStart[NUM_BANDS];


uint16_t highBandEnd[NUM_BANDS];


float rawBandDb[NUM_BANDS];


float fastBassDb[LOW_BANDS];


// ============================================================
// COLORS
// ============================================================

uint8_t bandR[NUM_BANDS];

uint8_t bandG[NUM_BANDS];

uint8_t bandB[NUM_BANDS];


// ============================================================
// DISPLAY LEVELS
// ============================================================

float bandLevel[NUM_BANDS];

float peakLevel[NUM_BANDS];


// ============================================================
// DISPLAY STATE
// ============================================================

int oldHeight[NUM_BANDS];

int oldPeakY[NUM_BANDS];


// ============================================================
// WIFI CONNECTION
// ============================================================

bool connectWiFi(
  uint32_t timeoutMs
)
{
  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    return true;
  }


  Serial.println();

  Serial.print(
    "Connecting to WiFi "
  );

  Serial.println(
    WIFI_SSID
  );


  WiFi.mode(
    WIFI_STA
  );


  WiFi.setAutoReconnect(
    true
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  uint32_t start =
    millis();


  while (
    WiFi.status() !=
      WL_CONNECTED
    &&
    millis() - start <
      timeoutMs
  )
  {
    delay(
      250
    );


    Serial.print(
      "."
    );
  }


  Serial.println();


  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    Serial.print(
      "WiFi connected. IP: "
    );


    Serial.println(
      WiFi.localIP()
    );


    return true;
  }


  Serial.println(
    "WiFi connection failed."
  );


  return false;
}


// ============================================================
// NTP CLOCK SYNC
// ============================================================

bool syncInternetClock()
{
  configTzTime(
    TZ_INFO,
    "pool.ntp.org",
    "time.google.com",
    "time.cloudflare.com"
  );


  Serial.println(
    "Synchronizing NTP clock..."
  );


  struct tm timeInfo;


  uint32_t start =
    millis();


  while (
    millis() - start <
    10000
  )
  {
    if (
      getLocalTime(
        &timeInfo,
        500
      )
    )
    {
      internetTimeValid =
        true;


      char buffer[32];


      strftime(
        buffer,
        sizeof(buffer),
        "%Y-%m-%d %H:%M:%S",
        &timeInfo
      );


      Serial.print(
        "Time synchronized: "
      );


      Serial.println(
        buffer
      );


      return true;
    }


    delay(
      100
    );
  }


  Serial.println(
    "NTP synchronization failed."
  );


  internetTimeValid =
    false;


  return false;
}


// ============================================================
// GET GHIMBAV TEMPERATURE
// ============================================================

// Read a numeric scalar or an indexed value of a daily array within one object.
// Reject null/missing/malformed values instead of silently displaying zero.
bool readWeatherNumber(const String &object, const char *key, bool array,
                       float &value, size_t index = 0)
{
  String token = String("\"") + key + "\"";
  int position = object.indexOf(token);
  if (position < 0) return false;
  position = object.indexOf(':', position + token.length());
  if (position < 0) return false;
  const char *start = object.c_str() + position + 1;
  while (*start == ' ' || *start == '\n' || *start == '\r' || *start == '\t') ++start;
  if (array)
  {
    if (*start++ != '[') return false;
    while (*start == ' ' || *start == '\n' || *start == '\r' || *start == '\t') ++start;
    for (size_t i = 0; i < index; ++i)
    {
      char *end;
      const float skipped = strtof(start, &end);
      if (end == start || !isfinite(skipped)) return false;
      while (*end == ' ' || *end == '\n' || *end == '\r' || *end == '\t') ++end;
      if (*end != ',') return false;
      start = end + 1;
    }
  }
  char *end;
  value = strtof(start, &end);
  if (end == start || !isfinite(value)) return false;
  while (*end == ' ' || *end == '\n' || *end == '\r' || *end == '\t') ++end;
  return *end == ',' || *end == '}' || (array && *end == ']');
}

bool weatherObject(const String &payload, const char *key, String &object)
{
  int position = payload.indexOf(String("\"") + key + "\"");
  if (position < 0) return false;
  int begin = payload.indexOf('{', position);
  int end = payload.indexOf('}', begin);
  if (begin < 0 || end < 0) return false;
  object = payload.substring(begin, end + 1);
  return true;
}

// All weather parsing stays in the background task, away from audio sampling.
const char *skipWeatherWhitespace(const char *text)
{
  while (*text == ' ' || *text == '\n' || *text == '\r' || *text == '\t') ++text;
  return text;
}

// Forecast timestamps are plain ISO strings without JSON escape sequences.
bool readWeatherTime(const String &object, size_t index, char *value, size_t capacity)
{
  int position = object.indexOf("\"time\"");
  if (position < 0) return false;
  position = object.indexOf(':', position + 6);
  if (position < 0) return false;
  const char *cursor = skipWeatherWhitespace(object.c_str() + position + 1);
  if (*cursor != '[') return false;
  cursor = skipWeatherWhitespace(cursor + 1);
  for (size_t i = 0; ; ++i)
  {
    if (*cursor != '"') return false;
    const char *begin = cursor + 1;
    const char *end = strchr(begin, '"');
    if (!end) return false;
    cursor = skipWeatherWhitespace(end + 1);
    if (*cursor != ',' && *cursor != ']') return false;
    if (i == index)
    {
      const size_t length = end - begin;
      if (length + 1 != capacity || begin[4] != '-' || begin[7] != '-') return false;
      memcpy(value, begin, length);
      value[length] = '\0';
      return true;
    }
    if (*cursor != ',') return false;
    cursor = skipWeatherWhitespace(cursor + 1);
  }
}

bool readWeatherIntegerArray(const String &object, const char *key,
                             int *values, size_t capacity, size_t &count)
{
  count = 0;
  const String token = String("\"") + key + "\"";
  int position = object.indexOf(token);
  if (position < 0) return false;
  position = object.indexOf(':', position + token.length());
  if (position < 0) return false;
  const char *cursor = skipWeatherWhitespace(object.c_str() + position + 1);
  if (*cursor != '[') return false;
  cursor = skipWeatherWhitespace(cursor + 1);
  if (*cursor == ']') return false;
  while (true)
  {
    if (count >= capacity) return false;
    char *end;
    const long value = strtol(cursor, &end, 10);
    if (end == cursor || value < 0 || value > 99) return false;
    values[count++] = (int)value;
    cursor = skipWeatherWhitespace(end);
    if (*cursor == ']') return true;
    if (*cursor != ',') return false;
    cursor = skipWeatherWhitespace(cursor + 1);
  }
}

// Count the conditions we actually display, combining intensity variants.
// Clear/mainly clear both display SUNNY; rain intensities all display RAIN.
constexpr int weatherConditionGroup(int code)
{
  switch (code)
  {
    case 0: case 1: return 0;
    case 2: case 3: return code;
    case 45: case 48: return 45;
    case 51: case 53: case 55: return 51;
    case 56: case 57: return 56;
    case 61: case 63: case 65: return 61;
    case 66: case 67: return 66;
    case 71: case 73: case 75: case 77: case 85: case 86: return 71;
    case 80: case 81: case 82: return 80;
    case 95: case 97: return 95;
    case 96: case 99: return 96;
    default: return -1;
  }
}

constexpr bool predominantWeatherCode(const int *codes, const int *daylight,
                            size_t count, int &conditionCode)
{
  // A complete local day can contain 23, 24 or 25 hours around DST changes.
  if (count < 23 || count > 25) return false;
  uint8_t hours[100] = {};
  uint8_t daylightHours[100] = {};
  for (size_t i = 0; i < count; ++i)
  {
    const int group = weatherConditionGroup(codes[i]);
    if (group < 0 || (daylight[i] != 0 && daylight[i] != 1)) return false;
    ++hours[group];
    daylightHours[group] += daylight[i];
  }

  int best = -1;
  for (int group = 0; group < 100; ++group)
  {
    if (hours[group] == 0) continue;
    if (best < 0 || hours[group] > hours[best] ||
        (hours[group] == hours[best] && daylightHours[group] > daylightHours[best]))
      best = group;
  }
  // Equal totals prefer more daylight hours; a remaining tie uses the lower
  // representative code for a stable result, never the worst condition.
  conditionCode = best;
  return best >= 0;
}

bool readPredominantWeather(const String &hourly, DailyForecast *forecasts)
{
  int codes[50], daylight[50];
  size_t codeCount, daylightCount;
  if (!readWeatherIntegerArray(hourly, "weather_code", codes, 50, codeCount) ||
      !readWeatherIntegerArray(hourly, "is_day", daylight, 50, daylightCount) ||
      codeCount != daylightCount) return false;

  int dailyCodes[2][25], dailyDaylight[2][25];
  size_t dailyCount[2] = {};
  for (size_t hour = 0; hour < codeCount; ++hour)
  {
    char timestamp[17];
    if (!readWeatherTime(hourly, hour, timestamp, sizeof(timestamp)) ||
        timestamp[10] != 'T') return false;
    const int day = strncmp(timestamp, forecasts[0].date, 10) == 0 ? 0 :
                    strncmp(timestamp, forecasts[1].date, 10) == 0 ? 1 : -1;
    if (day < 0 || dailyCount[day] >= 25) return false;
    const size_t slot = dailyCount[day]++;
    dailyCodes[day][slot] = codes[hour];
    dailyDaylight[day][slot] = daylight[hour];
  }
  for (int day = 0; day < 2; ++day)
    if (!predominantWeatherCode(dailyCodes[day], dailyDaylight[day], dailyCount[day],
                                forecasts[day].conditionCode)) return false;
  return true;
}

bool fetchWeather(WeatherReading &reading)
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return false;
  }


  WiFiClientSecure client;


  // HTTPS encryption is still used, but we do not install
  // a CA certificate in this embedded application.

  client.setInsecure();


  HTTPClient https;


  https.setTimeout(
    5000
  );


  if (
    !https.begin(
      client,
      WEATHER_URL
    )
  )
  {
    return false;
  }


  int httpCode =
    https.GET();


  if (
    httpCode !=
    HTTP_CODE_OK
  )
  {
    Serial.print(
      "Weather HTTP error: "
    );


    Serial.println(
      httpCode
    );


    https.end();


    return false;
  }


  String payload =
    https.getString();


  https.end();


  String current, daily, hourly;
  if (!weatherObject(payload, "current", current) ||
      !weatherObject(payload, "daily", daily) ||
      !weatherObject(payload, "hourly", hourly) ||
      !readWeatherNumber(current, "temperature_2m", false, reading.currentC)) return false;
  if (reading.currentC < -100 || reading.currentC > 100) return false;

  for (int day = 0; day < 2; ++day)
  {
    DailyForecast &forecast = reading.forecasts[day];
    if (!readWeatherTime(daily, day, forecast.date, sizeof(forecast.date)) ||
        !readWeatherNumber(daily, "temperature_2m_max", true, forecast.maximumC, day) ||
        forecast.maximumC < -100 || forecast.maximumC > 100) return false;
  }
  return readPredominantWeather(hourly, reading.forecasts);
}


// ============================================================
// BACKGROUND WEATHER TASK
//
// Runs separately so HTTPS requests don't stop the FFT loop.
//
// On ESP32-S3 the Arduino application normally has another
// CPU core available. We pin this low-priority job to core 0.
// ============================================================

void setAuxiliaryServicesPaused(bool paused)
{
  if (
    auxiliaryServicesPaused ==
    paused
  )
  {
    return;
  }


  auxiliaryServicesPaused =
    paused;


  if (
    weatherTaskHandle !=
    nullptr
  )
  {
    xTaskNotifyGive(weatherTaskHandle);
  }
}


bool auxiliaryServicesNeeded()
{
  // A failed startup connection must recover even while spectrum is showing.
  return !auxiliaryServicesPaused || (ENABLE_CLOCK_SCREEN && !internetTimeValid);
}

void weatherTask(void *parameter)
{
  uint32_t lastWiFiAttemptMs = 0;
  bool wifiRetryPending = false;
  bool wasConnected = WiFi.status() == WL_CONNECTED;
  bool ntpConfigured = internetTimeValid;
  uint32_t lastWeatherAttemptMs = millis();
  uint32_t nextWeatherDelayMs = weatherValid ? WEATHER_UPDATE_INTERVAL_MS : 0;

  while (true)
  {
    const uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED)
    {
      wasConnected = false;
      // Retry independently of the weather timer, without blocking the FFT.
      if (!wifiRetryPending ||
          (uint32_t)(now - lastWiFiAttemptMs) >= WIFI_RETRY_INTERVAL_MS)
      {
        lastWiFiAttemptMs = now;
        wifiRetryPending = true;
        Serial.println("WiFi disconnected; retrying connection...");
        if (WiFi.getMode() == WIFI_OFF)
        {
          WiFi.mode(WIFI_STA);
          WiFi.setAutoReconnect(true);
          WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
        else
        {
          WiFi.setAutoReconnect(true);
          if (!WiFi.reconnect())
          {
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
          }
        }
      }
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
      continue;
    }

    wifiRetryPending = false;
    if (!wasConnected)
    {
      wasConnected = true;
      nextWeatherDelayMs = 0;
      Serial.print("WiFi reconnected. IP: ");
      Serial.println(WiFi.localIP());
    }

    // Configure NTP after recovery if startup never reached clock setup.
    if (!ntpConfigured)
    {
      configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
      ntpConfigured = true;
    }
    if (!internetTimeValid)
    {
      struct tm timeInfo;
      if (getLocalTime(&timeInfo, 0))
      {
        internetTimeValid = true;
      }
    }

    if (auxiliaryServicesNeeded() &&
        (nextWeatherDelayMs == 0 ||
         (uint32_t)(now - lastWeatherAttemptMs) >= nextWeatherDelayMs))
    {
      WeatherReading newWeather;
      const bool success = fetchWeather(newWeather);
      lastWeatherAttemptMs = millis();
      nextWeatherDelayMs = success
        ? WEATHER_UPDATE_INTERVAL_MS : WEATHER_RETRY_INTERVAL_MS;
      if (success)
      {
        publishWeather(newWeather);
        Serial.print("Ghimbav temperature: ");
        Serial.print(newWeather.currentC, 1);
        Serial.println(" C");
      }
    }

    // Check connectivity every second; notifications request an early refresh.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) > 0)
    {
      nextWeatherDelayMs = 0;
    }
  }
}


// ============================================================
// INITIAL INTERNET SETUP
// ============================================================

void setupInternetServices()
{
  if (connectWiFi(12000))
  {
    syncInternetClock();

    WeatherReading initialWeather;
    if (fetchWeather(initialWeather))
    {
      publishWeather(initialWeather);
      Serial.print("Initial Ghimbav temperature: ");
      Serial.print(initialWeather.currentC, 1);
      Serial.println(" C");
    }
    else
    {
      Serial.println("Initial weather request failed.");
    }
  }
  else
  {
    Serial.println("Clock/weather unavailable until WiFi connects.");
  }

  // Start recovery even if the initial WiFi connection failed.
  const BaseType_t taskResult = xTaskCreatePinnedToCore(
    weatherTask,
    "weatherTask",
    12288,
    nullptr,
    1,
    &weatherTaskHandle,
    0
  );
  if (taskResult != pdPASS)
  {
    Serial.println("Failed to start WiFi/weather recovery task.");
  }
}


// ============================================================
// SPECTRUM COLOR
// ============================================================

void spectrumColor(
  int band,
  uint8_t &r,
  uint8_t &g,
  uint8_t &b
)
{
  float position =
    (
      (float)band /
      (NUM_BANDS - 1)
    )
    *
    5.0f;


  int section =
    (int)position;


  float f =
    position -
    section;


  if (
    section >=
    5
  )
  {
    section =
      4;


    f =
      1.0f;
  }


  switch (section)
  {
    case 0:

      r =
        255;


      g =
        (uint8_t)(
          255.0f *
          f
        );


      b =
        0;

      break;


    case 1:

      r =
        (uint8_t)(
          255.0f *
          (1.0f - f)
        );


      g =
        255;


      b =
        0;

      break;


    case 2:

      r =
        0;


      g =
        255;


      b =
        (uint8_t)(
          255.0f *
          f
        );

      break;


    case 3:

      r =
        0;


      g =
        (uint8_t)(
          255.0f *
          (1.0f - f)
        );


      b =
        255;

      break;


    default:

      r =
        (uint8_t)(
          180.0f *
          f
        );


      g =
        0;


      b =
        255;

      break;
  }
}


// ============================================================
// SCREEN POSITION -> FREQUENCY
// ============================================================

float positionToFrequency(
  float position
)
{
  if (
    position <=
    0.5f
  )
  {
    float t =
      position /
      0.5f;


    return
      MIN_FREQ *
      powf(
        CENTER_FREQ /
        MIN_FREQ,
        t
      );
  }


  float t =
    (
      position -
      0.5f
    )
    /
    0.5f;


  return
    CENTER_FREQ *
    powf(
      MAX_FREQ /
      CENTER_FREQ,
      t
    );
}


// ============================================================
// FREQUENCY -> FIRST HIGH FFT BIN AT OR ABOVE THE BAND EDGE
// ============================================================

int highFreqToBin(
  float frequency
)
{
  int bin =
    (int)ceilf(
      frequency *
      HIGH_FFT_SIZE /
      HIGH_SAMPLE_RATE
    );


  if (
    bin <
    1
  )
  {
    bin =
      1;
  }


  if (
    bin >
    HIGH_FFT_SIZE / 2 - 1
  )
  {
    bin =
      HIGH_FFT_SIZE / 2 - 1;
  }


  return bin;
}


// ============================================================
// FIR SETUP
// ============================================================

void setupLowPassFIR()
{
  float normalizedCutoff =
    FIR_CUTOFF_HZ /
    HIGH_SAMPLE_RATE;


  float sum =
    0.0f;


  int middle =
    (FIR_TAPS - 1) /
    2;


  for (
    int n = 0;
    n < FIR_TAPS;
    n++
  )
  {
    int m =
      n -
      middle;


    float h;


    if (
      m ==
      0
    )
    {
      h =
        2.0f *
        normalizedCutoff;
    }

    else
    {
      h =
        sinf(
          2.0f *
          PI_F *
          normalizedCutoff *
          m
        )
        /
        (
          PI_F *
          m
        );
    }


    float w =
      0.54f
      -
      0.46f *
      cosf(
        2.0f *
        PI_F *
        n /
        (FIR_TAPS - 1)
      );


    firCoeff[n] =
      h *
      w;


    sum +=
      firCoeff[n];
  }


  for (
    int n = 0;
    n < FIR_TAPS;
    n++
  )
  {
    firCoeff[n] /=
      sum;


    firHistory[n] =
      0.0f;
  }


  firWriteIndex =
    0;


  decimationCounter =
    0;
}


// ============================================================
// LOW SAMPLE RING
// ============================================================

void appendLowSample(
  float sample
)
{
  lowRing[lowWriteIndex] =
    sample;


  lowWriteIndex++;


  if (
    lowWriteIndex >=
    LOW_FFT_SIZE
  )
  {
    lowWriteIndex =
      0;
  }


  if (
    lowSampleCount <
    LOW_FFT_SIZE
  )
  {
    lowSampleCount++;
  }
}


// ============================================================
// FIR + DECIMATION
// ============================================================

void processDecimatorSample(
  float sample
)
{
  firHistory[firWriteIndex] =
    sample;


  firWriteIndex++;


  if (
    firWriteIndex >=
    FIR_TAPS
  )
  {
    firWriteIndex =
      0;
  }


  decimationCounter++;


  if (
    decimationCounter <
    LOW_DECIMATION
  )
  {
    return;
  }


  decimationCounter =
    0;


  float output =
    0.0f;


  int index =
    firWriteIndex;


  for (
    int n = 0;
    n < FIR_TAPS;
    n++
  )
  {
    output +=
      firCoeff[n] *
      firHistory[index];


    index++;


    if (
      index >=
      FIR_TAPS
    )
    {
      index =
        0;
    }
  }


  appendLowSample(
    output
  );
}


// ============================================================
// PROCESS RAW BLOCK THROUGH DECIMATOR
// ============================================================

void processDecimatorBlock(
  float *samples,
  int count
)
{
  for (
    int i = 0;
    i < count;
    i++
  )
  {
    processDecimatorSample(
      samples[i]
    );
  }
}


// ============================================================
// COPY LOW BUFFER INTO FFT BUFFER
// ============================================================

void copyLowWindowToFFT()
{
  int index =
    lowWriteIndex;


  for (
    int i = 0;
    i < LOW_FFT_SIZE;
    i++
  )
  {
    lowReal[i] =
      lowRing[index];


    lowImag[i] =
      0.0f;


    index++;


    if (
      index >=
      LOW_FFT_SIZE
    )
    {
      index =
        0;
    }
  }
}


// ============================================================
// PREPARE FREQUENCY BANDS
// ============================================================

void prepareBands()
{
  for (
    int band = 0;
    band < NUM_BANDS;
    band++
  )
  {
    float leftPosition =
      (float)band /
      NUM_BANDS;


    float rightPosition =
      (float)(band + 1) /
      NUM_BANDS;


    float centerPosition =
      (
        (float)band +
        0.5f
      )
      /
      NUM_BANDS;


    float lowFreq =
      positionToFrequency(
        leftPosition
      );


    float highFreq =
      positionToFrequency(
        rightPosition
      );


    float centerFreq =
      positionToFrequency(
        centerPosition
      );


    bandTargetFrequency[band] =
      centerFreq;


    bandBoostDb[band] =
      (
        (float)band /
        (NUM_BANDS - 1)
      )
      *
      HIGH_FREQ_BOOST_DB;


    // ========================================================
    // LOW 32 COLUMNS
    // ========================================================

    if (
      band <
      LOW_BANDS
    )
    {
      // Coarse fast-attack bins must not drive multiple narrow bass columns.
      fastAssistEnabled[band] =
        (highFreq - lowFreq) >= (HIGH_SAMPLE_RATE / HIGH_FFT_SIZE);

      lowBandEdges[band] = lowFreq;
      lowBandEdges[band + 1] = highFreq;


      int fastBin =
        (int)lroundf(
          centerFreq *
          HIGH_FFT_SIZE /
          HIGH_SAMPLE_RATE
        );


      if (
        fastBin <
        1
      )
      {
        fastBin =
          1;
      }


      if (
        fastBin >
        HIGH_FFT_SIZE / 2 - 1
      )
      {
        fastBin =
          HIGH_FFT_SIZE / 2 - 1;
      }


      fastAssistBin[band] =
        fastBin;
    }


    // ========================================================
    // HIGH 32 COLUMNS
    // ========================================================

    else
    {
      int startBin =
        highFreqToBin(
          lowFreq
        );


      int endBin =
        highFreqToBin(
          highFreq
        ) - (band == NUM_BANDS - 1 ? 0 : 1);

      // Use [lowFreq, highFreq), as in the low FFT: each bin belongs to
      // one column. Inclusive shared edge bins produced duplicate bars.


      if (
        endBin <
        startBin
      )
      {
        endBin =
          startBin;
      }


      highBandStart[band] =
        startBin;


      highBandEnd[band] =
        endBin;
    }


    spectrumColor(
      band,
      bandR[band],
      bandG[band],
      bandB[band]
    );
  }
}


// ============================================================
// ADC ERROR
// ============================================================

void adcFatal(
  const char *message,
  esp_err_t error
)
{
  Serial.print(
    message
  );


  Serial.print(
    ": "
  );


  Serial.println(
    esp_err_to_name(
      error
    )
  );


  while (true)
  {
    delay(
      1000
    );
  }
}


// ============================================================
// ADC DMA SETUP
// ============================================================

void setupADC_DMA()
{
  esp_err_t err =
    adc_continuous_io_to_channel(
      AUDIO_PIN,
      &adcUnit,
      &adcChannel
    );


  if (
    err !=
    ESP_OK
  )
  {
    adcFatal(
      "Invalid ADC pin",
      err
    );
  }


  if (
    adcUnit !=
    ADC_UNIT_1
  )
  {
    Serial.println(
      "ADC continuous requires ADC1"
    );


    while (true)
    {
      delay(
        1000
      );
    }
  }


  adc_continuous_handle_cfg_t handleConfig =
    {};


  handleConfig.max_store_buf_size =
    ADC_FRAME_BYTES *
    16;


  handleConfig.conv_frame_size =
    ADC_FRAME_BYTES;


  err =
    adc_continuous_new_handle(
      &handleConfig,
      &adcHandle
    );


  if (
    err !=
    ESP_OK
  )
  {
    adcFatal(
      "adc_continuous_new_handle",
      err
    );
  }


  adc_digi_pattern_config_t pattern[1] =
    {};


  pattern[0].atten =
    ADC_ATTEN_DB_12;


  pattern[0].channel =
    (uint8_t)
    adcChannel;


  pattern[0].unit =
    (uint8_t)
    ADC_UNIT_1;


  pattern[0].bit_width =
    SOC_ADC_DIGI_MAX_BITWIDTH;


  adc_continuous_config_t adcConfig =
    {};


  adcConfig.sample_freq_hz =
    (uint32_t)
    (
      HIGH_SAMPLE_RATE
    );


  adcConfig.conv_mode =
    ADC_CONV_SINGLE_UNIT_1;


  adcConfig.format =
    ADC_DIGI_OUTPUT_FORMAT_TYPE2;


  adcConfig.pattern_num =
    1;


  adcConfig.adc_pattern =
    pattern;


  err =
    adc_continuous_config(
      adcHandle,
      &adcConfig
    );


  if (
    err !=
    ESP_OK
  )
  {
    adcFatal(
      "adc_continuous_config",
      err
    );
  }


  err =
    adc_continuous_start(
      adcHandle
    );


  if (
    err !=
    ESP_OK
  )
  {
    adcFatal(
      "adc_continuous_start",
      err
    );
  }
}


// ============================================================
// READ ADC
// ============================================================

bool readRawHop(
  float *destination
)
{
  size_t samplesCopied =
    0;


  while (
    samplesCopied <
    HIGH_HOP_SIZE
  )
  {
    uint32_t bytesRead =
      0;


    esp_err_t err =
      adc_continuous_read(
        adcHandle,
        adcRawBuffer,
        sizeof(adcRawBuffer),
        &bytesRead,
        1000
      );


    if (
      err !=
      ESP_OK
    )
    {
      return false;
    }


    for (
      uint32_t i = 0;

      i +
      SOC_ADC_DIGI_RESULT_BYTES
      <=
      bytesRead;

      i +=
      SOC_ADC_DIGI_RESULT_BYTES
    )
    {
      adc_digi_output_data_t *sample =
        (adc_digi_output_data_t *)
        &adcRawBuffer[i];


      if (
        sample->type2.channel ==
        (uint32_t)
        adcChannel
      )
      {
        destination[samplesCopied] =
          (float)
          sample->type2.data;


        samplesCopied++;


        if (
          samplesCopied >=
          HIGH_HOP_SIZE
        )
        {
          break;
        }
      }


    }
  }

  return true;
}


// ============================================================
// CONSUME AUDIO BLOCK
// ============================================================

void setupInputHighPass()
{
  const float omega =
    2.0f *
    PI_F *
    INPUT_HIGH_PASS_CUTOFF_HZ /
    HIGH_SAMPLE_RATE;


  const float sine =
    sinf(omega);


  const float cosine =
    cosf(omega);


  for (
    int stage = 0;
    stage < INPUT_HIGH_PASS_STAGES;
    ++stage
  )
  {
    HighPassBiquad &filter =
      inputHighPass[stage];


    const float alpha =
      sine /
      (2.0f * inputHighPassQ[stage]);


    const float a0 =
      1.0f +
      alpha;


    filter.b0 =
      (1.0f + cosine) /
      (2.0f * a0);


    filter.b1 =
      -(1.0f + cosine) /
      a0;


    filter.b2 =
      filter.b0;


    filter.a1 =
      -2.0f *
      cosine /
      a0;


    filter.a2 =
      (1.0f - alpha) /
      a0;


    filter.x1 =
      0.0f;


    filter.x2 =
      0.0f;


    filter.y1 =
      0.0f;


    filter.y2 =
      0.0f;
  }
}


float applyInputHighPass(float sample)
{
  float output =
    sample;


  for (
    int stage = 0;
    stage < INPUT_HIGH_PASS_STAGES;
    ++stage
  )
  {
    HighPassBiquad &filter =
      inputHighPass[stage];


    const float filtered =
      filter.b0 * output +
      filter.b1 * filter.x1 +
      filter.b2 * filter.x2 -
      filter.a1 * filter.y1 -
      filter.a2 * filter.y2;


    filter.x2 =
      filter.x1;


    filter.x1 =
      output;


    filter.y2 =
      filter.y1;


    filter.y1 =
      filtered;


    output =
      filtered;
  }


  return output;
}


void consumeRawHop(
  float *samples
)
{
  for (
    int i = 0;
    i < HIGH_HOP_SIZE;
    ++i
  )
  {
    samples[i] =
      applyInputHighPass(samples[i]);
  }


  memmove(
    &highWindow[0],
    &highWindow[HIGH_HOP_SIZE],
    HIGH_HOP_SIZE *
    sizeof(float)
  );


  memcpy(
    &highWindow[HIGH_HOP_SIZE],
    samples,
    HIGH_HOP_SIZE *
    sizeof(float)
  );


  processDecimatorBlock(
    samples,
    HIGH_HOP_SIZE
  );
}


// ============================================================
// PRIME FFT WINDOWS
// ============================================================

void primeAudioWindows()
{
  while (
    lowSampleCount <
    LOW_FFT_SIZE
  )
  {
    if (
      !readRawHop(
        newRawHop
      )
    )
    {
      continue;
    }


    consumeRawHop(
      newRawHop
    );
  }
}


// ============================================================
// ADVANCE AUDIO
// ============================================================

bool advanceAudioWindows()
{
  if (
    !readRawHop(
      newRawHop
    )
  )
  {
    return false;
  }


  consumeRawHop(
    newRawHop
  );


  return true;
}


// ============================================================
// AMPLITUDE -> DB
// ============================================================

float amplitudeToDb(
  float amplitude
)
{
  float normalized =
    amplitude /
    ADC_REFERENCE_AMPLITUDE;


  if (
    normalized <=
    0.000001f
  )
  {
    return
      -120.0f;
  }


  return
    20.0f *
    log10f(
      normalized
    );
}


// ============================================================
// OVERALL AUDIO RMS
// ============================================================

float calculateFrameRmsDb()
{
  double mean =
    0.0;


  for (
    int i = 0;
    i < HIGH_FFT_SIZE;
    i++
  )
  {
    mean +=
      highWindow[i];
  }


  mean /=
    HIGH_FFT_SIZE;


  double sumSquares =
    0.0;


  for (
    int i = 0;
    i < HIGH_FFT_SIZE;
    i++
  )
  {
    double sample =
      (double)
      highWindow[i]
      -
      mean;


    sumSquares +=
      sample *
      sample;
  }


  float rms =
    sqrtf(
      (float)(
        sumSquares /
        HIGH_FFT_SIZE
      )
    );


  return
    amplitudeToDb(
      rms
    );
}


// ============================================================
// GLOBAL COMPRESSOR
// ============================================================

void updateGlobalCompressor(
  float currentFrameRmsDb
)
{
  if (
    !ENABLE_DYNAMIC_COMPRESSION
  )
  {
    globalGainDb =
      0.0f;


    desiredGlobalGainDb =
      0.0f;


    programLevelInitialized =
      false;


    return;
  }


  uint32_t now =
    millis();


  float dt =
    (float)(
      now -
      lastDynamicsTimeMs
    );


  lastDynamicsTimeMs =
    now;


  if (
    dt <=
    0.0f
  )
  {
    return;
  }


  if (
    dt >
    100.0f
  )
  {
    dt =
      100.0f;
  }


  if (
    !programLevelInitialized
  )
  {
    programLevelDb =
      currentFrameRmsDb;


    programLevelInitialized =
      true;
  }

  else
  {
    float detectorCoefficient =
      1.0f -
      expf(
        -dt /
        PROGRAM_LEVEL_SMOOTH_MS
      );


    programLevelDb +=
      (
        currentFrameRmsDb -
        programLevelDb
      )
      *
      detectorCoefficient;
  }


  desiredGlobalGainDb =
    -(
      programLevelDb -
      COMP_PIVOT_DB
    )
    *
    (
      1.0f -
      1.0f /
      COMP_RATIO
    );


  if (desiredGlobalGainDb > 0.0f)
  {
    desiredGlobalGainDb += COMP_EXTRA_QUIET_BOOST_DB;
  }


  if (
    desiredGlobalGainDb >
    COMP_MAX_BOOST_DB
  )
  {
    desiredGlobalGainDb =
      COMP_MAX_BOOST_DB;
  }


  if (
    desiredGlobalGainDb <
    COMP_MAX_CUT_DB
  )
  {
    desiredGlobalGainDb =
      COMP_MAX_CUT_DB;
  }


  if (
    desiredGlobalGainDb >
    0.0f
  )
  {
    float allowedBoost;


    if (
      programLevelDb <=
      NOISE_GATE_DB
    )
    {
      allowedBoost =
        0.0f;
    }

    else if (
      programLevelDb >=
      FULL_BOOST_FROM_DB
    )
    {
      allowedBoost =
        COMP_MAX_BOOST_DB;
    }

    else
    {
      float position =
        (
          programLevelDb -
          NOISE_GATE_DB
        )
        /
        (
          FULL_BOOST_FROM_DB -
          NOISE_GATE_DB
        );


      allowedBoost =
        position *
        COMP_MAX_BOOST_DB;
    }


    if (
      desiredGlobalGainDb >
      allowedBoost
    )
    {
      desiredGlobalGainDb =
        allowedBoost;
    }
  }


  float timeConstant;


  if (
    programLevelDb <=
      NOISE_GATE_DB
    &&
    globalGainDb >
      0.0f
  )
  {
    timeConstant =
      NOISE_GAIN_RELEASE_MS;
  }

  else if (
    desiredGlobalGainDb <
    globalGainDb
  )
  {
    timeConstant =
      COMP_ATTACK_MS;
  }

  else
  {
    timeConstant =
      COMP_RELEASE_MS;
  }


  float coefficient =
    1.0f -
    expf(
      -dt /
      timeConstant
    );


  globalGainDb +=
    (
      desiredGlobalGainDb -
      globalGainDb
    )
    *
    coefficient;
}


// ============================================================
// DB -> DISPLAY HEIGHT
// ============================================================

float dbToHeight(
  float db
)
{
  float level =
    (
      db -
      DB_MIN
    )
    /
    (
      DB_MAX -
      DB_MIN
    )
    *
    DISPLAY_HEIGHT_SCALE;


  if (
    level <
    0.0f
  )
  {
    level =
      0.0f;
  }


  if (
    level >
    1.0f
  )
  {
    level =
      1.0f;
  }


  return
    level *
    PANEL_RES_Y;
}


// ============================================================
// CALCULATE SPECTRUM
// ============================================================

// LOW PEAK MAPPING BEGIN
void mapLowSpectrumPeaks()
{
  const int binCount = LOW_FFT_SIZE / 2;
  const float binHz = LOW_SAMPLE_RATE / LOW_FFT_SIZE;
  const int firstBin = (int)floorf(MIN_FREQ / binHz);
  const int lastBin = (int)ceilf(CENTER_FREQ / binHz);
  static int16_t nextBand[LOW_FFT_SIZE / 2];
  static float nextPosition[LOW_FFT_SIZE / 2];
  bool previousUsed[LOW_FFT_SIZE / 2] = {};
  if (!lowPeakTrackingReady)
  {
    for (int k = 0; k < binCount; ++k) previousLowPeakBand[k] = -1;
    lowPeakTrackingReady = true;
  }
  for (int k = 0; k < binCount; ++k) nextBand[k] = -1;
  for (int band = 0; band < LOW_BANDS; ++band) lowPeakMagnitude[band] = 0.0f;

  float strongest = 0.0f;
  for (int k = firstBin; k <= lastBin; ++k)
    strongest = fmaxf(strongest, lowReal[k]);

  for (int k = firstBin; k <= lastBin; ++k)
  {
    const float magnitude = lowReal[k];
    // One maximum per lobe, including an exactly flat two-bin peak.
    if (magnitude <= 0.0f || magnitude < strongest * LOW_PEAK_RELATIVE_FLOOR
        || magnitude <= lowReal[k - 1] || magnitude < lowReal[k + 1]) continue;

    // Three-bin log-parabolic interpolation estimates frequency between bins.
    // Keep the measured peak amplitude so vertical calibration is unchanged.
    const float left = logf(fmaxf(lowReal[k - 1], 1.0e-20f));
    const float middle = logf(fmaxf(magnitude, 1.0e-20f));
    const float right = logf(fmaxf(lowReal[k + 1], 1.0e-20f));
    const float curvature = left - 2.0f * middle + right;
    const float offset = curvature < -1.0e-6f
      ? fmaxf(-0.5f, fminf(0.5f, 0.5f * (left - right) / curvature)) : 0.0f;
    const float position = k + offset;
    const float estimatedFrequency = position * binHz;
    // Allow interpolation error at the lower endpoint (e.g. a 40 Hz tone).
    if (estimatedFrequency < MIN_FREQ - 0.25f * binHz
        || estimatedFrequency >= CENTER_FREQ) continue;
    const float frequency = fmaxf(MIN_FREQ, estimatedFrequency);

    int band = 0;
    while (band < LOW_BANDS - 1 && frequency >= lowBandEdges[band + 1]) ++band;

    // Match only a nearby peak from the preceding frame; never merge current
    // maxima. A peak can move immediately, without waiting for more frames.
    int previous = -1;
    float distance = 1.0f;
    for (int j = k - 1; j <= k + 1; ++j)
    {
      if (j < 1 || j >= binCount || previousLowPeakBand[j] < 0 || previousUsed[j]) continue;
      const float candidateDistance = fabsf(position - previousLowPeakPosition[j]);
      if (candidateDistance < distance)
      {
        distance = candidateDistance;
        previous = j;
      }
    }
    if (previous >= 0)
    {
      previousUsed[previous] = true;
      const int oldBand = previousLowPeakBand[previous];
      const float margin = fminf(0.5f * binHz, LOW_PEAK_HYSTERESIS
        * (lowBandEdges[oldBand + 1] - lowBandEdges[oldBand]));
      if (frequency >= lowBandEdges[oldBand] - margin
          && frequency < lowBandEdges[oldBand + 1] + margin) band = oldBand;
    }
    nextBand[k] = band;
    nextPosition[k] = position;
    lowPeakMagnitude[band] = fmaxf(lowPeakMagnitude[band], magnitude);
  }

  // Assign targets only. The shared display smoothing below handles attack,
  // release and peak-marker decay for bass just as it does for higher bands.
  // A moving tone may briefly leave a fading bar in its previous column.
  memcpy(previousLowPeakBand, nextBand, sizeof(nextBand));
  memcpy(previousLowPeakPosition, nextPosition, sizeof(nextPosition));
}
// LOW PEAK MAPPING END

void calculateSpectrum()
{
  frameRmsDb =
    calculateFrameRmsDb();


  updateGlobalCompressor(
    frameRmsDb
  );


  // ==========================================================
  // HIGH FFT
  // ==========================================================

  for (
    int i = 0;
    i < HIGH_FFT_SIZE;
    i++
  )
  {
    highReal[i] =
      highWindow[i];


    highImag[i] =
      0.0f;
  }


  highFFT.dcRemoval();


  highFFT.windowing(
    FFTWindow::Hamming,
    FFTDirection::Forward
  );


  highFFT.compute(
    FFTDirection::Forward
  );


  highFFT.complexToMagnitude();


  // ==========================================================
  // LOW FFT
  // ==========================================================

  copyLowWindowToFFT();


  lowFFT.dcRemoval();


  lowFFT.windowing(
    FFTWindow::Hamming,
    FFTDirection::Forward
  );


  lowFFT.compute(
    FFTDirection::Forward
  );


  lowFFT.complexToMagnitude();

  mapLowSpectrumPeaks();


  const float highFFTScale =
    2.0f /
    HIGH_FFT_SIZE;


  const float lowFFTScale =
    2.0f /
    LOW_FFT_SIZE;


  // ==========================================================
  // RAW SPECTRUM
  // ==========================================================

  for (
    int band = 0;
    band < NUM_BANDS;
    band++
  )
  {
    if (
      band <
      LOW_BANDS
    )
    {
      const float lowAmplitude = lowPeakMagnitude[band] * lowFFTScale;


      rawBandDb[band] =
        amplitudeToDb(
          lowAmplitude
        )
        +
        LOW_BRANCH_GAIN_DB;


      uint16_t fastBin =
        fastAssistBin[band];


      float fastAmplitude =
        highReal[fastBin]
        *
        highFFTScale;


      fastBassDb[band] =
        amplitudeToDb(
          fastAmplitude
        );
    }

    else
    {
      float magnitude =
        0.0f;


      for (
        uint16_t bin =
          highBandStart[band];

        bin <=
          highBandEnd[band];

        bin++
      )
      {
        if (
          highReal[bin] >
          magnitude
        )
        {
          magnitude =
            highReal[bin];
        }
      }


      float amplitude =
        magnitude *
        highFFTScale;


      rawBandDb[band] =
        amplitudeToDb(
          amplitude
        );
    }
  }


  // ==========================================================
  // DISPLAY LEVELS
  // ==========================================================

  for (
    int band = 0;
    band < NUM_BANDS;
    band++
  )
  {
    float displayDb =
      rawBandDb[band]
      +
      globalGainDb
      +
      bandBoostDb[band];


    float target =
      dbToHeight(
        displayDb
      );


    if (
      band <
      LOW_BANDS
      &&
      lowPeakMagnitude[band] > 0.0f
      &&
      fastAssistEnabled[band]
    )
    {
      float fastDisplayDb =
        fastBassDb[band]
        +
        globalGainDb
        +
        bandBoostDb[band];


      float fastTarget =
        dbToHeight(
          fastDisplayDb
        );


      if (
        fastTarget >
          target
        &&
        fastTarget >
          bandLevel[band]
      )
      {
        target =
          target
          +
          (
            fastTarget -
            target
          )
          *
          FAST_ATTACK_ASSIST;
      }
    }


    if (
      target >
      bandLevel[band]
    )
    {
      bandLevel[band] +=
        (
          target -
          bandLevel[band]
        )
        *
        0.80f;
    }

    else
    {
      bandLevel[band] +=
        (
          target -
          bandLevel[band]
        )
        *
        0.15f;
    }


    if (
      bandLevel[band] >
      peakLevel[band]
    )
    {
      peakLevel[band] =
        bandLevel[band];
    }

    else
    {
      peakLevel[band] -=
        0.30f;


      if (
        peakLevel[band] <
        0.0f
      )
      {
        peakLevel[band] =
          0.0f;
      }
    }
  }
}


// ============================================================
// DRAW BAND PIXEL
// ============================================================

void drawBandPixel(
  int x,
  int y
)
{
  display->drawPixelRGB888(
    x,
    y,
    bandR[x],
    bandG[x],
    bandB[x]
  );
}


// ============================================================
// ERASE PIXEL
// ============================================================

void erasePixel(
  int x,
  int y
)
{
  display->drawPixelRGB888(
    x,
    y,
    0,
    0,
    0
  );
}


// ============================================================
// FLICKER-FREE SPECTRUM DRAW
// ============================================================

void drawSpectrum()
{
  for (
    int x = 0;
    x < NUM_BANDS;
    x++
  )
  {
    int newHeight =
      (int)(
        bandLevel[x] +
        0.5f
      );


    if (
      newHeight <
      0
    )
    {
      newHeight =
        0;
    }


    if (
      newHeight >
      PANEL_RES_Y
    )
    {
      newHeight =
        PANEL_RES_Y;
    }


    int newPeakY =
      -1;


    if (
      peakLevel[x] >=
      1.0f
    )
    {
      newPeakY =
        PANEL_RES_Y -
        (int)
        ceilf(
          peakLevel[x]
        );


      if (
        newPeakY <
        0
      )
      {
        newPeakY =
          0;
      }


      if (
        newPeakY >=
        PANEL_RES_Y
      )
      {
        newPeakY =
          PANEL_RES_Y -
          1;
      }
    }


    // --------------------------------------------------------
    // Bar grew
    // --------------------------------------------------------

    if (
      newHeight >
      oldHeight[x]
    )
    {
      for (
        int h =
          oldHeight[x];

        h <
          newHeight;

        h++
      )
      {
        int y =
          PANEL_RES_Y -
          1 -
          h;


        drawBandPixel(
          x,
          y
        );
      }
    }


    // --------------------------------------------------------
    // Bar shrank
    // --------------------------------------------------------

    else if (
      newHeight <
      oldHeight[x]
    )
    {
      for (
        int h =
          newHeight;

        h <
          oldHeight[x];

        h++
      )
      {
        int y =
          PANEL_RES_Y -
          1 -
          h;


        erasePixel(
          x,
          y
        );
      }
    }


    // --------------------------------------------------------
    // Restore old peak
    // --------------------------------------------------------

    if (
      oldPeakY[x] >=
        0
      &&
      oldPeakY[x] !=
        newPeakY
    )
    {
      int barTop =
        PANEL_RES_Y -
        newHeight;


      if (
        oldPeakY[x] >=
          barTop
        &&
        newHeight >
          0
      )
      {
        drawBandPixel(
          x,
          oldPeakY[x]
        );
      }

      else
      {
        erasePixel(
          x,
          oldPeakY[x]
        );
      }
    }


    // --------------------------------------------------------
    // Draw peak
    // --------------------------------------------------------

    if (
      newPeakY >=
      0
    )
    {
      display->drawPixelRGB888(
        x,
        newPeakY,
        255,
        255,
        255
      );
    }


    oldHeight[x] =
      newHeight;


    oldPeakY[x] =
      newPeakY;
  }
}


// ============================================================
// RESET SPECTRUM DISPLAY STATE
// ============================================================

void resetSpectrumDisplayState()
{
  for (
    int x = 0;
    x < NUM_BANDS;
    x++
  )
  {
    oldHeight[x] =
      0;


    oldPeakY[x] =
      -1;
  }
}


// ============================================================
// DRAW CLOCK + GHIMBAV TEMPERATURE
// ============================================================

// Compose text in RAM; pastel bubbles tint only the letters, leaving black behind.
bool clockTextMask[PANEL_RES_Y][PANEL_RES_X] = {};
uint16_t clockPixelColors[PANEL_RES_Y][PANEL_RES_X] = {};

struct ClockPastelBubble
{
  float x, y, driftX, driftY, radius;
  uint32_t startedMs, durationMs;
  uint8_t paletteIndex;
};

const int CLOCK_PASTEL_BUBBLE_COUNT = 5;
const uint32_t CLOCK_COLOR_FRAME_MS = 40;
ClockPastelBubble clockPastelBubbles[CLOCK_PASTEL_BUBBLE_COUNT] = {};
uint32_t lastClockColorFrameMs = 0;

// Use a built-in parameter type so Arduino's generated prototype needs no struct.
void restartClockPastelBubble(int index, uint32_t now, bool stagger)
{
  ClockPastelBubble &bubble = clockPastelBubbles[index];
  bubble.x = random(PANEL_RES_X);
  bubble.y = random(PANEL_RES_Y);
  bubble.driftX = random(-120, 121) * 0.1f;
  bubble.driftY = random(-60, 61) * 0.1f;
  bubble.radius = random(14, 29);
  bubble.durationMs = random(18000, 32001);
  // Stagger initial ages so color is already spread across the first frame.
  bubble.startedMs = now - (stagger ? random(bubble.durationMs) : 0);
  bubble.paletteIndex = random(6);
}

void clearClockMaskRows(int y, int height)
{
  for (int row = y; row < y + height && row < PANEL_RES_Y; ++row)
  {
    memset(clockTextMask[row], 0, sizeof(clockTextMask[row]));
  }
}

void setClockMaskPixel(int x, int y)
{
  if (x >= 0 && x < PANEL_RES_X && y >= 0 && y < PANEL_RES_Y)
  {
    clockTextMask[y][x] = true;
  }
}

void renderClockText(bool textChanged)
{
  const uint32_t now = millis();
  if (!textChanged && (uint32_t)(now - lastClockColorFrameMs) < CLOCK_COLOR_FRAME_MS)
    return;
  lastClockColorFrameMs = now;

  // Pronounced pastel tints: blush, peach, lemon, mint, sky, lilac.
  static const uint8_t tint[6][3] =
  {
    {0, 144, 76}, {0, 76, 152}, {16, 0, 152},
    {136, 0, 68}, {136, 60, 0}, {60, 128, 0}
  };
  float centersX[CLOCK_PASTEL_BUBBLE_COUNT];
  float centersY[CLOCK_PASTEL_BUBBLE_COUNT];
  float strengths[CLOCK_PASTEL_BUBBLE_COUNT];
  for (int i = 0; i < CLOCK_PASTEL_BUBBLE_COUNT; ++i)
  {
    ClockPastelBubble &bubble = clockPastelBubbles[i];
    if (bubble.durationMs == 0)
      restartClockPastelBubble(i, now, true);
    else if ((uint32_t)(now - bubble.startedMs) >= bubble.durationMs)
      restartClockPastelBubble(i, now, false);
    const float age = (float)(uint32_t)(now - bubble.startedMs) / bubble.durationMs;
    centersX[i] = bubble.x + bubble.driftX * (age - 0.5f);
    centersY[i] = bubble.y + bubble.driftY * (age - 0.5f);
    const float envelope = sinf(age * 3.14159265f);
    strengths[i] = envelope * envelope;
  }

  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      uint16_t color = 0;
      if (clockTextMask[y][x])
      {
        float reductions[3] = {};
        float totalWeight = 0.0f;
        for (int i = 0; i < CLOCK_PASTEL_BUBBLE_COUNT; ++i)
        {
          const ClockPastelBubble &bubble = clockPastelBubbles[i];
          const float dx = x - centersX[i];
          const float dy = y - centersY[i];
          const float falloff = 1.0f - (dx * dx + dy * dy) /
                                       (bubble.radius * bubble.radius);
          if (falloff <= 0.0f) continue;
          const float weight = 1.8f * falloff * falloff * strengths[i];
          totalWeight += weight;
          for (int channel = 0; channel < 3; ++channel)
            reductions[channel] += tint[bubble.paletteIndex][channel] * weight;
        }
        // Normalize overlaps to preserve pastel brightness without oversaturation.
        const float divisor = fmaxf(1.0f, totalWeight);
        color = display->color565(
          (uint8_t)(255.0f - reductions[0] / divisor + 0.5f),
          (uint8_t)(255.0f - reductions[1] / divisor + 0.5f),
          (uint8_t)(255.0f - reductions[2] / divisor + 0.5f));
      }
      if (color != clockPixelColors[y][x])
      {
        // Replace each pixel directly; only obsolete glyph pixels go black.
        display->fillRect(x, y, 1, 1, color);
        clockPixelColors[y][x] = color;
      }
    }
  }
}

// Original 5x7 clock glyphs, now drawn at their native size.
void drawClockText(int x, int y, const char *text)
{
  const int scale = 1;
  const int glyphAdvance = 6 * scale;
  const char characters[] = "0123456789:";
  static const uint8_t glyphs[][7] =
  {
    {14, 17, 19, 21, 25, 17, 14}, // 0
    {4, 12, 4, 4, 4, 4, 14}, // 1
    {14, 17, 1, 2, 4, 8, 31}, // 2
    {31, 2, 4, 2, 1, 17, 14}, // 3
    {2, 6, 10, 18, 31, 2, 2}, // 4
    {31, 16, 30, 1, 1, 17, 14}, // 5
    {6, 8, 16, 30, 17, 17, 14}, // 6
    {31, 1, 2, 4, 8, 8, 8}, // 7
    {14, 17, 17, 14, 17, 17, 14}, // 8
    {14, 17, 17, 15, 1, 2, 12}, // 9
    {0, 0, 4, 0, 4, 0, 0}, // :
  };

  for (; *text; ++text)
  {
    const char *glyph = strchr(characters, *text);
    if (!glyph)
    {
      x += glyphAdvance;


      continue;
    }


    const size_t index = glyph - characters;
    for (int row = 0; row < 7; ++row)
    {
      for (int column = 0; column < 5; ++column)
      {
        if (glyphs[index][row] & (1 << (4 - column)))
        {
          for (int pixelY = 0; pixelY < scale; ++pixelY)
          {
            for (int pixelX = 0; pixelX < scale; ++pixelX)
            {
              setClockMaskPixel(
                x + column * scale + pixelX,
                y + row * scale + pixelY
              );
            }
          }
        }
      }
    }


    // Keep the original tighter colon spacing at the smaller scale.
    const bool nextCharacterIsSeparator = *(text + 1) == ':';


    x +=
      glyphAdvance -
      ((*text == ':' || nextCharacterIsSeparator) ? scale : 0);
  }
}

// Compact 4x5 weather letters, matching the clock-screen preview.
int weatherGlyphWidth(char character)
{
  if (character == '.') return 1;
  if (character == ' ') return 2;
  if (character == '-' || character == '*' || character == 'I') return 3;
  return 4;
}

int weatherTextWidth(const char *text, int spacing = 1)
{
  int width = 0;
  for (; *text; ++text) width += weatherGlyphWidth(*text) + spacing;
  return width > 0 ? width - spacing : 0;
}

void drawWeatherText(int x, int y, const char *text, int spacing = 1)
{
  const char characters[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.* ";
  static const uint8_t glyphs[][5] =
  {
    {6,9,9,9,6}, {4,12,4,4,14}, // 0-1
    {14,1,6,8,15}, {14,1,6,1,14}, // 2-3
    {9,9,15,1,1}, {15,8,14,1,14}, // 4-5
    {7,8,14,9,6}, {15,1,2,4,4}, // 6-7
    {6,9,6,9,6}, {6,9,7,1,14}, // 8-9
    {6,9,15,9,9}, // A
    {14,9,14,9,14}, // B
    {7,8,8,8,7}, // C
    {14,9,9,9,14}, // D
    {15,8,14,8,15}, // E
    {15,8,14,8,8}, // F
    {7,8,11,9,7}, // G
    {9,9,15,9,9}, // H
    {7,2,2,2,7}, // I, 3 pixels wide
    {3,1,1,9,6}, // J
    {9,10,12,10,9}, // K
    {8,8,8,8,15}, // L
    {9,15,15,9,9}, // M
    {9,13,11,9,9}, // N
    {6,9,9,9,6}, // O
    {14,9,14,8,8}, // P
    {6,9,9,11,7}, // Q
    {14,9,14,10,9}, // R
    {7,8,6,1,14}, // S
    {15,6,6,6,6}, // T
    {9,9,9,9,6}, // U
    {9,9,9,6,6}, // V
    {9,9,15,15,9}, // W
    {9,9,6,9,9}, // X
    {9,9,6,6,6}, // Y
    {15,1,6,8,15}, // Z
    {0,0,7,0,0}, // minus, 3 pixels wide
    {0,0,0,0,1}, // decimal point, 1 pixel wide
    {2,5,2,0,0}, // degree, 3 pixels wide
    {0,0,0,0,0} // space, 2 pixels wide
  };
  for (; *text; ++text)
  {
    const int width = weatherGlyphWidth(*text);
    const char *glyph = strchr(characters, *text);
    if (glyph)
      for (int row = 0; row < 5; ++row)
        for (int column = 0; column < width; ++column)
          if (glyphs[glyph - characters][row] & (1 << (width - 1 - column)))
            setClockMaskPixel(x + column, y + row);
    x += width + spacing;
  }
}

void drawTemperatureRows(float currentC, bool currentValid,
                         float maximumC, bool maximumValid, bool showTomorrow)
{
  const char *labels[2] = {"NOW", showTomorrow ? "N MAX" : "MAX"};
  const float temperatures[2] = {currentC, maximumC};
  const bool valid[2] = {currentValid, maximumValid};
  char values[2][16];
  bool negative[2];
  for (int row = 0; row < 2; ++row)
  {
    if (valid[row] && isfinite(temperatures[row]))
    {
      snprintf(values[row], sizeof(values[row]), "%.1f*C", (double)temperatures[row]);
      // Normalize negative zero after rounding to the displayed precision.
      if (strcmp(values[row], "-0.0*C") == 0) strcpy(values[row], "0.0*C");
    }
    else
      strcpy(values[row], "--.-*C");
    negative[row] = valid[row] && isfinite(temperatures[row]) && values[row][0] == '-';
  }

  // Center the actual pair, keeping the original three-pixel gap before the
  // sign. Only an unsigned row gets an empty sign slot when the other is minus.
  const int gap = 3;
  int spacing = 1;
  int labelWidth = 0;
  int valueWidth = 0;
  int signWidth = 0;
  for (;;)
  {
    labelWidth = valueWidth = 0;
    signWidth = (negative[0] || negative[1]) ? weatherGlyphWidth('-') + spacing : 0;
    for (int row = 0; row < 2; ++row)
    {
      const int labelSize = weatherTextWidth(labels[row], spacing);
      const int valueSize = weatherTextWidth(values[row] + (negative[row] ? 1 : 0), spacing);
      if (labelSize > labelWidth) labelWidth = labelSize;
      if (valueSize > valueWidth) valueWidth = valueSize;
    }
    if (labelWidth + gap + signWidth + valueWidth <= PANEL_RES_X || spacing == 0) break;
    spacing = 0;
  }
  const int labelX = (PANEL_RES_X - labelWidth - gap - signWidth
    - valueWidth) / 2;
  const int signX = labelX + labelWidth + gap;
  const int digitX = signX + signWidth;
  for (int row = 0; row < 2; ++row)
  {
    const int y = 12 + row * 6;
    clearClockMaskRows(y, 5);
    drawWeatherText(labelX, y, labels[row], spacing);
    if (negative[row]) drawWeatherText(signX, y, "-", spacing);
    drawWeatherText(digitX, y, values[row] + (negative[row] ? 1 : 0), spacing);
  }
}

const char *weatherConditions(int code)
{
  switch (code)
  {
    case 0: case 1: return "SUNNY";
    case 2: return "PART CLOUD";
    case 3: return "CLOUDY";
    case 45: case 48: return "FOG";
    case 51: case 53: case 55: return "DRIZZLE";
    case 56: case 57: return "FRZ DRIZ";
    case 61: case 63: case 65: return "RAIN";
    case 66: case 67: return "FRZ RAIN";
    case 71: case 73: case 75: case 77: case 85: case 86: return "SNOW";
    case 80: case 81: case 82: return "SHOWERS";
    case 95: case 97: return "T-STORM";
    case 96: case 99: return "STORM HAIL";
    default: return "NO DATA";
  }
}

// Advance the calendar at noon so DST changes cannot skip or repeat a date.
void clockForecastDate(const struct tm &timeInfo, char *date, size_t capacity)
{
  struct tm forecastTime = timeInfo;
  if (timeInfo.tm_hour >= 18)
  {
    forecastTime.tm_mday++;
    forecastTime.tm_hour = 12;
    forecastTime.tm_isdst = -1;
    mktime(&forecastTime);
  }
  strftime(date, capacity, "%Y-%m-%d", &forecastTime);
}

// NOAA solar equations, with the standard 90.833-degree sunrise/sunset zenith.
// https://gml.noaa.gov/grad/solcalc/main.js
double ghimbavSunEventUtcMinutes(double julianDay, bool sunrise)
{
  const double radians = 0.017453292519943295;
  const double latitude = 45.6635 * radians;
  const double longitude = 25.5066;
  const double t = (julianDay - 2451545.0) / 36525.0;
  const double meanLongitude = fmod(280.46646 + t * (36000.76983 + t * 0.0003032),
                                    360.0) * radians;
  const double anomaly = (357.52911 + t * (35999.05029 - 0.0001537 * t)) * radians;
  const double eccentricity = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double center = sin(anomaly) * (1.914602 - t * (0.004817 + 0.000014 * t))
    + sin(2.0 * anomaly) * (0.019993 - 0.000101 * t) + sin(3.0 * anomaly) * 0.000289;
  const double omega = (125.04 - 1934.136 * t) * radians;
  const double apparentLongitude = meanLongitude
    + (center - 0.00569 - 0.00478 * sin(omega)) * radians;
  const double obliquity = (23.0 + (26.0 + (21.448
    - t * (46.8150 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0
    + 0.00256 * cos(omega)) * radians;
  const double declination = asin(sin(obliquity) * sin(apparentLongitude));
  const double y = pow(tan(obliquity / 2.0), 2.0);
  const double equationOfTime = 4.0 / radians * (y * sin(2.0 * meanLongitude)
    - 2.0 * eccentricity * sin(anomaly)
    + 4.0 * eccentricity * y * sin(anomaly) * cos(2.0 * meanLongitude)
    - 0.5 * y * y * sin(4.0 * meanLongitude)
    - 1.25 * eccentricity * eccentricity * sin(2.0 * anomaly));
  const double hourAngle = acos(cos(90.833 * radians)
    / (cos(latitude) * cos(declination)) - tan(latitude) * tan(declination)) / radians;
  return 720.0 - 4.0 * (longitude + (sunrise ? hourAngle : -hourAngle)) - equationOfTime;
}

bool isGhimbavNight(time_t utcNow, const struct tm &utcTime)
{
  static time_t cachedMidnight = (time_t)-1;
  static double sunriseMinutes = 0.0;
  static double sunsetMinutes = 0.0;
  const int seconds = utcTime.tm_hour * 3600 + utcTime.tm_min * 60 + utcTime.tm_sec;
  const time_t utcMidnight = utcNow - seconds;
  if (utcMidnight != cachedMidnight)
  {
    const double julianDay = (double)utcMidnight / 86400.0 + 2440587.5;
    // Refine each event using the sun's position at its estimated time.
    const double riseEstimate = ghimbavSunEventUtcMinutes(julianDay, true);
    const double setEstimate = ghimbavSunEventUtcMinutes(julianDay, false);
    sunriseMinutes = ghimbavSunEventUtcMinutes(julianDay + riseEstimate / 1440.0, true);
    sunsetMinutes = ghimbavSunEventUtcMinutes(julianDay + setEstimate / 1440.0, false);
    cachedMidnight = utcMidnight;
  }
  // Both events occur within the same UTC day at Ghimbav. UTC avoids DST shifts.
  const double minutes = seconds / 60.0;
  return minutes < sunriseMinutes || minutes >= sunsetMinutes;
}

// Both the clock and spectrum follow the same sunset/sunrise brightness schedule.
void updateScheduledBrightness(bool force = false)
{
  static uint32_t lastCheckMs = 0;
  static uint8_t scheduledBrightness = MATRIX_BRIGHTNESS_LIGHT;
  const uint32_t now = millis();
  if (force || (uint32_t)(now - lastCheckMs) >= 1000)
  {
    lastCheckMs = now;
    const time_t utcNow = time(nullptr);
    struct tm utcTime;
    // Calculate locally after NTP sync, even while weather/WiFi is unavailable.
    if (internetTimeValid && gmtime_r(&utcNow, &utcTime))
    {
      scheduledBrightness = isGhimbavNight(utcNow, utcTime)
        ? MATRIX_BRIGHTNESS_DARK : MATRIX_BRIGHTNESS_LIGHT;
    }
  }
  const uint8_t target = scheduledBrightness;
  if (target != matrixBrightness)
  {
    matrixBrightness = target;
    display->setBrightness8(matrixBrightness);
  }
}

void drawClock()
{
  if (!internetTimeValid) return;

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 10)) return;

  const bool showTomorrow = timeInfo.tm_hour >= 18;
  char forecastDate[11];
  clockForecastDate(timeInfo, forecastDate, sizeof(forecastDate));
  static char displayedForecastDate[11] = "";
  static bool displayedTomorrow = false;
  const bool forecastChanged = strcmp(forecastDate, displayedForecastDate) != 0 ||
                               showTomorrow != displayedTomorrow;

  WeatherReading reading;
  uint32_t currentWeatherVersion;
  bool valid;
  portENTER_CRITICAL(&weatherMux);
  reading = latestWeather;
  currentWeatherVersion = weatherVersion;
  valid = weatherValid;
  portEXIT_CRITICAL(&weatherMux);
  const bool timeChanged = timeInfo.tm_min != lastClockMinute;
  const bool outsideChanged = currentWeatherVersion != displayedWeatherVersion;
  const bool textChanged = timeChanged || outsideChanged || forecastChanged;

  if (timeChanged)
  {
    lastClockMinute = timeInfo.tm_min;
    clearClockMaskRows(0, 9);
    char timeString[6];
    strftime(timeString, sizeof(timeString), "%H:%M", &timeInfo);
    // HH:MM occupies 27x7 pixels with a steady colon, centered above the weather.
    drawClockText((PANEL_RES_X - 27) / 2, 2, timeString);
  }

  if (timeChanged || outsideChanged || forecastChanged)
  {
    displayedWeatherVersion = currentWeatherVersion;
    strcpy(displayedForecastDate, forecastDate);
    displayedTomorrow = showTomorrow;
    const DailyForecast *forecast = nullptr;
    if (valid)
      for (int day = 0; day < 2; ++day)
        if (strcmp(forecastDate, reading.forecasts[day].date) == 0)
          forecast = &reading.forecasts[day];
    drawTemperatureRows(reading.currentC, valid,
                        forecast ? forecast->maximumC : NAN, forecast != nullptr, showTomorrow);
    clearClockMaskRows(24, 5);
    char conditions[16];
    snprintf(conditions, sizeof(conditions), "%s%s", showTomorrow ? "N " : "",
             forecast ? weatherConditions(forecast->conditionCode) : "NO DATA");
    const int spacing = weatherTextWidth(conditions) > PANEL_RES_X ? 0 : 1;
    drawWeatherText((PANEL_RES_X - weatherTextWidth(conditions, spacing)) / 2,
                    24, conditions, spacing);
    // Refresh a missing target date; never show a different day's forecast.
    static char requestedDate[11] = "";
    if (!forecast && strcmp(forecastDate, requestedDate) != 0 && weatherTaskHandle)
    {
      strcpy(requestedDate, forecastDate);
      xTaskNotifyGive(weatherTaskHandle);
    }
  }

  renderClockText(textChanged);

  // Start the delay only after the clock has actually been rendered.
  if (!clockDisplayStarted)
  {
    clockDisplayedSinceMs = millis();
    clockDisplayStarted = true;
  }
}


// ============================================================
// ENTER CLOCK MODE
// ============================================================

void enterClockMode()
{
  if (
    clockMode
  )
  {
    return;
  }


  clockMode =
    true;
  updateScheduledBrightness(true);
  clockDisplayStarted = false;


  setAuxiliaryServicesPaused(
    false
  );


  lastClockMinute =
    -1;


  displayedWeatherVersion =
    0;


  display->clearScreen();
  memset(clockTextMask, 0, sizeof(clockTextMask));
  memset(clockPixelColors, 0, sizeof(clockPixelColors));
  memset(clockPastelBubbles, 0, sizeof(clockPastelBubbles));



  Serial.println(
    "CLOCK + WEATHER mode"
  );


  drawClock();
}


// ============================================================
// LEAVE CLOCK MODE
// ============================================================

void leaveClockMode()
{
  if (
    !clockMode
  )
  {
    return;
  }


  clockMode =
    false;
  updateScheduledBrightness(true);


  setAuxiliaryServicesPaused(
    true
  );


  display->clearScreen();


  resetSpectrumDisplayState();


  Serial.println(
    "Audio detected -> SPECTRUM mode"
  );
}


// ============================================================
// UPDATE DISPLAY MODE
// ============================================================

// Match drawSpectrum's rounding: 0.5 pixels becomes one visible bar pixel.
bool spectrumHasVisibleBars()
{
  for (int band = 0; band < NUM_BANDS; ++band)
  {
    if (bandLevel[band] >= 0.5f) return true;
  }
  return false;
}

void updateDisplayMode()
{
  if (
    !ENABLE_CLOCK_SCREEN
  )
  {
    lowSignalStartMs =
      0;


    audioReturnStartMs = 0;


    if (
      clockMode
    )
    {
      leaveClockMode();
    }


    return;
  }


  if (
    !internetTimeValid
  )
  {
    return;
  }


  uint32_t now =
    millis();


  const float signalThresholdDb = clockMode ? CLOCK_EXIT_DB : CLOCK_ENTER_DB;
  const bool signalDetected = isfinite(frameRmsDb)
    && frameRmsDb > signalThresholdDb && spectrumHasVisibleBars();


  // ==========================================================
  // SPECTRUM -> CLOCK
  // ==========================================================

  if (
    !clockMode
  )
  {
    audioReturnStartMs = 0;
    // Require sustained absence of drawable audio, ignoring blank noise frames.
    if (!signalDetected)
    {
      if (
        lowSignalStartMs ==
        0
      )
      {
        lowSignalStartMs =
          now;
      }


      if (
        (uint32_t)(now - lowSignalStartMs) >=
        CLOCK_ENTER_DELAY_MS
      )
      {
        enterClockMode();


        lowSignalStartMs =
          0;
      }
    }

    else
    {
      lowSignalStartMs =
        0;
    }
  }


  // ==========================================================
  // CLOCK -> SPECTRUM
  // ==========================================================

  else
  {
    lowSignalStartMs =
      0;


    if (signalDetected)
    {
      if (audioReturnStartMs == 0) audioReturnStartMs = now;
      if ((uint32_t)(now - audioReturnStartMs) >= CLOCK_EXIT_CONFIRM_MS)
      {
        leaveClockMode();
        audioReturnStartMs = 0;
      }
    }
    else
    {
      audioReturnStartMs = 0;
    }
  }
}


// ============================================================
// DISPLAY SETUP
// ============================================================

void setupDisplay()
{
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X,
    PANEL_RES_Y,
    PANEL_CHAIN
  );


  mxconfig.gpio.r1 =
    R1_PIN;

  mxconfig.gpio.g1 =
    G1_PIN;

  mxconfig.gpio.b1 =
    B1_PIN;


  mxconfig.gpio.r2 =
    R2_PIN;

  mxconfig.gpio.g2 =
    G2_PIN;

  mxconfig.gpio.b2 =
    B2_PIN;


  mxconfig.gpio.a =
    A_PIN;

  mxconfig.gpio.b =
    B_PIN;

  mxconfig.gpio.c =
    C_PIN;

  mxconfig.gpio.d =
    D_PIN;

  mxconfig.gpio.e =
    E_PIN;


  mxconfig.gpio.clk =
    CLK_PIN;

  mxconfig.gpio.lat =
    LAT_PIN;

  mxconfig.gpio.oe =
    OE_PIN;


  // ==========================================================
  // KNOWN-GOOD FLICKER-FREE CONFIG
  // ==========================================================

  mxconfig.double_buff =
    false;


  mxconfig.i2sspeed =
    HUB75_I2S_CFG::HZ_8M;


  mxconfig.setPixelColorDepthBits(
    8
  );


  mxconfig.min_refresh_rate =
    60;


  // With the SN74HCT245 in the signal path, the opposite clock edge is
  // required. Otherwise column 0 wraps onto the right edge of the panel.
  mxconfig.clkphase =
    false;


  display =
    new MatrixPanel_I2S_DMA(
      mxconfig
    );


  if (
    !display->begin()
  )
  {
    Serial.println(
      "HUB75 allocation failed"
    );


    while (true)
    {
      delay(
        1000
      );
    }
  }


  display->setBrightness8(
    matrixBrightness
  );


  display->clearScreen();


  Serial.print(
    "Matrix refresh: "
  );


  Serial.print(
    display->
    calculated_refresh_rate
  );


  Serial.println(
    " Hz"
  );

  // Matrix GPIOs and DMA are ready, with a cleared frame and brightness set.
  // This is never reached if display->begin() fails.
  ESP_ERROR_CHECK(gpio_set_level(BUFFER_OE_PIN, 0));
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ESP32-S3-DevKitC-1 onboard addressable RGB LED is on GPIO48.
  rgbLedWrite(48, 0, 0, 0);

  // The external pull-up disables the buffers during reset. Preload HIGH
  // before enabling the output to avoid an initial LOW pulse on /OE.
  ESP_ERROR_CHECK(gpio_set_level(BUFFER_OE_PIN, 1));
  ESP_ERROR_CHECK(gpio_set_direction(BUFFER_OE_PIN, GPIO_MODE_OUTPUT));

  Serial.begin(
    115200
  );


  delay(
    500
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    "Spectrum + Clock + Ghimbav Temperature"
  );

  Serial.println(
    "========================================"
  );


  // ----------------------------------------------------------
  // Matrix
  // ----------------------------------------------------------

  setupDisplay();


  // ----------------------------------------------------------
  // WiFi / NTP / weather
  //
  // Done before starting ADC DMA.
  // ----------------------------------------------------------

  setupInternetServices();


  if (ENABLE_CLOCK_SCREEN && internetTimeValid)
  {
    startupAudioDelayActive = true;
    enterClockMode();
  }


  // ----------------------------------------------------------
  // Initialize spectrum state
  // ----------------------------------------------------------

  for (
    int i = 0;
    i < NUM_BANDS;
    i++
  )
  {
    bandLevel[i] =
      0.0f;


    peakLevel[i] =
      0.0f;


    oldHeight[i] =
      0;


    oldPeakY[i] =
      -1;


    rawBandDb[i] =
      -120.0f;
  }


  for (
    int i = 0;
    i < LOW_FFT_SIZE;
    i++
  )
  {
    lowRing[i] =
      0.0f;
  }


  for (
    int i = 0;
    i < HIGH_FFT_SIZE;
    i++
  )
  {
    highWindow[i] =
      0.0f;
  }


  setupLowPassFIR();


  setupInputHighPass();


  prepareBands();


  setupADC_DMA();


  primeAudioWindows();


  lastDynamicsTimeMs =
    millis();


  Serial.println();

  Serial.println(
    "Spectrum analyzer running"
  );


  Serial.println(
    "Bass resolution : 7.8125 Hz/bin"
  );


  Serial.println(
    "Main resolution : 39.06 Hz/bin"
  );


  Serial.print(
    "Internet clock  : "
  );


  Serial.println(
    internetTimeValid
      ? "YES"
      : "NO"
  );


  Serial.print(
    "Weather         : "
  );


  if (
    weatherValid
  )
  {
    Serial.print(
      outsideTemperatureC,
      1
    );


    Serial.println(
      " C"
    );
  }

  else
  {
    Serial.println(
      "not available"
    );
  }


  Serial.print(
    "Clock enter     : "
  );


  Serial.print(
    CLOCK_ENTER_DB,
    1
  );


  Serial.print(
    " dB for "
  );


  Serial.print(
    CLOCK_ENTER_DELAY_MS
  );


  Serial.println(
    " ms"
  );


  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  updateScheduledBrightness();

  // Keep the startup clock visible before evaluating the GPIO1 audio signal.
  if (
    startupAudioDelayActive
    &&
    clockDisplayStarted
    &&
    (uint32_t)(millis() - clockDisplayedSinceMs) >= STARTUP_AUDIO_DELAY_MS
  )
  {
    startupAudioDelayActive = false;
    lastDynamicsTimeMs = millis();
  }


  if (!startupAudioDelayActive)
  {
    calculateSpectrum();
    updateDisplayMode();
  }


  // ==========================================================
  // DISPLAY
  // ==========================================================

  if (
    clockMode
  )
  {
    drawClock();
  }

  else
  {
    drawSpectrum();
  }


  // ==========================================================
  // STATUS
  // ==========================================================

  static uint32_t frames =
    0;


  static uint32_t lastTime =
    millis();


  frames++;


  if (
    millis() -
    lastTime >=
    1000
  )
  {
    Serial.print(
      "FPS "
    );


    Serial.print(
      frames
    );


    Serial.print(
      " | RMS "
    );


    Serial.print(
      frameRmsDb,
      1
    );


    Serial.print(
      " | AVG "
    );


    Serial.print(
      programLevelDb,
      1
    );


    Serial.print(
      " | gain "
    );


    if (
      globalGainDb >=
      0.0f
    )
    {
      Serial.print(
        "+"
      );
    }


    Serial.print(
      globalGainDb,
      1
    );


    Serial.print(
      " dB | temp "
    );


    if (
      weatherValid
    )
    {
      Serial.print(
        outsideTemperatureC,
        1
      );


      Serial.print(
        " C"
      );
    }

    else
    {
      Serial.print(
        "N/A"
      );
    }


    Serial.print(
      " | "
    );


    Serial.println(
      clockMode
        ? "CLOCK"
        : "SPECTRUM"
    );


    frames =
      0;


    lastTime +=
      1000;
  }


  // ==========================================================
  // NEXT ADC DMA BLOCK
  // ==========================================================

  if (
    !advanceAudioWindows()
  )
  {
    Serial.println(
      "ADC DMA read failed"
    );


    delay(
      100
    );
  }
}
