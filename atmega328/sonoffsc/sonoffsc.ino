/*

  Sonoff SC
  Copyright (C) 2017 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


//#include <Ticker.h>

#include "at_messages.h"
#include <SerialLink.h>
#include "taskDHT.h"
#include "taskLight.h"
#include "taskDust.h"
#include "taskAudio.h"
#include "taskRGB.hh"
#include "taskMove.hh"

#define SERIAL_BAUDRATE         1000000

#define LDR_PIN                 A3

#define SHARP_READ_PIN          A1
#define SHARP_LED_PIN           9

#define MICROPHONE_PIN          A2

#define DHT_PIN                 6

#define RGB_PIN                11
#define RGB_COUNT              18

#define MOVE_PIN                7

#if 0
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

#define FAN_PIN                 7
#define FAN_OFF_DELAY           0

#define RGB_TIMEOUT             0

#define ADC_COUNTS              1024

#define MW_PIN                  13

//#define NOISE_READING_DELAY     100
#define NOISE_READING_WINDOW    20
#define NOISE_BUFFER_SIZE       20

#define CLAP_DEBOUNCE_DELAY     150
#define CLAP_TIMEOUT_DELAY      1000
#define CLAP_SENSIBILITY        80
#define CLAP_COUNT_TRIGGER      4
#define CLAP_BUFFER_SIZE        7
#define CLAP_TOLERANCE          1.50

#define MAX_SERIAL_BUFFER       20

#define DEFAULT_EVERY           1
#define DEFAULT_PUSH            1
#define DEFAULT_CLAP            0
#define DEFAULT_THRESHOLD       0

#define NULL_VALUE              -999

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

bool rgbRunning = false;
unsigned long rgbTimeout = RGB_TIMEOUT;
unsigned long rgbStart = 0;

int clapTimings[CLAP_BUFFER_SIZE];
byte clapPointer = 0;


// If push == false the slave waits for the master to request for values
// If push == true the slaves sends messages
//
// If every == 0 values are retrieved upon request
// If every > 0 values are retrieved every <every> seconds and cached/sent
//
// If push == true and every == 0 messages are never sent.

bool push = DEFAULT_PUSH;
bool clap = DEFAULT_CLAP;
unsigned long every = 1000L * DEFAULT_EVERY;
unsigned int threshold = DEFAULT_THRESHOLD;
unsigned long fanoff = FAN_OFF_DELAY;


float temperature = NULL_VALUE;
int humidity = NULL_VALUE;
float dust = NULL_VALUE;
int light = NULL_VALUE;
int noise = NULL_VALUE;
bool movement;

//unsigned int noise_count = 0;
//unsigned long noise_sum = 0;
//unsigned int noise_peak = 0;
//unsigned int noise_min = 1024;
//unsigned int noise_max = 0;

unsigned int noise_buffer[NOISE_BUFFER_SIZE] = {0};
unsigned int noise_buffer_pointer = 0;
unsigned int noise_buffer_sum = 0;


// -----------------------------------------------------------------------------
// FAN
// -----------------------------------------------------------------------------

void fanStatus(bool status) {
  digitalWrite(FAN_PIN, status ? HIGH : LOW);
}

bool fanStatus() {
  return digitalRead(FAN_PIN) == HIGH;
}

// -----------------------------------------------------------------------------
// RGB LED
// -----------------------------------------------------------------------------

void rgbLoop() {

  if (rgbRunning && (rgbTimeout > 0)) {
    if (millis() - rgbStart > rgbTimeout) {
      ws2812fx.setMode(FX_MODE_FADE);
      rgbRunning = false;
    }
  }

  ws2812fx.service();

}

void rgbOff() {
  ws2812fx.setColor(0);
  ws2812fx.setMode(FX_MODE_STATIC);
}

void rgbEffect(unsigned int effect) {
  ws2812fx.setMode(effect);
  rgbStart = millis();
  rgbRunning = true;
}

void rgbColor(unsigned long color) {
  ws2812fx.setColor(color);
  rgbStart = millis();
  rgbRunning = true;
}

// -----------------------------------------------------------------------------
// SENSORS
// -----------------------------------------------------------------------------


float getTemperature() {
  loadTempAndHum();
  return temperature;
}

int getHumidity() {
  loadTempAndHum();
  return humidity;
}

int getNoise() {

  int value = 0;

  //if (noise_count > 0) {

  value = noise_buffer_sum / NOISE_BUFFER_SIZE;

  //Serial.print("CNT : "); Serial.println(noise_count);
  //Serial.print("SUM : "); Serial.println(noise_sum / noise_count);
  //Serial.print("PEAK: "); Serial.println(noise_peak / noise_count);
  //Serial.print("MAX : "); Serial.println(noise_max);
  //Serial.print("MIN : "); Serial.println(noise_min);
  //Serial.print("VAL : "); Serial.println(value);

  //noise_count = 0;
  //noise_sum = 0;
  //noise_peak = 0;
  //noise_min = ADC_COUNTS;
  //noise_max = 0;

  //}

  return value;

}

// -----------------------------------------------------------------------------
// MICROWAVE
// -----------------------------------------------------------------------------

bool getMovement() {
  return digitalRead(MW_PIN) == HIGH;
}

void moveLoop(bool force = false) {
  bool value = getMovement();
  if (force || (movement != value)) {
    link.send_P(at_move, value ? 1 : 0, false);
  }
  movement = value;
}

// -----------------------------------------------------------------------------
// MIC
// -----------------------------------------------------------------------------

void clapDecode() {

  // at least 2 claps
  if (clapPointer > 0) {

    byte code = 2;
    if (clapPointer > 1) {
      int length = clapTimings[0] * CLAP_TOLERANCE;
      for(byte i=1; i<clapPointer; i++) {
        code <<= 1;
        if (clapTimings[i] > length) code += 1;
      }
    }

    link.send_P(at_code, code);

  }

  // reset
  clapPointer = 0;

}

void clapRecord(int value) {

  static bool reading = false;
  static unsigned long last_clap;
  static int counts = 0;
  unsigned long current = millis();
  unsigned long span = current - last_clap;

  if (value > CLAP_SENSIBILITY) {
    ++counts;
  } else {
    counts = 0;
  }

  if (counts == CLAP_COUNT_TRIGGER) {

    //Serial.print("Value: "); Serial.println(value);

    // Is it the first clap?
    if (!reading) {

      last_clap = current;
      reading = true;

      // or not
    } else {

      //Serial.print("Span : "); Serial.println(span);

      // timed out
      if (span > CLAP_TIMEOUT_DELAY) {

        clapDecode();

        // reset
        reading = false;

      } else if (span < CLAP_DEBOUNCE_DELAY) {

        // do nothing

        // new clap!
      } else if (clapPointer < CLAP_BUFFER_SIZE) {
        clapTimings[clapPointer] = span;
        last_clap = current;
        clapPointer++;

        // buffer overrun
      } else {
        clapPointer = 0;
        reading = false;
      }

    }

    // check if we have to process it
  } else if (reading) {

    if (span > CLAP_TIMEOUT_DELAY) {

      clapDecode();

      // back to idle
      reading = false;

    }

  }

}

void noiseLoop() {

  static unsigned long last_reading = 0;
  static unsigned int triggered = 0;

  unsigned int sample;
  //unsigned int count = 0;
  //unsigned long sum;
  unsigned int min = ADC_COUNTS;
  unsigned int max = 0;

  // Check MIC every NOISE_READING_DELAY
  // if (millis() - last_reading < NOISE_READING_DELAY) return;
  last_reading = millis();

  while (millis() - last_reading < NOISE_READING_WINDOW) {
    sample = analogRead(MICROPHONE_PIN);
    //++count;
    //sum += sample;
    if (sample < min) min = sample;
    if (sample > max) max = sample;
  }

  //++noise_count;
  //unsigned int average = 100 * (sum / count) / ADC_COUNTS;
  //noise_sum += average;

  unsigned int peak = map(max - min, 0, ADC_COUNTS, 0, 100);
  //Serial.println(peak);
  if (clap) clapRecord(peak);

  noise_buffer_sum = noise_buffer_sum + peak - noise_buffer[noise_buffer_pointer];
  noise_buffer[noise_buffer_pointer] = peak;
  noise_buffer_pointer = (noise_buffer_pointer + 1) % NOISE_BUFFER_SIZE;

  //noise_peak += peak;
  //if (max > noise_max) noise_max = max;
  //if (min < noise_min) noise_min = min;

  if (threshold > 0) {
    unsigned int value = noise_buffer_sum / NOISE_BUFFER_SIZE;
    if (value > threshold) {
      if (value > triggered) {
        link.send_P(at_noise, value);
        triggered = value;
      }
    } else if (triggered > 0) {
      link.send_P(at_noise, value);
      triggered = 0;
    }
  }

}

#endif

// -----------------------------------------------------------------------------
// COMMUNICATION
// -----------------------------------------------------------------------------

// How to respond to AT+...=? requests
bool olinkGet(char * key) {
#if 0
  if (strcmp_P(key, at_push) == 0) {
    link.send(key, push ? 1 : 0, false);
    return true;
  }

  if (strcmp_P(key, at_fan) == 0) {
    link.send(key, fanStatus() ? 1 : 0, false);
    return true;
  }

  if (strcmp_P(key, at_fanoff) == 0) {
    link.send(key, fanoff, false);
    return true;
  }

  if (strcmp_P(key, at_clap) == 0) {
    link.send(key, clap ? 1 : 0, false);
    return true;
  }

  if (strcmp_P(key, at_thld) == 0) {
    link.send(key, threshold, false);
    return true;
  }

  if (strcmp_P(key, at_every) == 0) {
    link.send(key, every / 1000, false);
    return true;
  }

  if (strcmp_P(key, at_temp) == 0) {
    if (every == 0) temperature = getTemperature();
    if (temperature == NULL_VALUE) return false;
    link.send(key, 10 * temperature, false);
    return true;
  }

  if (strcmp_P(key, at_hum) == 0) {
    if (every == 0) humidity = getHumidity();
    if (humidity == NULL_VALUE) return false;
    link.send(key, humidity, false);
    return true;
  }

  if (strcmp_P(key, at_noise) == 0) {
    if (every == 0) noise = getNoise();
    if (noise == NULL_VALUE) return false;
    link.send(key, noise, false);
    return true;
  }

  if (strcmp_P(key, at_dust) == 0) {
    if (every == 0) {
      getDustDefer(true);
    } else {
      if (dust == NULL_VALUE) return false;
      link.send(key, dust, false);
    }
    return true;
  }

  if (strcmp_P(key, at_light) == 0) {
    if (every == 0) light = getLight();
    if (light == NULL_VALUE) return false;
    link.send(key, light, false);
    return true;
  }

  if (strcmp_P(key, at_move) == 0) {
    link.send(key, getMovement() ? 1 : 0, false);
    return true;
  }

  if (strcmp_P(key, at_timeout) == 0) {
    link.send(key, rgbTimeout, false);
    return true;
  }

  if (strcmp_P(key, at_effect) == 0) {
    link.send(key, ws2812fx.getMode(), false);
    return true;
  }

  if (strcmp_P(key, at_color) == 0) {
    link.send(key, ws2812fx.getColor(), false);
    return true;
  }

  if (strcmp_P(key, at_speed) == 0) {
    link.send(key, ws2812fx.getSpeed(), false);
    return true;
  }

  if (strcmp_P(key, at_bright) == 0) {
    link.send(key, ws2812fx.getBrightness(), false);
    return true;
  }
#endif
  return false;

}

// Functions for responding to AT+...=<long> commands that set values and functions
bool olinkSet(char * key, long value) {

  Serial.print("Got key ");
  Serial.print(key);
  Serial.print(" value ");
  Serial.println(value);

#if 0
  if (strcmp_P(key, at_push) == 0) {
    if (0 <= value && value <= 1) {
      push = value == 1;
      return true;
    }
  }

  if (strcmp_P(key, at_clap) == 0) {
    if (0 <= value && value <= 1) {
      clap = value == 1;
      return true;
    }
  }

  if (strcmp_P(key, at_every) == 0) {
    if (5 <= value && value <= 300) {
      every = 1000L * value;
      return true;
    }
  }

  if (strcmp_P(key, at_thld) == 0) {
    if (0 <= value && value <= 100) {
      threshold = value;
      return true;
    }
  }

  if (strcmp_P(key, at_fan) == 0) {
    if (0 <= value && value <= 1) {
      fanStatus(value == 1);
      return true;
    }
  }

  if (strcmp_P(key, at_fanoff) == 0) {
    fanoff = value;
    return true;
  }

  if (strcmp_P(key, at_timeout) == 0) {
    if (0 <= value) {
      rgbTimeout = value;
      return true;
    }
  }

  if (strcmp_P(key, at_color) == 0) {
    if (0 <= value && value <= 0xFFFFFF) {
      rgbColor(value);
      return true;
    }
  }

  if (strcmp_P(key, at_effect) == 0) {
    if (0 <= value && value < MODE_COUNT) {
      rgbEffect(value);
      return true;
    }
  }

  if (strcmp_P(key, at_speed) == 0) {
    if (SPEED_MIN <= value && value <= SPEED_MAX) {
      ws2812fx.setSpeed(value);
      return true;
    }
  }

  if (strcmp_P(key, at_bright) == 0) {
    if (BRIGHTNESS_MIN <= value && value <= BRIGHTNESS_MAX) {
      ws2812fx.setBrightness(value);
      return true;
    }
  }
#endif
  return false;

}

//Setup callbacks when AT commands are recieved
void linkSetup() {
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------

SerialLink link(Serial);

TaskDHT tdht(link, DHT_PIN);
TaskLight tlight(link, LDR_PIN);
TaskDust tdust(link, SHARP_READ_PIN, SHARP_LED_PIN);
TaskAudio taudio(link, MICROPHONE_PIN);
TaskRGB trgb(link, RGB_PIN, RGB_COUNT);
TaskMove tmove(link, MOVE_PIN);

volatile bool goON = false;

bool linkWaitHello(char * key, long value) {
  Serial.println(key);

    if(strcmp_P(key, at_hello) == 0){
      goON = true;
    }

}


void setup() {
  // Setup Serial port
  Serial.begin(SERIAL_BAUDRATE);


#if 0

  // Setup physical pins on the ATMega328
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MW_PIN, INPUT);

  rgbStart = millis();
  rgbRunning = true;

#endif

  link.onSet(linkWaitHello);
  while (!goON) {
    link.handle();
  }
  link.send_P(at_hello, 1);

  trgb.start();
}

extern volatile int adcReady;


void loop() {
  static int rotate = 0;
  bool ready = true;

#if 1
#if 1
  ready = taudio.loop();
  if(!ready) return;
#endif
  switch (rotate++) {
    case 0:
      tlight.loop();
      break;
    case 1:
      tdust.loop();
      break;
    case 2:
      tdht.loop();
      break;
    case 3:
      link.handle();
      break;
    case 4:
      trgb.loop();
      break;
    case 5:
      tmove.loop();
      break;
    default:
      rotate = 0;
      break;
  }
#else
  tdust.loop();
  tlight.loop();
#endif
}
