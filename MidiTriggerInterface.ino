/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 rppicomidi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/**
 * This demo program is designed to test the USB MIDI Host driver for a single USB
 * MIDI device connected to the USB Host port. It sends to the USB MIDI device the
 * sequence of half-steps from B-flat to D whose note numbers correspond to the
 * transport button LEDs on a Mackie Control compatible control surface. It also
 * prints to a UART serial port console the messages received from the USB MIDI device.
 *
 * This program works with a single USB MIDI device connected via a USB hub, but it
 * does not handle multiple USB MIDI devices connected at the same time.
 */

static bool printEnabled = true;

#include "pico/stdlib.h"

#if defined(USE_TINYUSB_HOST) || !defined(USE_TINYUSB)
#error "Please use the Menu to select Tools->USB Stack: Adafruit TinyUSB"
#endif
#include "pio_usb.h"
#define HOST_PIN_DP 16  // Pin used as D+ for host, D- = D+ + 1
#include "EZ_USB_MIDI_HOST.h"
// USB Host object
Adafruit_USBH_Host USBHost;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault)

static uint8_t midiDevAddr = 0;

static bool core0_booting = true;
static bool core1_booting = true;

#include <NeoPixelConnect.h>
#define NUM_NEOPIXEL 1
NeoPixelConnect p(PIN_NEOPIXEL, NUM_NEOPIXEL, pio0, 1);

#include <MIDI.h>
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDIusb);

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1015 ads;
constexpr int READY_PIN = 13;
volatile bool new_data = false;
void NewDataReadyISR() {
  new_data = true;
}

#include <Wire.h>
#define SDA_PIN 26  // Beispiel-Pin
#define SCL_PIN 27  // Beispiel-Pin

#define ALARM_MS 50
int64_t alarm_callback(alarm_id_t id, __unused void* user_data) {
  p.neoPixelFill(0, 32, 0, true);
  return 0;
}

/* MIDI IN MESSAGE REPORTING */
static void onNoteOn(Channel channel, byte note, byte velocity) {
  if (velocity == 0) {
    onNoteOff(channel, note, velocity);
    return;
  }

  MIDIusb.sendNoteOn(note, velocity, channel);

  p.neoPixelFill(255, 0, 0, true);
  add_alarm_in_ms(ALARM_MS, alarm_callback, NULL, false);

  if (printEnabled) {
    Serial.printf("C%u: Note on#%u v=%u\r\n", channel, note, velocity);
  }
}

static void onNoteOff(Channel channel, byte note, byte velocity) {
  MIDIusb.sendNoteOff(note, velocity, channel);

  if (printEnabled) {
    Serial.printf("C%u: Note off#%u v=%u\r\n", channel, note, velocity);
  }
}

static void onMidiError(int8_t errCode) {
  if (printEnabled) {
    Serial.printf("MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse" : "",
                  (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
                  (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx" : "");
  }
}

static void onPolyphonicAftertouch(Channel channel, byte note, byte amount) {}
static void onControlChange(Channel channel, byte controller, byte value) {}
static void onProgramChange(Channel channel, byte program) {}
static void onAftertouch(Channel channel, byte value) {}
static void onPitchBend(Channel channel, int value) {}
static void onSysEx(byte* array, unsigned size) {}
static void onSMPTEqf(byte data) {}
static void onSongPosition(unsigned beats) {}
static void onSongSelect(byte songnumber) {}
static void onTuneRequest() {}
static void onMidiClock() {}
static void onMidiStart() {}
static void onMidiContinue() {}
static void onMidiStop() {}
static void onActiveSense() {}
static void onSystemReset() {}
static void onMidiTick() {}
static void onMidiInWriteFail(uint8_t devAddr, uint8_t cable, bool fifoOverflow) {}

static void registerMidiInCallbacks() {
  auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midiDevAddr, 0);
  if (intf == nullptr)
    return;
  intf->setHandleNoteOff(onNoteOff);                      // 0x80
  intf->setHandleNoteOn(onNoteOn);                        // 0x90
  intf->setHandleAfterTouchPoly(onPolyphonicAftertouch);  // 0xA0
  intf->setHandleControlChange(onControlChange);          // 0xB0
  intf->setHandleProgramChange(onProgramChange);          // 0xC0
  intf->setHandleAfterTouchChannel(onAftertouch);         // 0xD0
  intf->setHandlePitchBend(onPitchBend);                  // 0xE0
  intf->setHandleSystemExclusive(onSysEx);                // 0xF0, 0xF7
  intf->setHandleTimeCodeQuarterFrame(onSMPTEqf);         // 0xF1
  intf->setHandleSongPosition(onSongPosition);            // 0xF2
  intf->setHandleSongSelect(onSongSelect);                // 0xF3
  intf->setHandleTuneRequest(onTuneRequest);              // 0xF6
  intf->setHandleClock(onMidiClock);                      // 0xF8
  // 0xF9 as 10ms Tick is not MIDI 1.0 standard but implemented in the Arduino MIDI Library
  intf->setHandleTick(onMidiTick);              // 0xF9
  intf->setHandleStart(onMidiStart);            // 0xFA
  intf->setHandleContinue(onMidiContinue);      // 0xFB
  intf->setHandleStop(onMidiStop);              // 0xFC
  intf->setHandleActiveSensing(onActiveSense);  // 0xFE
  intf->setHandleSystemReset(onSystemReset);    // 0xFF
  intf->setHandleError(onMidiError);

  auto dev = usbhMIDI.getDevFromDevAddr(midiDevAddr);
  if (dev == nullptr)
    return;
  dev->setOnMidiInWriteFail(onMidiInWriteFail);
}

/* CONNECTION MANAGEMENT */
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables) {
  p.neoPixelFill(0, 32, 0, true);

  if (printEnabled) {
    Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
  }

  midiDevAddr = devAddr;

  registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr) {
  p.neoPixelFill(0, 0, 128, true);

  midiDevAddr = 0;

  if (printEnabled) {
    Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
  }
}

// core1's setup
void setup1() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);

  // Sets pin PIN_5V_EN (Pin 18) to HIGH to enable USB power
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);

  Serial.begin(115200);
  while (!Serial)
    ;  // wait for native usb
  if (printEnabled) {
    Serial.println("EZ USB MIDI HOST PIO Example for Arduino\r\n");
    Serial.println("Core1 setup to run TinyUSB host with pio-usb\r\n");
  }

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    delay(2000);  // wait for native usb
    if (printEnabled) {
      Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
      Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    }
    while (1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* Need to swap PIOs so PIO code from CYW43 PIO SPI driver will fit */
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  USBHost.configure_pio_usb(1, &pio_cfg);
  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing work done in core1 to free up core0 for other work
  usbhMIDI.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);

  core1_booting = false;
  while (core0_booting)
    ;
}

// core1's loop
void loop1() {
  USBHost.task();
  usbhMIDI.readAll();
}

void setup() {
  MIDIusb.begin();
  MIDIusb.turnThruOff();  // turn off echo

  Serial.begin(115200);
  while (!Serial)
    ;  // wait for native usb


  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_TWOTHIRDS);

  // ads.setDataRate(RATE_ADS1015_128SPS);
  ads.setDataRate(RATE_ADS1015_250SPS);
  // ads.setDataRate(RATE_ADS1015_490SPS);
  // ads.setDataRate(RATE_ADS1015_920SPS);
  // ads.setDataRate(RATE_ADS1015_1600SPS);
  // ads.setDataRate(RATE_ADS1015_2400SPS);
  // ads.setDataRate(RATE_ADS1015_3300SPS);


  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  if (ads.begin(0x48, &Wire)) {
    if (printEnabled) {
      Serial.println("ADC initialization SUCCESSFUL\r\n");
    }
  } else {
    if (printEnabled) {
      Serial.println("ADC initialization FAILED\r\n");
    }
  }

  pinMode(READY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);

  core0_booting = false;
  while (core1_booting)
    ;
}

volatile int sample_n1 = 0;
volatile int sample_n2 = 0;
volatile int sample_n3 = 0;

void loop() {
  while (!new_data)
    ;

  int16_t sample = abs(ads.getLastConversionResults());
  new_data = false;
  //Serial.println(sample);

  sample_n3 = sample_n2;
  sample_n2 = sample_n1;
  sample_n1 = sample;

  if (sample_n3 < sample_n2 && sample_n2 > sample_n1 && sample_n2 > 10) {

    uint8_t velocity = min(6 * sqrt(sample_n2), 127);
    onNoteOn((Channel)1, (byte)36, (byte)velocity);
    delay(ALARM_MS);
    sample_n3 = 0;
    sample_n2 = 0;
    sample_n1 = 0;
    onNoteOff((Channel)1, (byte)36, (byte)0);
  }
}