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

#define ALARM_MS 50
int64_t alarm_callback(alarm_id_t id, __unused void* user_data) {
  p.neoPixelFill(0, 32, 0, true);
  return 0;
}

// Drum Pedals
#define HIHAT_PIN 27
#define KICK_PIN 28

uint32_t encodeVariables(uint16_t A, uint16_t B, uint16_t C) {
  A = static_cast<uint16_t>(A & 0xFF);   // 8 Bit
  B = static_cast<uint16_t>(B & 0xFFF);  // 12 Bit
  C = static_cast<uint16_t>(C & 0xFFF);  // 12 Bit

  return static_cast<uint32_t>((A << 24) | (B << 12) | C);
}

static void decodeVariables(uint32_t packed, uint16_t& A, uint16_t& B, uint16_t& C) {
  A = static_cast<uint16_t>((packed >> 24) & 0xFF);   // 8 Bits
  B = static_cast<uint16_t>((packed >> 12) & 0xFFF);  // 12 Bits
  C = static_cast<uint16_t>(packed & 0xFFF);          // 12 Bits
}

// timer for sampling ADCs
struct repeating_timer timer;

bool timer_callback(struct repeating_timer* t) {
  uint16_t sampleKick = analogRead(KICK_PIN);
  uint16_t sampleHiHat = analogRead(HIHAT_PIN);
  uint32_t samplePedals = encodeVariables(1, sampleKick, sampleHiHat);

  rp2040.fifo.push_nb(samplePedals);
  return true;
}

static void sendNoteOn(Channel channel, byte note, byte velocity) {
  if (velocity == 0) {
    sendNoteOff(channel, note, velocity);
    return;
  }

  MIDIusb.sendNoteOn(note, velocity, channel);

  p.neoPixelFill(255, 0, 0, true);
  add_alarm_in_ms(ALARM_MS, alarm_callback, NULL, false);

  if (printEnabled) {
    Serial.printf("C%u: Note on#%u v=%u\r\n", channel, note, velocity);
  }
}

static void sendNoteOff(Channel channel, byte note, byte velocity) {
  MIDIusb.sendNoteOff(note, velocity, channel);

  if (printEnabled) {
    Serial.printf("C%u: Note off#%u v=%u\r\n", channel, note, velocity);
  }
}


/* MIDI IN MESSAGE REPORTING */
static void onNoteOn(Channel channel, byte note, byte velocity) {
  rp2040.fifo.push_nb(encodeVariables(2, note, velocity));
}

static void onNoteOff(Channel channel, byte note, byte velocity) {
  rp2040.fifo.push_nb(encodeVariables(3, note, velocity));
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
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(KICK_PIN, INPUT);
  pinMode(HIHAT_PIN, INPUT);

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

  USBHost.configure_pio_usb(0, &pio_cfg);
  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing work done in core1 to free up core0 for other work
  usbhMIDI.begin(&USBHost, 0, onMIDIconnect, onMIDIdisconnect);

  // Start ADC timer
  add_repeating_timer_ms(-1, timer_callback, NULL, &timer);

  core0_booting = false;
  while (core1_booting)
    ;
}

// core1's loop
void loop() {
  USBHost.task();
  usbhMIDI.readAll();
}

void setup1() {
  MIDIusb.begin();
  MIDIusb.turnThruOff();  // turn off echo

  Serial.begin(115200);
  while (!Serial)
    ;  // wait for native usb

  core1_booting = false;
  while (core0_booting)
    ;
}

volatile int16_t sample_n1 = 0;
volatile int16_t sample_n2 = 0;
volatile int16_t sample_n3 = 0;
volatile int16_t sample_n4 = 0;

volatile int16_t velocity = 0;

void loop1() {
  if (rp2040.fifo.available()) {
    uint16_t source = 0;
    uint16_t dataA = 0;
    uint16_t dataB = 0;
    decodeVariables(rp2040.fifo.pop(), source, dataA, dataB);

    switch (source) {
      case 1:  // Drums Pedal Control
        {
          // Kick Sample
          int16_t sample = dataA;

          sample_n4 = sample_n3;
          sample_n3 = sample_n2;
          sample_n2 = sample_n1;
          sample_n1 = sample;

          if (sample_n4 >= 100 && sample_n1 < 100) {
            velocity = max(min(sample_n3 - sample_n1, 127), 1);
            sendNoteOn((Channel)1, (byte)36, (byte)velocity);
            sample_n4 = sample_n3 = sample_n2 = sample_n1 = 0;
          } else if (sample_n4 <= 100 && sample_n1 > 100) {
            velocity = 0;
            sendNoteOff((Channel)1, (byte)36, (byte)velocity);
            sample_n4 = sample_n3 = sample_n2 = sample_n1 = 512;
          }

          //*
          Serial.print("0,1023,");
          Serial.print(sample);
          Serial.print(",");
          Serial.println(velocity);  // debug value
          //*/

          break;
        }
      case 2:
        {
          uint16_t note = dataA;
          uint16_t velocity = dataB;
          uint16_t channel = 1;
          sendNoteOn(channel, note, velocity);
          break;
        }
      case 3:
        {
          uint16_t note = dataA;
          uint16_t velocity = dataB;
          uint16_t channel = 1;
          sendNoteOff(channel, note, velocity);
          break;
        }
    }
  }
}