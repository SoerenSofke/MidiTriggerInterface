//inline void meineFunktion() __attribute__((always_inline));

enum PedalState { INIT,
                  ON,
                  OFF };

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
int64_t ledOffset_callback(alarm_id_t id, __unused void* user_data) {
  if (midiDevAddr) {
    p.neoPixelFill(0, 32, 0, true);
  } else {
    p.neoPixelFill(0, 0, 128, true);
  }

  return 0;
}

// Drum Pedals
#define HIHAT_PIN 27
#define KICK_PIN 28

static uint32_t encodeVariables(uint16_t A, uint16_t B, uint16_t C) {
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

static uint16_t handleHiHatPedal(uint8_t pedalPosition_HiHat, uint16_t note) {
  switch (pedalPosition_HiHat) {
      // open
    case 0:
      return 46;
      // semiopen
    case 1 ... 15:
      return 100;
    case 16 ... 31:
      return 101;
    case 32 ... 47:
      return 102;
    case 48 ... 63:
      return 103;
    case 64 ... 79:
      return 104;
    case 80 ... 95:
      return 105;
      // close
    default:
      return 42;
  }
}

static constexpr uint8_t ccKick = 20;
static constexpr uint8_t notesKick[2] = { 35, 36 };
static uint8_t noteKick = notesKick[1];

static constexpr uint8_t ccSnare = 21;
static constexpr uint8_t notesSnare[2] = { 38, 40 };
static uint8_t noteSnare = notesSnare[0];

static constexpr uint8_t ccRide = 22;
static constexpr uint8_t notesRide[2] = { 51, 59 };
static uint8_t noteRide = notesRide[0];

static constexpr uint8_t notesRideBell[2] = { 53, 05 };
static uint8_t noteRideBell = notesRideBell[0];

static constexpr uint8_t ccCrash = 24;
static constexpr uint8_t notesCrash[2] = { 49, 57 };
static uint8_t noteCrash = notesCrash[0];

static constexpr uint8_t notesCrashChoke[2] = { 02, 03 };
static uint8_t noteCrashChoke = notesCrashChoke[0];


// timer for sampling ADCs
struct repeating_timer timer;

bool adcTimer_callback(struct repeating_timer* t) {
  uint16_t sampleKick = analogRead(KICK_PIN);
  uint16_t sampleHiHat = analogRead(HIHAT_PIN);
  uint32_t samplePedals = encodeVariables(1, sampleKick, sampleHiHat);

  rp2040.fifo.push_nb(samplePedals);
  return true;
}

uint8_t mapNote(uint8_t note) {
  note = note;

  switch (note) {
    case notesKick[0]:
    case notesKick[1]:
      {
        note = noteKick;
        break;
      }
    case notesSnare[0]:
    case notesSnare[1]:
      {
        note = noteSnare;
        break;
      }
    case notesRide[0]:
    case notesRide[1]:
      {
        note = noteRide;
        break;
      }
    case notesRideBell[0]:
    case notesRideBell[1]:
      {
        note = noteRideBell;
        break;
      }
    case notesCrash[0]:
    case notesCrash[1]:
      {
        note = noteCrash;
        break;
      }
    case notesCrashChoke[0]:
    case notesCrashChoke[1]:
      {
        note = noteCrashChoke;
        break;
      }
    default:
      note = note;
  }

  return note;
}


static void sendNoteOn(Channel channel, byte note, byte velocity) {
  if (velocity == 0) {
    sendNoteOff(channel, note);
    return;
  }

  note = mapNote(note);
  MIDIusb.sendNoteOn(note, velocity, channel);

  p.neoPixelFill(255, 0, 0, true);
  add_alarm_in_ms(ALARM_MS, ledOffset_callback, NULL, false);

  if (printEnabled && Serial) {
    Serial.printf("#%3u @%3u ON\r\n", note, velocity);
  }
}

static void sendNoteOff(Channel channel, byte note) {
  uint8_t velocity = 0;

  note = mapNote(note);
  MIDIusb.sendNoteOff(note, velocity, channel);

  if (printEnabled && Serial) {
    Serial.printf("#%3u @%3u OFF\r\n", note, velocity);
  }
}

/* MIDI IN MESSAGE REPORTING */
static void onMidiError(int8_t errCode) {
  if (printEnabled && Serial) {
    Serial.printf("> MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse" : "",
                  (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
                  (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx" : "");
  }
}

static void onNoteOn(Channel channel, byte note, byte velocity) {
  rp2040.fifo.push_nb(encodeVariables(2, note, velocity));
}

static void onNoteOff(Channel channel, byte note, byte velocity) {
  rp2040.fifo.push_nb(encodeVariables(3, note, velocity));
}


static void onControlChange(Channel channel, byte controller, byte value) {
  uint8_t note = 0;

  switch (controller) {
    case ccKick:
      {
        note = notesKick[value < 65];

        if (note == noteKick) return;
        else noteKick = note;
        break;
      }
    case ccSnare:
      {
        note = notesSnare[value < 65];

        if (note == noteSnare) return;
        else noteSnare = note;
        break;
      }
    case ccRide:
      {
        note = notesRide[value < 65];

        if (note == noteRide) return;
        else {
          noteRide = note;
          noteRideBell = notesRideBell[value < 65];
        }

        break;
      }
    case ccCrash:
      {
        note = notesCrash[value < 65];

        if (note == noteCrash) return;
        else {
          noteCrash = note;
          noteCrashChoke = notesCrashChoke[value < 65];
        }
        break;
      }
    default: return;
  }

  if (printEnabled && Serial) {
    Serial.printf("CC#%3u @%2u \r\n", controller, note);
  }

  p.neoPixelFill(0, 0, 255, true);
  add_alarm_in_ms(ALARM_MS, ledOffset_callback, NULL, false);
}

static void onPolyphonicAftertouch(Channel channel, byte note, byte amount) {}
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

  if (printEnabled && Serial) {
    Serial.printf("> MIDI device at address %u initialized.\r\n", devAddr);
  }

  midiDevAddr = devAddr;

  registerMidiInCallbacks();
}

static void onMIDIdisconnect(uint8_t devAddr) {
  p.neoPixelFill(0, 0, 128, true);

  midiDevAddr = 0;

  if (printEnabled && Serial) {
    Serial.printf("> MIDI device at address %u unplugged.\r\n", devAddr);
  }
}

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
  if (printEnabled && Serial) {
    Serial.println("> MIDI Trigger for Marlenes Drumkit :-)\r\n");
  }

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL) {
    delay(2000);
    if (printEnabled && Serial) {
      Serial.printf("> Error: CPU Clock = %u, PIO USB require CPU clock must be 120 Mhz.\r\n", cpu_hz);
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

  add_repeating_timer_ms(-1, adcTimer_callback, NULL, &timer);

  core0_booting = false;
  while (core1_booting)
    ;
}

void loop() {
  USBHost.task();
  usbhMIDI.readAll();
}

void setup1() {
  MIDIusb.begin();
  MIDIusb.turnThruOff();  // turn off echo

  Serial.begin(115200);
  p.neoPixelFill(0, 0, 128, true);

  core1_booting = false;
  while (core0_booting)
    ;
}

static void handlePedal(uint8_t note,
                        uint16_t& pedalPosition,
                        uint16_t& pedalPosition_n1,
                        uint16_t& pedalPosition_n2,
                        uint16_t& pedalPosition_n3,
                        uint16_t& pedalPosition_n4,
                        uint16_t& pedalPosition_n5,
                        PedalState& padelState) {

  // Map Position to 0 .. 127
  pedalPosition = max(pedalPosition - 25, 0);
  pedalPosition = min(pedalPosition, 255);
  pedalPosition = map(pedalPosition, 0, 255, 127, 0);

  // Buffer Most Recent Position Values
  pedalPosition_n5 = pedalPosition_n4;
  pedalPosition_n4 = pedalPosition_n3;
  pedalPosition_n3 = pedalPosition_n2;
  pedalPosition_n2 = pedalPosition_n1;
  pedalPosition_n1 = pedalPosition;

  // Check for Onset
  if (pedalPosition_n1 >= 96 && pedalPosition_n2 < 96 && padelState != PedalState::ON) {
    sendNoteOn((Channel)1, (byte)note, (byte)(pedalPosition_n1 - pedalPosition_n5));
    pedalPosition_n5 = pedalPosition_n4 = pedalPosition_n3 = pedalPosition_n3 = pedalPosition_n1 = 127;
    padelState = PedalState::ON;

    // Check for Offset
  } else if (pedalPosition_n1 < 32 && pedalPosition_n2 >= 32 && padelState != PedalState::OFF) {
    sendNoteOff((Channel)1, (byte)note);
    pedalPosition_n5 = pedalPosition_n4 = pedalPosition_n3 = pedalPosition_n3 = pedalPosition_n1 = 0;
    padelState = PedalState::OFF;
  }
}

void loop1() {
  if (rp2040.fifo.available()) {
    static PedalState padelState_Kick = PedalState::INIT;
    static PedalState padelState_HiHat = PedalState::INIT;

    static uint16_t pedalPosition_Kick = 0;
    static uint16_t pedalPosition_Kick_n1 = 0;
    static uint16_t pedalPosition_Kick_n2 = 0;
    static uint16_t pedalPosition_Kick_n3 = 0;
    static uint16_t pedalPosition_Kick_n4 = 0;
    static uint16_t pedalPosition_Kick_n5 = 0;

    static uint16_t pedalPosition_HiHat = 0;
    static uint16_t pedalPosition_HiHat_n1 = 0;
    static uint16_t pedalPosition_HiHat_n2 = 0;
    static uint16_t pedalPosition_HiHat_n3 = 0;
    static uint16_t pedalPosition_HiHat_n4 = 0;
    static uint16_t pedalPosition_HiHat_n5 = 0;

    uint16_t source = 0;
    uint16_t dataLeft = 0;
    uint16_t dataRight = 0;
    uint8_t note = 0;

    decodeVariables(rp2040.fifo.pop(), source, dataLeft, dataRight);

    switch (source) {
      case 1:  // Drums Pedals Control
        {
          note = 36;
          pedalPosition_Kick = dataRight;
          handlePedal(note,
                      pedalPosition_Kick,
                      pedalPosition_Kick_n1,
                      pedalPosition_Kick_n2,
                      pedalPosition_Kick_n3,
                      pedalPosition_Kick_n4,
                      pedalPosition_Kick_n5,
                      padelState_Kick);

          note = 44;
          pedalPosition_HiHat = dataLeft;
          handlePedal(note,
                      pedalPosition_HiHat,
                      pedalPosition_HiHat_n1,
                      pedalPosition_HiHat_n2,
                      pedalPosition_HiHat_n3,
                      pedalPosition_HiHat_n4,
                      pedalPosition_HiHat_n5,
                      padelState_HiHat);

          break;
        }
      case 2:
        {
          uint16_t note = dataLeft;
          uint16_t velocity = dataRight;
          uint16_t channel = 1;

          // if the HiHat was triggered, check the pedal position and modify the MIDI node
          if (note == 42) {
            note = handleHiHatPedal(pedalPosition_HiHat, note);
          }

          sendNoteOn(channel, note, velocity);
          break;
        }
      case 3:
        {
          uint16_t note = dataLeft;
          uint16_t channel = 1;
          sendNoteOff(channel, note);
          break;
        }
    }
  }
}