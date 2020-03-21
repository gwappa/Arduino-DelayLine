/*
 * DelayLine
 * 
 * @author gwappa
 * @date May 23, 2018
 * @license MIT
 * 
 * Digitally delays the input TTL signal.
 * The maximal throughput is determined by the delay latency and the BUFFER_SIZE.
 * 
 * [Input settings]
 * Arduino detects the rising and the falling edges on different interrupt pins.
 * They default to pins #2 (RISING) and #3 (FALLING);
 * check the RISE_DETECTOR and the FALL_DETECTOR macros.
 * 
 * If you are using one of Atmega328-type boards (e.g. UNO), there are almost
 * no other options but using the default settings.
 * 
 * [Output settings]
 * Output pin can be one of any other pins.
 * Specify it using the OUTPUT_PIN macro (default #13).
 * 
 * [Delay latency settings]
 * The delay latency can be configured by writing to the serial port.
 *
 * Send a single character to the port, where its ordinal is:
 *
 * $$ \mathrm{ord}(c) = \mathrm{ord}(' ') + r $$
 *
 * where $c$ is the character and $r$ is the command flag.
 * note that the code for the space character ' ' is 32.
 *
 * the command flag is interpreted as follows (note that the flag
 * can range from 0 to 63):
 *
 * | bit|     7|     6|     5|     4|     3|     2|     1|     0|
 * |----|------|------|------|------|------|------|------|------|
 * |desc|      |      |ENABLE|FEXP=1|  EXP1|  EXP0| FRAC1| FRAC0|
 * |    |      |      |      |FEXP=0|  LIN3|  LIN2|  LIN1|  LIN0|
 *
 * + The device will generate output only when `ENABLE` is asserted.
 * + If `FEXP` is asserted, bits 2 and 3 will be interpreted using the "exponential mode".
 *   Otherwise, bits 0-3 will be interpreted using the "linear mode".
 *   
 * + Exponential mode:
 *   - The exponent part `EXP` is calculated as `EXP1 * 2 + EXP0`.
 *   - If the exponent part is larger than `0`, then the fractional part `FRAC`
 *     is interpreted as `FRAC1 * 2 + FRAC0`.
 *   - The actual delay will be calculated as `(2 ^ FRAC + 1) * (10 ^ EXP)`
 *   (note the condition `EXP > 0`).
 *   
 * + Linear mode:
 *   - bits 0–3 will be evaluated as an unsigned integer `LIN`.
 *   - The actual delay will be calculated as `DELAY_FACTOR * LIN` ms.
 *   - The macro `DELAY_FACTOR` is 20 by default.
 *   
 * + If both the exponent and the fractional parts are `0`, 
 *   then the device falls into the "direct-output" mode (i.e. no delay).
 */

// the size of the buffer (ON + OFF)
#define BUFFER_SIZE 720
#define DELAY_FACTOR 20

#define RISE_DETECTOR 2
#define FALL_DETECTOR 3
#define OUTPUT_PIN 13

#define FLAG_ENABLE     ((char)0x20)
#define FLAG_EXPMODE    ((char)0x10)
#define FLAG_DELAY      ((char)0x0F)
#define FLAG_EXP        ((char)0x0C)
#define FLAG_FRAC       ((char)0x03)
#define FLAG_EXP1       ((char)0x08)
#define FLAG_EXP0       ((char)0x04)
#define FLAG_FRAC1      ((char)0x02)
#define FLAG_FRAC0      ((char)0x01)
#define HasDelay(CF)    ((CF) & FLAG_DELAY)
#define GetExponent(CF) (((CF) & FLAG_EXP)?  ( (((CF) & FLAG_EXP1 )?100:1) * (((CF) & FLAG_EXP0 )?10:1) ):1)
#define GetFraction(CF)                     ( (((CF) & FLAG_FRAC1)?  4:1) * (((CF) & FLAG_FRAC0)? 2:1) )
#define AsLinear(CF)    (((uint8_t)((CF) & FLAG_DELAY)) * (DELAY_FACTOR))

// default: enabled, direct mode
#define DEFAULT_CONFIG  ((char)0x30)
#define BAUD            115200

// offset-related macros
#define TIMESTAMP() ((unsigned int)millis())
#define INCREMENT(OF) if((++(OF)) == BUFFER_SIZE) (OF) = 0;
#define WaitingForOnEvent(OF) ((((int)OF) & 0x01) == 0)
#define WaitingForOffEvent(OF) ((((int)OF) & 0x01) != 0)

// output-related macros
#define TurnOn(PORT, MASK) *(PORT) |= (MASK);
#define TurnOff(PORT, MASK) *(PORT) &= ~(MASK);

// ON/OFF scheduler definitions
volatile unsigned int scheduler[BUFFER_SIZE]; // the scheduler, as a series of millisecond timestamps
                                               // the even/odd offsets correspond to ON/OFF schedules
volatile int reader = 0; // reader offset for the scheduler
volatile int writer = 0; // writer offset for the scheduler

// the latency settings
volatile bool         enabled;
volatile bool         hasDelay;
volatile unsigned int latency_ms;

// output pin-port settings
          uint8_t outputMask;
volatile  uint8_t *outputPort;

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  outputMask = digitalPinToBitMask(OUTPUT_PIN);
  outputPort = portOutputRegister(digitalPinToPort(OUTPUT_PIN));

  Serial.begin(BAUD);
  
  setConfig(DEFAULT_CONFIG);
  
  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(RISE_DETECTOR), onevent, RISING);
  attachInterrupt(digitalPinToInterrupt(FALL_DETECTOR), offevent, FALLING);
}

void loop() {
  if (!enabled || !hasDelay) {
    return;
  }

  // the scheduling only matters
  // when there is a delay in output
    
  // check if the reader offset is behind the writer offset
  // to see if the schedule must be updated.
  cli();
  // it is just an inequality check since it can be `reader > writer` (eg when an overflow occurred).
  bool scheduleUpdate = (reader != writer);
  sei();

  if (scheduleUpdate) {
    unsigned int current = TIMESTAMP();

    // move the reader offset to match the current timestamp
    while ( (reader != writer) && (scheduler[reader] <= current) ) {
      // Serial.print('+'); // uncomment to just to see that this logic works in reality...
      INCREMENT(reader);
    }

    // update the output status according to the position of the reader offset
    if (WaitingForOnEvent(reader)) {
      TurnOff(outputPort, outputMask);
    } else {
      TurnOn(outputPort, outputMask);
    }
  }
  
  // if it is in the direct-output mode,
  // output must have been already taken care of
  // by the interrupt handlers.
}

/**
 * handles rising-edge events.
 */
void onevent() {
  if (!enabled) {
    return;
  }

  if (hasDelay) {

    // update the scheduler
    unsigned int target = TIMESTAMP() + latency_ms;
    scheduler[writer] = target;
    INCREMENT(writer);

    // since Arduino may miss the previous event(s),
    // make sure that the writer offset be at the expected position.
    if (WaitingForOnEvent(writer)) {
      scheduler[writer] = target;
      INCREMENT(writer);
    }
    
  } else {
    // direct-output mode
    TurnOn(outputPort, outputMask);
  }
}

/**
 * handles falling-edge events.
 */
void offevent() {
  if (!enabled) {
    return;
  }

  if (hasDelay) {

    // update the scheduler
    unsigned int target = TIMESTAMP() + latency_ms;
    scheduler[writer] = target;
    INCREMENT(writer);

    // since Arduino may miss the previous event(s),
    // make sure that the writer offset be at the expected position.
    if (WaitingForOffEvent(writer)) {
      scheduler[writer] = target;
      INCREMENT(writer);
    }
    
  } else {
    // direct-output mode
    TurnOff(outputPort, outputMask);
  }
}

void serialEvent() {
  cli();
  int c = Serial.read();
  if (c < 0) {
    return;
  }
  
  char ch = (char)c;
  if (ch >= ' ') {
    setConfig(ch - ' ');
  }
  sei();
}

void setConfig(const char& ch) {
  enabled    = ((ch & FLAG_ENABLE) != 0);
  hasDelay   = HasDelay(ch);
  
  if (!hasDelay) {
    latency_ms = 0;
    
  } else {
    if (ch & FLAG_EXPMODE) {
      latency_ms = GetExponent(ch) * GetFraction(ch);
    } else {
      latency_ms = AsLinear(ch);
    }
    
  }

  // clear the previous buffer
  reader = 0;
  writer = 0;
    
  // make sure the output does not stay ON.
  TurnOff(outputPort, outputMask);

  Serial.print('@');
  // Serial.print((uint8_t)(ch));
  // Serial.print(" >>> ");
  if (enabled) {
    Serial.print("enabled");
    if (ch & FLAG_EXPMODE) {
      Serial.print("(exp)=");
    } else {
      Serial.print("(lin)=");
    }
    Serial.println(latency_ms);
  } else {
    Serial.println("disabled");
  }
}
