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
 */

// the size of the buffer (ON + OFF)
// e.g. the value '800' corresponds to 400 event cycles
#define BUFFER_SIZE 800

#define RISE_DETECTOR 2
#define FALL_DETECTOR 3
#define OUTPUT_PIN 13

// default: enabled, direct mode
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
  const bool scheduleUpdate = (reader != writer);
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
    const unsigned int target = TIMESTAMP() + latency_ms;
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
    const unsigned int target = TIMESTAMP() + latency_ms;
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

/*
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
*/
