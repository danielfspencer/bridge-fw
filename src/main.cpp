#include <Arduino.h>

// timings (ns)
#define DRIVER_SETUP_TIME 10
#define DRIVER_HOLD_TIME 10

#define SAMPLER_SETUP_TIME 10
#define SAMPLER_HOLD_TIME 25

#define MIN_CLK_PERIOD 10

// pins
#define DRIVER_DATA 0

#define DRIVER_EN_N_OFFSET 0
#define DRIVER_CLK_OFFSET  1
#define SAMPLER_OUT_OFFSET 2
#define SAMPLER_CLK_OFFSET 3

#define READ_BUS_BASE      1
#define DATA_BUS_BASE      5
#define WRITE_BUS_BASE     9

#define CLK_AND_RST_EN_N   23
#define RST                22
#define CTRL_CLK           21
#define READ_CLK           20
#define WRITE_CLK          19

#define STATUS_EN          15
#define STATUS_BIT0        16
#define STATUS_BIT1        17
#define STATUS_BIT2        18

enum bus_type {
  READ_BUS = READ_BUS_BASE,
  DATA_BUS = DATA_BUS_BASE,
  WRITE_BUS = WRITE_BUS_BASE
};

enum status_leds {
  OFF = -1,
  SELF_TEST,
  STEP,
  TRANSACTION_READ,
  TRANSACTION_WRITE,
  TRANSACTION_COPY,
  SAMPLE_BUS_READ,
  SAMPLE_BUS_DATA,
  SAMPLE_BUS_WRITE
};

const bus_type BUSES[] = {READ_BUS, DATA_BUS, WRITE_BUS};

void set_status_leds(status_leds);
void self_test();
void set_bus_enable(bus_type bus, bool);
void write_word(uint16_t data, bus_type bus);

void setup() {
  // ---- pin mode setup
  pinMode(DRIVER_DATA, OUTPUT);

  // buses
  for (int i = 0; i < 3; i++) {
    pinMode(BUSES[i] + DRIVER_EN_N_OFFSET,  OUTPUT);
    pinMode(BUSES[i] + DRIVER_CLK_OFFSET,  OUTPUT);
    pinMode(BUSES[i] + SAMPLER_OUT_OFFSET,  INPUT);
    pinMode(BUSES[i] + SAMPLER_CLK_OFFSET,  OUTPUT);
  }

  // clocks and reset
  pinMode(CLK_AND_RST_EN_N, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(CTRL_CLK, OUTPUT);
  pinMode(READ_CLK, OUTPUT);
  pinMode(WRITE_CLK, OUTPUT);

  digitalWrite(CLK_AND_RST_EN_N, LOW);
  digitalWrite(RST, LOW);
  digitalWrite(CTRL_CLK, LOW);
  digitalWrite(READ_CLK, LOW);
  digitalWrite(WRITE_CLK, LOW);

  // ---- initial output setup
  set_status_leds(OFF);

  digitalWrite(CLK_AND_RST_EN_N, HIGH);
  digitalWrite(RST, LOW);

  Serial.begin(9600);
  delay(1000);
  self_test();
}

void set_status_leds(status_leds status) {
  if (status < 0) {
    digitalWrite(STATUS_EN, LOW);
  } else {
    digitalWrite(STATUS_BIT0, status & 1 << 0);
    digitalWrite(STATUS_BIT1, status & 1 << 1);
    digitalWrite(STATUS_BIT2, status & 1 << 2);
    digitalWrite(STATUS_EN, HIGH);
  }
}

inline void write_bit(bool bit, bus_type bus) {
  digitalWriteFast(bus + DRIVER_CLK_OFFSET, LOW);
  digitalWriteFast(DRIVER_DATA, bit);
  delayNanoseconds(DRIVER_SETUP_TIME);
  digitalWriteFast(bus + DRIVER_CLK_OFFSET, HIGH);
  delayNanoseconds(DRIVER_HOLD_TIME);
}

void write_word(uint16_t data, bus_type bus) {
  digitalWriteFast(bus + DRIVER_EN_N_OFFSET, HIGH);
  for (int i = 0; i < 16; i++) {
    bool bit = data & 1;
    data >>= 1;

    write_bit(bit, bus);
  }

  write_bit(0, bus);
  digitalWriteFast(bus + DRIVER_EN_N_OFFSET, LOW);
}

inline int read_bit(bus_type bus) {
  digitalWriteFast(bus + SAMPLER_CLK_OFFSET, LOW);
  delayNanoseconds(SAMPLER_SETUP_TIME);
  digitalWriteFast(bus + SAMPLER_CLK_OFFSET, HIGH);
  delayNanoseconds(SAMPLER_HOLD_TIME);

  return digitalReadFast(bus + SAMPLER_OUT_OFFSET);
}

uint16_t read_word(bus_type bus) {
  uint16_t result = digitalRead(bus + SAMPLER_OUT_OFFSET);
  for (int i = 1; i < 16; i++) {
    result |= read_bit(bus) << i;
  }

  return result;
}

void set_bus_enable(bus_type bus, bool enable) {
  digitalWrite(bus + DRIVER_EN_N_OFFSET, !enable);
}

bool test_bus(bus_type bus) {
  // disable clock drive
  digitalWrite(CLK_AND_RST_EN_N, 1);

  // enable bus driver
  set_bus_enable(bus, true);

  bool success = true;

  for (int j = 0; j < 1; j++) {
    for (int i = 0; i <= 0xffff; i++) {
      write_word(i, bus);

      // produce high-low tranistion on write clock (latches data)
      digitalWriteFast(WRITE_CLK, HIGH);
      delayNanoseconds(MIN_CLK_PERIOD / 2);
      digitalWriteFast(WRITE_CLK, LOW);
      delayNanoseconds(MIN_CLK_PERIOD / 2);

      uint16_t result = read_word(bus);

      if (result != i) {
        success = false;
        break;
      }
    }
  }

  set_bus_enable(bus, false);
  return success;
}

void self_test() {
  set_status_leds(SELF_TEST);
  Serial.println("Self test...");

  // for each bus
  for (int i = 0; i < 3; i++) {
    set_bus_enable(BUSES[i], true);

    switch (i) {
      case 0: Serial.print("read bus"); break;
      case 1: Serial.print("data bus"); break;
      case 2: Serial.print("write bus"); break;
    }

    // run test
    if(test_bus(BUSES[i])) {
      Serial.println(" [ok]");
    } else {
      Serial.println(" [failed]");

      for (int i = 0; i < 3; i++) {
        set_bus_enable(BUSES[i], false);
      }

      // flash 'self test' light and the bus light that has failed
      while (true) {
        for (int j = 0; j < 100; j++) {
          set_status_leds(SELF_TEST);
          delay(1);
          set_status_leds(i + 5);
          delay(1);
        }
        set_status_leds(OFF);
        delay(100);
      }
    }

    set_bus_enable(BUSES[i], false);
  }

  set_status_leds(OFF);
}


void loop() {
  // TODO listen for transaction commands from host
  self_test();
}
