/*
 *    cnc vfd control for fanuc NC
 *    by eric barch (ericb@ericbarch.com)
 *    07.06.14
 */


// VFD params
// max AVI voltage 04.13
// MO1 output 03.01 (2 = master frequency attained, 19 = zero speed output signal)
// MO2 11.00 (2 = master frequency attained, 19 = zero speed output signal)

// 02.00 (set to value of 3 for RS-485 master freq source)
// 09.04 (set to value of 5 for 8-O-1 RTU)


// needed for vfd
#include <ModbusMaster.h>


// the 12 bits for speed command are scaled between 0 and this number for VFD frequency
#define MAX_RPM_VAL 180


#define LAST_OFF 1
#define LAST_FWD 2
#define LAST_REV 3

int last_drive_state = LAST_OFF;

/* INPUTS */
#define FIRST_INPUT_PIN 28

#define NC_INPUT_TORQUE_LIMIT_LOW 28
#define NC_INPUT_MACHINE_READY 29
#define NC_INPUT_HIGH_LOW_GEAR 30
#define NC_INPUT_TORQUE_LIMIT_HIGH 31
#define NC_INPUT_ORIENTATION_COMMAND 32
#define NC_INPUT_SPEED_BIT1 33
#define NC_INPUT_SPEED_BIT2 34
#define NC_INPUT_SPEED_BIT3 35
#define NC_INPUT_SPEED_BIT4 36
#define NC_INPUT_SPEED_BIT5 37
#define NC_INPUT_SPEED_BIT6 38
#define NC_INPUT_SPEED_BIT7 39
#define NC_INPUT_SPEED_BIT8 40
#define NC_INPUT_SPEED_BIT9 41
#define NC_INPUT_SPEED_BIT10 42
#define NC_INPUT_SPEED_BIT11 43
#define NC_INPUT_SPEED_BIT12 44
#define NC_INPUT_FORWARD 45
#define NC_INPUT_REVERSE 46
#define NC_INPUT_ESTOP 47

#define VFD_INPUT_ZERO_SPEED 48
#define VFD_INPUT_SPEED_ARRIVAL 49

#define LAST_INPUT_PIN 49


/* OUTPUTS */
#define FIRST_OUTPUT_PIN 22

#define NC_OUTPUT_ZERO_SPEED 22
#define NC_OUTPUT_SPEED_ARRIVAL 23
#define NC_OUTPUT_TORQUE_LIMITING 24
#define NC_OUTPUT_SPINDLE_ALARM 25
#define NC_OUTPUT_ORIENTATION_COMPLETE 26

#define LAST_OUTPUT_PIN 26


// instantiate modbus device as slave ID 1 (VFD)
ModbusMaster vfd(1);


/* INITIALIZATION */
void setup() {
  // init vfd
  vfd.begin(9600);
  
  // make inputs
  for (int i=FIRST_INPUT_PIN; i<=LAST_INPUT_PIN; i++) {
    pinMode(i, INPUT);
    
    // pullup mode (all optoisolated inputs have pullups already)
    if (i >= NC_INPUT_SPEED_BIT1 && i <= NC_INPUT_SPEED_BIT12)
      digitalWrite(i, HIGH);
  }
  
  // make outputs
  for (int i=FIRST_OUTPUT_PIN; i<=LAST_OUTPUT_PIN; i++) {
    pinMode(i, OUTPUT);
    
    // relays are ACTIVE HIGH, so this disables everything
    digitalWrite(i, LOW);
  }
}


void loop() {
  /* READ ALL INPUTS */
  
  // everything is active low, let's flip that for our booleans
  bool torque_limit_low = digitalRead(NC_INPUT_TORQUE_LIMIT_LOW) == LOW;
  bool machine_ready = digitalRead(NC_INPUT_MACHINE_READY) == LOW;
  bool high_low_gear = digitalRead(NC_INPUT_HIGH_LOW_GEAR) == LOW;
  bool torque_limit_high = digitalRead(NC_INPUT_TORQUE_LIMIT_HIGH) == LOW;
  bool orientation_command = digitalRead(NC_INPUT_ORIENTATION_COMMAND) == LOW;
  bool spindle_fwd_requested = digitalRead(NC_INPUT_FORWARD) == LOW;
  bool spindle_rev_requested = digitalRead(NC_INPUT_REVERSE) == LOW;
  bool e_stop_okay = digitalRead(NC_INPUT_ESTOP) == LOW;
  bool vfd_at_zero_speed = digitalRead(VFD_INPUT_ZERO_SPEED) == LOW;
  bool vfd_at_speed = digitalRead(VFD_INPUT_SPEED_ARRIVAL) == LOW;
  
  // calculate speed (0-4095) based on 12 bit input from NC
  uint16_t speed_to_dac = get_speed_from_nc();
  
  // are we good to enable?
  if (e_stop_okay) {
    
    // allow gear switching
    if (torque_limit_low) {
      digitalWrite(NC_OUTPUT_TORQUE_LIMITING, HIGH);
    } else {
      digitalWrite(NC_OUTPUT_TORQUE_LIMITING, LOW);
    }
    
    // set vfd freq
    if ((spindle_fwd_requested || spindle_rev_requested) && last_drive_state == LAST_OFF)
      set_vfd_freq(map(speed_to_dac, 0, 4095, 0, MAX_RPM_VAL));
    
    // vfd fwd, rev, or stop
    if (spindle_fwd_requested) {
      set_vfd_fwd();
      last_drive_state = LAST_FWD;
    } else if (spindle_rev_requested) {
      set_vfd_rev();
      last_drive_state = LAST_REV;
    } else {
      set_vfd_off();
      last_drive_state = LAST_OFF;
    }
    
    // HACK: WE HAVE TO DO THIS BECAUSE WE AREN'T GETTING STATE BACK FROM THE VFD DRIVE
    if (!spindle_fwd_requested && !spindle_rev_requested) {
      digitalWrite(NC_OUTPUT_ZERO_SPEED, HIGH);
      digitalWrite(NC_OUTPUT_SPEED_ARRIVAL, LOW);
    } else {
      digitalWrite(NC_OUTPUT_ZERO_SPEED, LOW);
      digitalWrite(NC_OUTPUT_SPEED_ARRIVAL, HIGH);
    }
    
    // enable the machine
    digitalWrite(NC_OUTPUT_SPINDLE_ALARM, HIGH);
  } else {
    // put the machine in a "safe" disabled state
    machine_safe_state();
    last_drive_state = LAST_OFF;
  }
  
  // wait a bit...
  delay(250);
}


/* VFD routines */
void set_vfd_freq(uint16_t freq) {
  static uint16_t data_address = 0x2001;
  static uint16_t freq_setpoint_data_content = freq*100;
  
  // set speed
  vfd.writeSingleRegister(data_address, freq_setpoint_data_content);
  
  // in case another VFD command is about to be sent
  delay(15);
}

void set_vfd_off() {
  // off
  vfd.writeSingleRegister(0x2000, 0x0001);
}

void set_vfd_fwd() {
  // fwd run
  vfd.writeSingleRegister(0x2000, 0x0012);
}

void set_vfd_rev() {
  // rev run
  vfd.writeSingleRegister(0x2000, 0x0022);
}


void machine_safe_state() {
  // turn off the drive
  set_vfd_off();
  
  // turn all relays off
  for (int i=FIRST_OUTPUT_PIN; i<=LAST_OUTPUT_PIN; i++) {
    // relays are ACTIVE HIGH
    digitalWrite(i, LOW);
  }
}


uint16_t get_speed_from_nc() {
  // calculate speed being sent to us from NC
  uint16_t desired_speed = 0;
  for (int i=0; i<12; i++) {
      if (digitalRead(NC_INPUT_SPEED_BIT1+i) == HIGH) {
        // because active low, this is a DIGITAL 0
      } else {
        // because active low, this is a DIGITAL 1
        desired_speed += round(pow(2, i));
      }
  }
  
  // return 12 bit value
  return desired_speed;
}
