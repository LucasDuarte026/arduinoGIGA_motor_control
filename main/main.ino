#include <Arduino.h>
#include <time.h>        
#include <chrono>        
#include "mbed.h"        
#include "math.h"        

/*
  This code is made by Lucas Duarte, for the Arduino Giga R1 board
  - It uses Mbed OS (FreeRTOS) to run tasks and handle interrupts for an encoder.
  - The code reads an encoder, measures velocity, and controls a motor based on a potentiometer
  - read the potentiometer value to control the motor speed and direction.
  - read the encoder to measure the motor's velocity.
  - read the current and voltage from the motor to use in the closed-control looṕ.
*/

using namespace rtos;
using namespace mbed;       // For ThisThread, Callback, etc.

#define TICKS_PER_REVOLUTION  1632.672       // Each revoluction = 48 CPR *  34.014:1 reduction = 1632,672 ticks
#define SUB_VCC_SUPPLY  3.3                  // The voltage supplied Bus from the Arduino Giga
#define GIGA_ADC_RESOLUTION  4096            // 2^12 = 4096 ADC resolution 12bits. but also can be other resolutions

Mutex SerialMutex; // Mutex to protect Serial access

// Define the pins interruption pins connected to the encoder A and B phases
const int ENCODER_PIN_A = 18; // Intterrupt 5 pin 18 (D18) for encoder A
const int ENCODER_PIN_B = 19; // Interrupt  4 pin 19 (D19) for encoder B

// Human interface pins
const int SWITCH_BUTTON_DIRECTION_PIN = 4; // TO read the switch for use to change the direction of the motor
const int BUTTON_BRAKE_PIN = 6; // Pin 6 for reading brake button to stop the motor
const int POTENTIOMETER_PIN = A2; // Pin A2 for potentiometer input as a general human interface
const int MOTOR_CURRENT_PIN = A3; // Pin A4 for VOLTAGE sensing from the motor with a current sensor ACS712-05B (5A)
const int MOTOR_VOLTAGE_PIN_NEGATIVE = A4; // Pin A4 for VOLTAGE sensing from the motor
const int MOTOR_VOLTAGE_PIN_POSITIVE = A5; // Pin A5 for VOLTAGE sensing from the motor

// Motor control pins for the Arduino motor-control Shield
const int DIRECTION_PIN = 12; // Pin 12 for motor direction control
const int CONTROL_PWM_PIN = 3; // Pin 3 for motor control PWM signal 8-bits
const int BRAKE_PIN = 9;       // Pin 9 for motor brake control HIGH or LOW


// Volatile are used to tell the compiler that these variables can be changed by an interrupt service routine (ISR).
// This variable is modified by the Interrupt Service Routine (ISR) and read by the encoder_reader_task.
volatile long encoderCount = 0; // important to declare it as volatile to prevent the compiler from optimizing it out,

bool motor_direction = false; // false (LOW) for clockwise, true (HIGH) for counter-clockwise

// Variables to store the last known states of the encoder pins
// These are also volatile as they are modified inside the ISR.
volatile int lastEncoderStateA;
volatile int lastEncoderStateB;
volatile long rawCurrent = 0; // Variable to store the raw current reading from the ACS712 sensor in the A9 pin
volatile float current = 0;   // Variable to store the current in mA calculated from the rawCurrent reading
volatile float motor_voltage = 0; // unit: V unfinished section to read the motor voltage
volatile float velocity = 0 ; // Variable to store the velocity in revolutions per second calculated from the encoder readings
volatile int pot_Rawvalue; // Variable to store the potentiometer value

// Queue to send encoder data from reader task to printer task
// The type of data in the queue is a pointer to long.

// --- Task Function Prototypes ---
void system_health_task();    // Your LED blinking task
void measure_velocity_task();   // Task to read encoder and put data in queue
void drive_control_task();   // Task to read encoder and put data in queue

 
// Function to pad an integer with spaces to a specified width for better formatting and debugging
String padValue(int value, int width) {
  String s = String(value); // Convert the integer to a String
  while (s.length() < width) { // While the string length is less than the desired width
    s = " " + s; // Prepend a space
  }
  return s; // Return the padded string
}

// Interrupt Service Routine (ISR) to handle encoder changes and count the encoder ticks
void handleEncoderChange() {
  // Disable further interrupts briefly to ensure atomic reading of both pins.
  noInterrupts();

  int currentEncoderStateA = digitalRead(ENCODER_PIN_A);
  int currentEncoderStateB = digitalRead(ENCODER_PIN_B);

  // Check if either pin's state has actually changed from its last known state.

  // Determine direction and update encoderCount by checking the state of B relative to A's current state.
  if (currentEncoderStateA != lastEncoderStateA) { // If A has changed
    if (currentEncoderStateA == currentEncoderStateB) {
      encoderCount++; // Clockwise rotation
    } else {
      encoderCount--; // Counter-clockwise rotation 
    }
  }

  // ensure we only process if A *didn't* change but B did.
  else if (currentEncoderStateB != lastEncoderStateB) { // If B has changed (and A did not)
    // The logic here is essentially the inverse of the first 'if' block's logic
    if (currentEncoderStateA != currentEncoderStateB) {
      encoderCount++; // Clockwise rotation
    } else {
      encoderCount--; // Counter-clockwise rotation
    }
  }

  // Update the last known states for the next interrupt
  lastEncoderStateA = currentEncoderStateA;
  lastEncoderStateB = currentEncoderStateB;

  interrupts(); // Re-enable interrupts
}

// A simple thread to blink the built-in LED for system health check
void system_health_task() {
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH); // Turn LED on er
          ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
        digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
        ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
    }
}

// Task to measure velocity from the encoder readings
void measure_velocity_task() {
    // Variables to store the encoder count and time from the last velocity calculation
    long last_calculated_encoderCount = 0;
    unsigned long last_calculated_time_us = micros(); // micros for the maximum precision in microseconds
    float RAWvelocity = 0.0;
    for (;;) {
      
      // Check if enough time has passed since the last velocity calculation
      unsigned long current_time_us = micros(); // Get current time in microseconds
      noInterrupts(); // Temporarily disable interrupts to atomically read encoderCount
      long current_encoderCount = encoderCount;
      interrupts(); // Re-enable interrupts

      // Calculate the change in encoder count and time since the last calculation
      long delta_encoder = current_encoderCount - last_calculated_encoderCount;
      unsigned long delta_time_us = current_time_us - last_calculated_time_us;

      // Calculate velocity (ticks per second)
      // Ensure delta_time_us is not zero to prevent division by zero error
      if (delta_time_us > 0) { 
            // multiply 1000000.0F to convert to seconds
          RAWvelocity = (static_cast<float>(delta_encoder) * 1000000.0F) / static_cast<float>(delta_time_us);
      } else {
          RAWvelocity = 0; // If no time has passed, assume zero velocity
      }
      velocity = RAWvelocity/TICKS_PER_REVOLUTION; // Store the calculated velocity in revolutions per second
      // Update the last calculated encoder count and time for the next cycle
      last_calculated_encoderCount = current_encoderCount;
      last_calculated_time_us = current_time_us;

      // Optional: Print the calculated velocity for debugging
      // SerialMutex.lock();
      // Serial.print("| (rev/min): ");
      // Serial.println(velocity*60/TICKS_PER_REVOLUTION);
      // SerialMutex.unlock();
      
      // Serial.println("RAW: "+ padValue(rawCurrent,5) +" | Current: "+ String(current,2) + " (mA)");

        
      // -- -- --  -- -- --  -- -- --  -- -- -- unifinished section to read the motor voltage: -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- -- 
          long raw_motor_voltage_plus = analogRead(MOTOR_VOLTAGE_PIN_POSITIVE);
          long raw_motor_voltage_negative = analogRead(MOTOR_VOLTAGE_PIN_NEGATIVE);
          // Factor 5 comes from the voltage divider used to measure the motor voltage 5 = (R1 + R2) / R2, where R1 = 30k and R2 = 7.5k 
          motor_voltage = (static_cast<float>(raw_motor_voltage_plus-raw_motor_voltage_negative)*5.0)/GIGA_ADC_RESOLUTION; 
          // Optional: Print the calculated tension for debugging
          // SerialMutex.lock();
          // Serial.print(raw_motor_voltage_plus);
          // Serial.print(",");
          // Serial.print(raw_motor_voltage_negative);
          // Serial.println();
          // SerialMutex.unlock();

        } 
      // -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- --  -- -- -- 
      ThisThread::sleep_for(std::chrono::milliseconds(10)); // Sleep for 10 milliseconds to avoid busy-waiting
} 

// Task to control the motor based on the potentiometer value and encoder readings
void drive_control_task() {

    // This prevents the task from consuming CPU when there's no data to print. just wait for data to be available.
    int past_time = millis(); // Record the start time for the task
    for (;;) {
      rawCurrent = analogRead(A9); // Read the current sensor value A9 definedbefore 
      current = ((static_cast<float>(rawCurrent)*(SUB_VCC_SUPPLY/GIGA_ADC_RESOLUTION)- (SUB_VCC_SUPPLY/2.0))*(1000.0/0.185)); // Convert to current in mA     
      motor_direction = digitalRead(SWITCH_BUTTON_DIRECTION_PIN) ? HIGH : LOW; // Read the switch state to determine motor direction
      digitalWrite(DIRECTION_PIN, motor_direction ? HIGH : LOW); // S3et direction based on the motor_direction variable
      if(digitalRead(BUTTON_BRAKE_PIN) == LOW) { // If the brake is engaged
        digitalWrite(BRAKE_PIN, LOW); // Set PWM to 0 (brake)
        Serial.println("Brake engaged, motor stopping.");
      }
      else{
        pot_Rawvalue = analogRead(POTENTIOMETER_PIN); // Read the potentiometer value from A0          (0 - 1024) 
        analogWrite(CONTROL_PWM_PIN, static_cast<float>(pot_Rawvalue) * (1.0/4.0) * (1024.0/GIGA_ADC_RESOLUTION)); // Scale the potentiometer value to PWM range  (0 -  255)
        
        // Optional: Every second, print the potentiometer value
        // if(millis() - past_time > 1000) { 
        //   Serial.println("Potentiometer value: " + String(pot_Rawvalue) +" | wrote: " + String(pot_Rawvalue/4));
        //   past_time = millis(); // Reset 
        // }
    }
  }
}


void setup() {
    Serial.begin(115200);            // initialize serial communication at 115200 baud rate
    pinMode(LED_BUILTIN, OUTPUT); // Configure the built-in LED as output to system health check
    pinMode(DIRECTION_PIN, OUTPUT); // Configure the direction pin as output
    pinMode(CONTROL_PWM_PIN, OUTPUT); // Configure the control PWM pin as output
    pinMode(BRAKE_PIN, OUTPUT); // Configure the brake pin as output

    pinMode(BUTTON_BRAKE_PIN, INPUT_PULLUP); // Set the button pin as input to engage the brake
    pinMode(SWITCH_BUTTON_DIRECTION_PIN, INPUT_PULLUP); // Set the SWITCH pin as input to change motor direction
    
    analogReadResolution(std::log2(GIGA_ADC_RESOLUTION)); // define the ADC resolution to 12 bits (4096 levels)

    // Give some time for the serial monitor to connect
    ThisThread::sleep_for(std::chrono::milliseconds(2000)); // 2 seconds
    Serial.println("Arduino Mega: Starting Mbed OS (built with FreeRTOS underneath) tasks with interrupt-driven encoder...");

    // Configure encoder pins as inputs with internal pull-up resistors
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    // Read the initial state of the encoder pins before attaching interrupts
    lastEncoderStateA = digitalRead(ENCODER_PIN_A);
    lastEncoderStateB = digitalRead(ENCODER_PIN_B);

    // Attach the single interrupt service routine (ISR) to both encoder pins.
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderChange, CHANGE); // The 'CHANGE' mode triggers the ISR on both rising and falling edges.
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderChange, CHANGE);


    // Create and start tasks

    // Create a thread for system health check (LED blinking)
    Thread *sys_health_thread = new Thread(osPriorityNormal);
    sys_health_thread->start(callback(system_health_task));

    // Create a thread just for measuring velocity from the motor from the encoder readings
    Thread *encoder_read_thread = new Thread(osPriorityNormal);
    encoder_read_thread->start(callback(measure_velocity_task));

    // Create a thread for controlling the motor based on the potentiometer value and encoder readings
    Thread *drive_control_task_thread = new Thread(osPriorityNormal);
    drive_control_task_thread->start(callback(drive_control_task));

}

// loop function is not used in this case, as we are using FreeRTOS tasks
void loop() {}

