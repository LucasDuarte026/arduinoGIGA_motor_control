#include <Arduino.h>
#include <time.h>        
#include <chrono>        
#include "mbed.h"        


using namespace rtos;
using namespace mbed;       // For ThisThread, Callback, etc.

#define RGB_RED_PIN   PA_0 // Example pin, change as needed
#define RGB_GREEN_PIN PA_1 // Example pin, change as needed
#define RGB_BLUE_PIN  PA_2 // Example pin, change as needed

Mutex SerialMutex; // Mutex to protect Serial access

// Define the pins connected to the encoder A and B phases
// For Arduino Mega, Digital Pins 2 and 3 are external interrupt pins.
const int ENCODER_PIN_A = 18; // Intterrupt 5 pin 18 (D18) for encoder A
const int ENCODER_PIN_B = 19; // Interrupt  4 pin 19 (D19) for encoder B
const int DIRECTION_PIN = 12; // Pin 12 for motor direction control

const int BUTTON_BRAKE_PIN = 6; // Pin 6 for reading brake button 
const int SWITCH_BUTTON_DIRECTION_PIN = 5; // TO Change the direction of the motor

const int CONTROL_PWM_PIN = 3; 
const int BRAKE_PIN = 9;
const int CURRENT_SENSING_PIN = A0; // Pin A0 for current sensing (if needed)
const int POTENTIOMETER_PIN = A2; // Pin A2 for potentiometer input

struct SystemState {
    bool overall_sys = false;       // General system health (e.g., all good)
    bool reading_up = false;        // Indicates if sensor readings are actively updating
    bool brake_pressed = false;     // True if brake is pressed
    bool motor_running = false;     // True if motor is actively running (e.g., moving)
    bool motor_in_operation = false; // True if motor is powered/enabled, even if not moving
}sys_health; // Global system state variable

// This variable is modified by the Interrupt Service Routine (ISR) and read by the encoder_reader_task.
volatile long encoderCount = 0; // important to declare it as volatile to prevent the compiler from optimizing it out,

bool motor_direction = false; // false (LOW) for clockwise, true (HIGH) for counter-clockwise
// Variables to store the last known states of the encoder pins
// These are also volatile as they are modified inside the ISR.
volatile int lastEncoderStateA;
volatile int lastEncoderStateB;

// Queue to send encoder data from reader task to printer task
// The type of data in the queue is a pointer to long.
Queue<long, 100> encoder_data_queue; // Store long values, capacity of 10

// --- Task Function Prototypes ---
void system_health_task();    // Your LED blinking task
void encoder_reader_task();   // Task to read encoder and put data in queue
void encoder_printer_task();  // Task to get data from queue and print it
void drive_control_task();  // Task to get data from queue and print it

// --- Interrupt Service Routine (ISR) for Encoder ---
// This function will be called automatically when an interrupt occurs on ENCODER_PIN_A or ENCODER_PIN_B.

void setRgbColor(int red, int green, int blue) {
    analogWrite(RGB_RED_PIN, 255 - red);   // Invert for common anode
    analogWrite(RGB_GREEN_PIN, 255 - green); // Invert for common anode
    analogWrite(RGB_BLUE_PIN, 255 - blue);  // Invert for common anode
}

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

 
// --- Task Implementations ---
void system_health_task() {
    for (;;) {

        digitalWrite(LED_BUILTIN, HIGH); // Turn LED on er
        SerialMutex.lock();
        Serial.println("System health task: LED ON time is: ");
        SerialMutex.unlock();
        Serial.print(time(NULL)); Serial.println();
        ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
        digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
        ThisThread::sleep_for(std::chrono::seconds(1)); // Use chrono duration
    }
}

void encoder_reader_task() {
    // Variable to hold the count that we will send to the queue
    long currentEncoderPositionForQueue;

    for (;;) {
        // atomic read of the encoder count
        // This ensures that we read the encoderCount without being interrupted,
        noInterrupts();
        currentEncoderPositionForQueue = encoderCount;
        interrupts();

        // Only send data if the position has changed from the last time we sent it (it helps the overload when the motor is stopped).
        static long lastSentPosition = -999999; // Using static to retain value between calls
        if (currentEncoderPositionForQueue != lastSentPosition) {
            if (encoder_data_queue.try_put(&currentEncoderPositionForQueue)) {
                 // Data successfully sent.
                 lastSentPosition = currentEncoderPositionForQueue;
            } else {
                // If the queue is full, it means the printer task is not keeping up.
                SerialMutex.lock(); // Acquire mutex before printing
                // Serial.println("Encoder queue full, data lost!");
                SerialMutex.unlock(); 
            }
        }
        ThisThread::sleep_for(std::chrono::milliseconds(10)); // Check and send every 10ms
    }
}

void encoder_printer_task() {

    long receivedPosition; // Variable to store the data received from the queue
    // This prevents the task from consuming CPU when there's no data to print. just wait for data to be available.
    for (;;) {
        osEvent evt = encoder_data_queue.get(osWaitForever);

        // Check the status of the event.
        // osEventMessage indicates that a message (data) was successfully received.
        if (evt.status == osEventMessage) {
            // The received data is in evt.value.p (a pointer to void).
            // Cast it back to a pointer of the correct type (long*) and dereference it.
      
            SerialMutex.lock(); // Acquire mutex before printing
            receivedPosition = *(long*)evt.value.p;
            Serial.print(millis());
            Serial.print("| Encoder Position: ");
            Serial.println(receivedPosition);
            SerialMutex.unlock(); 
        } else {
            // This case should ideally not be reached with osWaitForever,
            // unless there's a critical error in Mbed OS (e.g., queue deleted or system error).
            SerialMutex.lock(); // Acquire mutex before printing
            Serial.print("Error receiving from encoder queue! Status: ");
            Serial.println(evt.status); // Print status for debugging
            SerialMutex.unlock(); 
        }
    }
}

// -------- ------------ Control the motor ----------- ---------------
String padLeft(long value, int width, char padChar = ' ') {
    String s = String(value);
    while (s.length() < width) {
        s = padChar + s;
    }
    return s;
}

void drive_control_task() {

    int pot_value; // Variable to store the data received from the queue
    // This prevents the task from consuming CPU when there's no data to print. just wait for data to be available.
    int past_time = millis(); // Record the start time for the task
    for (;;) {
      motor_direction = digitalRead(SWITCH_BUTTON_DIRECTION_PIN) ? HIGH : LOW; // Read the switch state to determine motor direction
      digitalWrite(DIRECTION_PIN, motor_direction ? HIGH : LOW); // Set direction based on the motor_direction variable
      // Serial.println("read break: " + String(digitalRead(BUTTON_BRAKE_PIN)));
      
      long rawRead =  analogRead(CURRENT_SENSING_PIN);
      float current = rawRead * (2000/3.3) * (3.3/4096);
      SerialMutex.lock();  
      // Serial.print(" |");
      // Serial.print(padLeft(rawRead, 10)); // Aqui você usa a função padLeft
      // Serial.print(" | current ");
      // Serial.println(current, 2);
      // SerialMutex.unlock();  
      if(digitalRead(BUTTON_BRAKE_PIN) == LOW) { // If the brake is engaged
        digitalWrite(BRAKE_PIN, 0); // Set PWM to 0 (brake)
        Serial.println("Brake engaged, motor stopping.");
      }
      else{
        pot_value = analogRead(POTENTIOMETER_PIN); // Read the potentiometer value from A0          (0 - 1024) 
        // Serial.println("direction: " + String(digitalRead(SWITCH_BUTTON_DIRECTION_PIN)));
        // Serial.println("POT: " + String(pot_value));
        
        
        analogWrite(CONTROL_PWM_PIN, pot_value / 4); // Scale the potentiometer value to PWM range  (0 -  255)
        if(millis() - past_time > 1000) { // Every second, print the potentiometer value
          // Serial.println("Potentiometer value: " + String(pot_value) +" | wrote: " + String(pot_value/4));
          past_time = millis(); // Reset 
        }
    }
    }
}




void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT); // Configure the built-in LED as output
    pinMode(DIRECTION_PIN, OUTPUT); // Configure the direction pin as output
    pinMode(CONTROL_PWM_PIN, OUTPUT); // Configure the control PWM pin as output
    pinMode(BRAKE_PIN, OUTPUT); // Configure the brake pin as output

    pinMode(BUTTON_BRAKE_PIN, INPUT_PULLUP); // Set the button pin as input to engage the brake
    pinMode(SWITCH_BUTTON_DIRECTION_PIN, INPUT_PULLUP); // Set the SWITCH pin as input to change motor direction

    // Give some time for the serial monitor to connect
    ThisThread::sleep_for(std::chrono::milliseconds(2000)); // 2 seconds
    Serial.println("Arduino Mega: Starting Mbed OS (FreeRTOS) tasks with interrupt-driven encoder...");

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
    Thread *sys_health_thread = new Thread(osPriorityNormal);
    sys_health_thread->start(callback(system_health_task));

    Thread *encoder_read_thread = new Thread(osPriorityNormal);
    encoder_read_thread->start(callback(encoder_reader_task));

    Thread *encoder_print_thread = new Thread(osPriorityNormal);
    // encoder_print_thread->start(callback(encoder_printer_task));

    Thread *drive_control_thread = new Thread(osPriorityNormal);
    drive_control_thread->start(callback(drive_control_task));
    
    sys_health.overall_sys = true; // Set the overall system state to true indicating the system is operational
}

// The loop() function is typically left empty when using Mbed OS/FreeRTOS
// as tasks handle all ongoing operations.
void loop() {}

