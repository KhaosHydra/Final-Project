// #include <Arduino.h>
// #include <Wire.h>
// #include <ESP32Servo.h>

// const int in3 = 27;
// const int in4 = 33;
// const int button = 39;

// int startGame = false;

// void setup() {
//     Serial.begin(9600);
//   // put your setup code here, to run once:
//     pinMode(in3, OUTPUT);
//     pinMode(in4, OUTPUT);
//     pinMode(button, INPUT_PULLUP);
// }

// bool runMotorCycle = false; // Flag to control when the cycle runs

// void handleMotorCycle() {
//     static unsigned long lastActionTime = 0; // Tracks the time of the last action
//     static int step = 0; // Tracks the current step in the cycle
//     static bool cycleRunning = false; // Tracks if the cycle is currently running

//     if (!runMotorCycle) {
//         // Reset the state to ensure the motor stops and is ready for the next call
//         if (cycleRunning) {
//             step = 0;
//             digitalWrite(in3, LOW);
//             digitalWrite(in4, LOW); // Ensure motor is stopped
//             cycleRunning = false;
//         }
//         return; // Do nothing unless runMotorCycle is true
//     }

//     // Proceed with the cycle
//     unsigned long currentTime = millis();

//     if (!cycleRunning) {
//         // Start the cycle
//         cycleRunning = true;
//         lastActionTime = currentTime;
//     }

//     switch (step) {
//         case 0: // Step 0: Spin motor forward for 1 second
//             digitalWrite(in3, HIGH);
//             digitalWrite(in4, LOW);
//             if (currentTime - lastActionTime >= 500) { // Check if 1 second has elapsed
//                 digitalWrite(in3, LOW);
//                 digitalWrite(in4, LOW); // Stop motor
//                 lastActionTime = currentTime; // Update the last action time
//                 step = 1; // Move to the next step
//             }
//             break;

//         case 1: // Step 1: Pause for 5 seconds
//             if (currentTime - lastActionTime >= 5000) { // Check if 5 seconds have elapsed
//                 lastActionTime = currentTime; // Update the last action time
//                 step = 2; // Move to the next step
//             }
//             break;

//         case 2: // Step 2: Spin motor backward for 900ms
//             digitalWrite(in3, LOW);
//             digitalWrite(in4, HIGH);
//             if (currentTime - lastActionTime >= 900) { // Check if 900ms have elapsed
//                 digitalWrite(in3, LOW);
//                 digitalWrite(in4, LOW); // Stop motor
//                 lastActionTime = currentTime; // Update the last action time
//                 step = 3; // Move to the next step
//             }
//             break;

//         case 3: // Step 3: Cycle complete, stop and reset
//             digitalWrite(in3, LOW);
//             digitalWrite(in4, LOW); // Ensure motor is stopped
//             runMotorCycle = false; // Mark the cycle as done
//             cycleRunning = false; // Reset the cycle flag
//             step = 0; // Reset steps for the next call
//             break;

//         default:
//             step = 0; // Reset in case of an invalid step
//             break;
//     }
// }


// void loop() {
//   // put your main code here, to run repeatedly:
//     int buttonState = digitalRead(button);

//     if (buttonState == HIGH) {
//         startGame = !startGame; // Toggle startGame when the button is pressed
//         runMotorCycle = true; // Start the motor cycle
//         Serial.println(startGame ? "Game Started" : "Game Stopped");
//     }

//     handleMotorCycle(); // Call the motor cycle function
// }