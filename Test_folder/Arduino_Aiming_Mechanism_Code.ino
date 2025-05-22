 
#include <Servo.h> // Include the Servo library to control servo motors
 
// Servo2 = Pan     Servo1 = Tilt
// Create two Servo objects to control two servo motors
Servo servo1; // Controls tilt
Servo servo2; // Controls pan
 
// Define the digital pins each servo is connected to
int servo1Pin = 9;  // Tilt servo connected to digital pin 9
int servo2Pin = 10; // Pan servo connected to digital pin 10
 
// Function to convert angle (0–270°) into corresponding pulse width (in microseconds)
// The specifc servos understand signals between ~500 to ~2500 microseconds
int angleToMicroseconds(int angle) {
 // Map the angle range 0–270° to a pulse width range of 500–2500 µs
 return map(angle, 0, 270, 500, 2500);
}
 
// Function to move a given servo to a specified angle
void moveServo(Servo &servo, int angle) {
 // Use writeMicroseconds instead of write(angle) for more precise control
 servo.writeMicroseconds(angleToMicroseconds(angle));
}
 
void setup() {
 // Attach each servo object to its designated pin
 servo1.attach(servo1Pin); // Connect tilt servo to pin 9
 servo2.attach(servo2Pin); // Connect pan servo to pin 10
 
 // Initialize the Serial Monitor for user input and messages
 Serial.begin(9600); 
 Serial.println("Enter angles for Servo 1 and Servo 2 (0-270°).");
}
 
void loop() {
 // Check if there's any data available from the Serial Monitor
 if (Serial.available() > 0) {
 
   // ===== Handle PAN (Servo2) =====
   Serial.println("Enter angle for pan (0-270):");
   while (!Serial.available()); // Wait until input is available
 
   int servo1Angle = Serial.parseInt(); // Read and convert the input into an integer
   while (Serial.available()) Serial.read(); // Clear remaining input from buffer
 
   // Validate the input angle
   if (servo1Angle >= 0 && servo1Angle <= 270) {
     moveServo(servo1, servo1Angle); // Move the tilt servo to specified angle
     Serial.print("Servo 1 moved to ");
     Serial.print(servo1Angle);
     Serial.println(" degrees.");
   } else {
     // Handle invalid input
     Serial.println("Invalid input. Enter a value between 0 and 270.");
   }
 
   delay(500); // Short pause for readability
 
   // ===== Handle TILT (Servo1) =====
   Serial.println("Enter angle for tilt (0-270):");
   while (!Serial.available()); // Wait for user input
 
   int servo2Angle = Serial.parseInt(); // Read tilt angle
   while (Serial.available()) Serial.read(); // Clear any extra input
 
   // Validate tilt angle
   if (servo2Angle >= 0 && servo2Angle <= 270) {
     moveServo(servo2, servo2Angle); // Move pan servo to specified angle
     Serial.print("Servo 2 moved to ");
     Serial.print(servo2Angle);
     Serial.println(" degrees.");
   } else {
     // Handle invalid input
     Serial.println("Invalid input. Enter a value between 0 and 270.");
   }
 
   delay(500); // Delay before next loop cycle
 }
 
 