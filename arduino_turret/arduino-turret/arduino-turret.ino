#include <Servo.h>

#define echoPin 7 // Echo Pin
#define trigPin 8 // Trigger Pin
#define LEDPin 13 // Onboard LED
#define ServoSensor1Pin 5 // Horizontal
#define ServoSensor2Pin 6 // Vertical
#define ServoGun1Pin 9 // Horizontal
#define ServoGun2Pin 10 // Vertical

Servo servo_sensor1;  // create servo object to control a servo
Servo servo_sensor2;  // create servo object to control a servo
Servo servo_gun1;  // create servo object to control a servo
Servo servo_gun2;  // create servo object to control a servo

// Variables to store the servo positions
int pos_sensor1 = 90;
int pos_sensor2 = 90;
int pos_gun1 = 90;
int pos_gun2 = 90;

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration;
float distance; // Duration used to calculate distance

enum class RotateDir {
    FORWARD,
    BACKWARD
};

void rotate_servo(Servo& servo, int& pos, RotateDir direction) {
    if (direction == RotateDir::FORWARD) {
        if (pos < 150) {
            pos += 10;
            servo.write(pos);
        }
    } else {
        if (pos > 30) {
            pos -= 10;
            servo.write(pos);
        }
    }
}

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)

  servo_sensor1.attach(ServoSensor1Pin);  // attaches the servo on pin 9 to the servo objectư
  servo_sensor2.attach(ServoSensor2Pin);  // attaches the servo on pin 9 to the servo objectư
  servo_gun1.attach(ServoGun1Pin);  // attaches the servo on pin 9 to the servo objectư
  servo_gun2.attach(ServoGun2Pin);  // attaches the servo on pin 9 to the servo objectư

  // servo_sensor1.write(0);   // rotate slowly servo to 0 degrees immediately
  // servo_sensor2.write(0);   // rotate slowly servo to 0 degrees immediately
  // servo_gun1.write(0);   // rotate slowly servo to 0 degrees immediately
  // servo_gun2.write(0);   // rotate slowly servo to 0 degrees immediately
  servo_sensor1.write(pos_sensor1);   // rotate slowly servo to 0 degrees immediately
  servo_sensor2.write(pos_sensor2);   // rotate slowly servo to 0 degrees immediately
  servo_gun1.write(pos_gun1);   // rotate slowly servo to 0 degrees immediately
  servo_gun2.write(pos_gun2);   // rotate slowly servo to 0 degrees immediately
}

void loop() {
  // Sound wave trigger

  // 1. Wyczyszczenie pinu TRIG
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 2. Wysłanie impulsu 10 mikrosekund
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. Odczyt czasu trwania sygnału na pinie ECHO
  duration = pulseIn(echoPin, HIGH);

  // 4. Obliczenie odległości (prędkość dźwięku ~0.034 cm/us)
  // Dzielimy przez 2, bo fala biegnie w dwie strony
  distance = (duration * 0.034) / 2;

  // Wyświetlenie wyniku
  Serial.print("Zmierzona odległość: ");
  Serial.print(distance);
  Serial.println(" cm");

  //if (distance >= maximumRange || distance <= minimumRange){
  if (minimumRange >= distance || distance >= maximumRange){
    Serial.println(distance);
    digitalWrite(LEDPin, HIGH);
  }
  else {
    Serial.println(distance); 
    digitalWrite(LEDPin, LOW);
  }
 
  //Delay 50ms before next reading.
  delay(500);

  // Rotate servo sensor 1 forward
  rotate_servo(servo_sensor1, pos_sensor1, RotateDir::FORWARD);
  delay(15); // waits for the servo to reach the position

}