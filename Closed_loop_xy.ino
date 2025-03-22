#include <ESP8266WiFi.h>

const char *ssid = "ravi";
const char *password = "ravi@123";

int port = 80;
WiFiServer server(port);

// Variables
volatile int pulseCount = 0;      // Counter for the number of pulses
unsigned long prevMillis = 0;     // To keep track of time
float rpm = 0;                    // Motor speed in RPM
int pulsesPerRevolution = 696;     // Encoder pulses per revolution (adjust according to your motor)

volatile int pulseCount1 = 0;      // Counter for the number of pulses
unsigned long prevMillis1 = 0;     // To keep track of time
float rpm1 = 0;                    // Motor speed in RPM
int pulsesPerRevolution1 = 696;     // Encoder pulses per revolution (adjust according to your motor)
float r = 0.02;                    // radius                     
float L = 0.175;                    // length of axis
float a = 0.07;

float kx = 0.5;
float ky = 0.5;
float lx = 0.1;
float ly = 0.1;

float theta_old = 0;
float x_old = 0;
float y_old = 0;

// PID Variables

float kp = 3.0;                    // Proportional gain
float ki = 8.0;                    // Integral gain
float kd = 0;                    // Derivative gain

float prevError = 0;
float integral = 0;

float prevError1 = 0;
float integral1 = 0;

float dt = 0.1;                    // Time step (seconds) -> 100ms


// Kalman filter variables for motor 1
float x_k = 0;                     // Estimated RPM (Kalman)
float P_k = 1;                     // Error covariance
float Q = 1;                    // Process noise covariance
float R = 30;                       // Measurement noise covariance
float K_k = 0;                     // Kalman gain

// Kalman filter variables for motor 2
float x_k1 = 0;                    // Estimated RPM (Kalman)
float P_k1 = 1;                    // Error covariance
float K_k1 = 0;                    // Kalman gain


// Interrupt Service Routine (ISR) for the encoder
void IRAM_ATTR pulseCounter() {    // "IRAM_ATTR" is specific for ESP8266 to handle ISRs
  pulseCount++;
}

void IRAM_ATTR pulseCounter1() {    // "IRAM_ATTR" is specific for ESP8266 to handle ISRs
  pulseCount1++;
}

// PID Control Function for Motor 1
float computePID(float desiredRPM, float currentRPM, float &prevError, float &integral) {
  float error = desiredRPM - currentRPM;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  // PID formula
  float output = kp * error + ki * integral + kd * derivative;
  
  // Limit the output to a range suitable for motor control (e.g., 0-255 for PWM)
  output = constrain(output, 0, 1023);
  
  return output;
}

// PID Control Function for Motor 2
float computePID1(float desiredRPM1, float currentRPM1, float &prevError1, float &integral1) {
  float error1 = desiredRPM1 - currentRPM1;
  integral1 += error1 * dt;
  float derivative1 = (error1 - prevError1) / dt;
  prevError1 = error1;
  
  // PID formula
  float output1 = kp * error1 + ki * integral1 + kd * derivative1;
  
  // Limit the output to a range suitable for motor control (e.g., 0-255 for PWM)
  output1 = constrain(output1, 0, 1023);
  
  return output1;
}


// Kalman filter update function for motor 1
float kalmanFilterUpdate(float z_k, float &x_k, float &P_k, float Q, float R) {
  // Prediction step
  P_k = P_k + Q;

  // Update step
  K_k = P_k / (P_k + R);
  x_k = x_k + K_k * (z_k - x_k);
  P_k = (1 - K_k) * P_k;

  return x_k;
}

// Kalman filter update function for motor 2
float kalmanFilterUpdate1(float z_k1, float &x_k1, float &P_k1, float Q, float R) {
  // Prediction step
  P_k1 = P_k1 + Q;

  // Update step
  K_k1 = P_k1 / (P_k1 + R);
  x_k1 = x_k1 + K_k1 * (z_k1 - x_k1);
  P_k1 = (1 - K_k1) * P_k1;

  return x_k1;
}


void setup() {
  Serial.begin(115200);                       // Initialize serial communication (ESP8266 usually runs better at 115200 baud)
  attachInterrupt(digitalPinToInterrupt(D2), pulseCounter, RISING);  // Attach interrupt to pin D2 (GPIO4)
  attachInterrupt(digitalPinToInterrupt(D8), pulseCounter1, RISING);
  analogWrite(D4, 0);  // Motor control
  analogWrite(D5, 0);
  Serial.println();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  Serial.println("AP started");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

void loop() {
    unsigned long currentMillis = millis();    // Get current time
    float timeInSeconds = currentMillis / 1000.0;
    
    if (currentMillis - prevMillis >= 100) {  // Every 100 ms
      detachInterrupt(digitalPinToInterrupt(D2)); // Disable interrupt temporarily
      detachInterrupt(digitalPinToInterrupt(D8)); // Disable interrupt temporarily

      // Calculate RPM
      rpm = (pulseCount / 0.1) / ((float)pulsesPerRevolution) * 60.0;  // Convert pulses to RPM
      rpm1 = (pulseCount1 / 0.1) / ((float)pulsesPerRevolution1) * 60.0;  // Convert pulses to RPM

      float lin_vel = (2 * PI/ 60) * r * (rpm + rpm1) / 2;
      float ang_vel = (2 * PI / 60) * r * (rpm - rpm1) / L;

      float x_dot = lin_vel * cos(theta_old);
      float y_dot = lin_vel * sin(theta_old);
      float theta_dot = ang_vel;

      float x = x_old + x_dot * 0.1;
      float y = y_old + y_dot * 0.1;
      float theta = theta_old + theta_dot * 0.1;

//      float des_x = sin(2*PI*0.5*timeInSeconds);
//      float des_y = cos(2*PI*0.5*timeInSeconds);
      float des_x = 2;
      float des_y = 2;

      float des_x_dot = 0*PI*cos(2*PI*0.5*timeInSeconds);
      float des_y_dot = -0*PI*sin(2*PI*0.5*timeInSeconds);
      
      float des_linvel = cos(theta)*(des_x_dot + lx*tanh(kx*(des_x - x)/lx)) + sin(theta)*(des_y_dot + ly*tanh(ky*(des_y - y)/ly));
      float des_angvel = -(1/a)*sin(theta)*(des_x_dot + lx*tanh(kx*(des_x - x)/lx)) + (1/a)*cos(theta)*(des_y_dot + ly*tanh(ky*(des_y - y)/ly));

      float desiredRPM = (30/PI)*((1/r)*(des_linvel) + (L/(2*r))*(des_angvel));   // converting rad/sec to RPM
      float desiredRPM1 = (30/PI)*((1/r)*(des_linvel) - (L/(2*r))*(des_angvel));

      desiredRPM = constrain(desiredRPM, 0, 240);
      desiredRPM1 = constrain(desiredRPM1, 0, 240);
      
      // PID control to adjust motor speeds
      float controlOutput = computePID(desiredRPM, rpm, prevError, integral);
      float controlOutput1 = computePID1(desiredRPM1, rpm1, prevError1, integral1);

       // Here you would typically set the motor speed based on the control output (e.g., via PWM)

      analogWrite(D4, controlOutput);  // Motor control
      analogWrite(D5, controlOutput1);

      // Print the result
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(", y: ");
      Serial.println(y);
//      Serial.print(", theta: ");
//      Serial.println(theta);
      
      // Reset pulse count and timer
      pulseCount = 0;
      pulseCount1 = 0;
      prevMillis = currentMillis;
      theta_old = theta;
      x_old = x;
      y_old = y;

      // Re-enable interrupts
      attachInterrupt(digitalPinToInterrupt(D2), pulseCounter, RISING);
      attachInterrupt(digitalPinToInterrupt(D8), pulseCounter1, RISING);
    }

}
