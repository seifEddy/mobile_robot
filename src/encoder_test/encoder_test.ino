// Define the pins for encoder phase A and B
const int encoderPinA = 2;
const int encoderPinB = 3;

// Variable to store the encoder count
volatile long encoderCount = 0;
volatile long lastEncoderCount = 0; // Variable to store the last encoder count for velocity calculation

// Previous state of encoder A
volatile byte lastEncoded = 0;

// Variable to store the last time we calculated velocity
unsigned long lastVelocityCalculation = 0;

// Function prototypes
void updateEncoder();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set the encoder pins as inputs
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach the interrupt function to the encoder pins
  // The interrupts are called on any change of pin state (RISING, FALLING or CHANGE)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // Calculate velocity every 100ms
  if (millis() - lastVelocityCalculation >= 100) {
    // Calculate the change in counts since the last calculation
    long countChange = encoderCount - lastEncoderCount;

    // Calculate the time elapsed in seconds
    float timeElapsed = (millis() - lastVelocityCalculation) / 1000.0;

    // Calculate the velocity
    float velocity = countChange / timeElapsed;

    // Print the velocity to the Serial Monitor
    Serial.print("Velocity: ");
    Serial.print(velocity);
    Serial.println(" counts/s");

    // Store the current encoder count and time for the next calculation
    lastEncoderCount = encoderCount;
    lastVelocityCalculation = millis();
  }
}

void updateEncoder() {
  // Read the current state of encoder pins
  int MSB = digitalRead(encoderPinA); // Most significant bit (MSB) - A phase
  int LSB = digitalRead(encoderPinB); // Least significant bit (LSB) - B phase

  // Convert the 2 pin state to single number
  int encoded = (MSB << 1) | LSB; // Convert the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; // Add the current state to the previous state

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

  lastEncoded = encoded; // Store this state as the last state for next position change
}
