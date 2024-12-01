//------------------------------------------------------------
//Name: Line finder digital mode
//Function: detect black line or white line
//Parameter:   When digital signal is HIGH, black line
//             When digital signal is LOW, white line
//-------------------------------------------------------------

int signalPin = 3;  // connected to digital pin 3

void setup() {
  pinMode(signalPin, INPUT);  // initialize the digital pin as an output:
  Serial.begin(115200);         // initialize serial communications at 9600 bps:
}

void loop() {
  if (HIGH == digitalRead(signalPin))
    Serial.println("black");
  else Serial.println("white");  // display the color
  delay(10);                    // wait for a second
}