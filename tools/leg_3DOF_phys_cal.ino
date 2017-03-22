/*
physical callibration code
default servo library pulse range is 544 to 2400 for 0 - 180 deg
*/


#include <Servo.h>

#define BUTTON 2
#define COXIA 0
#define FEMUR 1
#define TIBIA 2


//create servo objects for each lamp joint

const int numServos = 3; //how many servos do you have?
const int sPin[numServos] = {3, 5, 6}; //what pins do they correlate to?, 5, 6
Servo legServo[numServos]; //servo array
String joint[numServos] = {"Coxia", "Femur", "Tibia"}; //name of each servo 

int pb = 0;

// define Pots ADC Pins

int coxiaPot = A0;
int femurPot = A1;
int tibiaPot = A2;


int controlPot[numServos]; //control pot array
int potVal[numServos];    // variable to read the value from the analog pin
int sPos[numServos];
int pulse[numServos];

void setup()
{

	Serial.begin(19200); // initialize serial output
	Serial.println("it's on!");
	Serial.println("Press button to output current angle and pulse");
	pinMode (BUTTON, INPUT);
	for (int i = 0; i < 3; i++)
	{
		legServo[i].attach(sPin[i]);
	}
	

}

void loop()
{

	while(!pb)
	{

		for(int i = 0; i < numServos ; i++)
		{
			potVal[i] = analogRead(i);            // reads the value of the potentiometer (value between 0 and 1023)
			sPos[i] = map(potVal[i], 0, 1023, 0, 180);    // scale it to use it with the servo (value between 0 and 180)
			legServo[i].write(sPos[i]);                  // sets the servo position according to the scaled value
			delay(5);        // waits for the servo to get there
		}
		pb = digitalRead(BUTTON);
	}
	for (int x = 0; x < numServos; x++)
	
	{
		pulse[x] = map(sPos[x], 0, 180, 544, 2400);
		Serial.println(joint[x]);
		Serial.print("Pulse; ");
		Serial.print(pulse[x]);
		Serial.print("   Position: ");
		Serial.println(sPos[x]);
		

	}
	Serial.println();
	delay(100);
	pb = 0;
	
	
}
