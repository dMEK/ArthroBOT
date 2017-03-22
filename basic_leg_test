/*
Program outputs a range of position readings for a range of positions on
3 servos for a hexapod leg to a terminal
CoolTerm was used to save that data to a file for analysis
For 50 pulse positions, it outputs 50 raw and 50 smothed readings
The purpose is to analyse the noise and accuracy of the pos feedback
This code can be addapted for other servo configurations using pos feedback
by changing numServo, and the joint name array
Dillon MacEwan 13/8/16
*/



#include <Servo.h> //import servo library

#define BUTTON 2
#define COXIA 0
#define FEMUR 1
#define TIBIA 2

const int numServos = 3; //how many servos do you have?
const int sPin[numServos] = {3, 5, 6}; //what pins do they correlate to?
Servo servo[numServos]; //declare the servo array
String joint[numServos] = {"Coxia", "Femur", "Tibia"}; //name of each servo
//arrays for servo data - enter data from table in description
//L2
int highPulse[numServos] = {2374, 2169, 2262}; //pulse set for 180dg
int lowPulse[numServos] = {570, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1472, 1355, 1406}; //90 deg pulse
//int angleRange [numServos] = {182, 200, 191}; //actual movement range


//int here[numServos]; //angle moving from
//int there[numServos]; //angle moving to

//float x[numServos]; //angle converted to radians to derive cosine wave
//int angle; //angle derived from cosine function. sent to servo in loop

//boolean doneMove[numServos];





int pb = 0; //used to hold push button reading





void setup()
{

	Serial.begin(19200); // initialize serial output
	Serial.println("Press button to center joints...");

	pinMode (BUTTON, INPUT);

	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}
	pb = 0;
	delay(100); //debounce


	for (int i = 0; i < numServos; i++)
	{
		servo[i].attach(sPin[i]);
	}


	//this routine attaches the servos with set limits, and maps position to degrees

	for(int i = 0; i < numServos; i++)
	{

		

		servo[i].attach(sPin[i], lowPulse[i], highPulse[i]); //attach this servo
		servo[i].write(60);
		delay(500);
		servo[i].write(90);
		delay(500);
			servo[i].write(60);
		delay(500);
		servo[i].write(90);
		delay(500);

		Serial.print(i);
		Serial.print(joint[i]);
		Serial.print(" attached");
		Serial.println();
		



	}


}

void loop()
{
	


	for(int i = 0; i < numServos; i++)
	{

		

		
		//servo[i].write(60);
		delay(500);
		servo[i].write(90);
		delay(500);

		//Serial.print(i);
		//Serial.print(joint[i]);
		//Serial.print(" moved");
		//Serial.println();
		



	}
	

}


/*

this is the values for each leg assembly

//L1
int highPulse[numServos] = {2162, 2252, 2166}; //pulse set for 180dg
int lowPulse[numServos] = {550, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1356, 1396, 1358}; //90 deg pulse
//int angleRange [numServos] = {199, 191, 192}; //actual movement range

//L2
int highPulse[numServos] = {2374, 2169, 2262}; //pulse set for 180dg
int lowPulse[numServos] = {570, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1472, 1355, 1406}; //90 deg pulse
//int angleRange [numServos] = {182, 200, 191}; //actual movement range

//L3
int highPulse[numServos] = {2817, 2229, 2229}; //pulse set for 180dg
int lowPulse[numServos] = {630, 550, 550}; //0 deg pulse
int cenPulse[numServos] = {1724, 1390, 1390}; //90 deg pulse
//int angleRange [numServos] = {138, 193, 193}; //actual movement range

//R1
int highPulse[numServos] = {2184, 2323, 2160}; //pulse set for 180dg
int lowPulse[numServos] = {550, 560, 550}; //0 deg pulse
int cenPulse[numServos] = {1367, 1441, 1355}; //90 deg pulse
//int angleRange [numServos] = {197, 186, 195}; //actual movement range

//R2
int highPulse[numServos] = {2121, 2117, 2148}; //pulse set for 180dg
int lowPulse[numServos] = {550, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1335, 1328, 1349}; //90 deg pulse
//int angleRange [numServos] = {202,199,202}; //actual movement range

//R3
int highPulse[numServos] = {2175, 2152, 2183}; //pulse set for 180dg
int lowPulse[numServos] = {550, 550, 540}; //0 deg pulse
int cenPulse[numServos] = {1363, 1351, 1361}; //90 deg pulse
//int angleRange [numServos] = {187,189,188}; //actual movement range


*/







