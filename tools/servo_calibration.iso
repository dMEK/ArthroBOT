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

const int numServos = 1; //how many servos do you have?
const int sPin[numServos] = {3}; //what pins do they correlate to?, 5, 6
Servo servo[numServos]; //declare the servo array
String joint[numServos] = {"Coxia"}; //name of each servo , "Femur", "Tibia"
//arrays for servo data
int highPulse[numServos];
int lowPulse[numServos];
int cenPulse[numServos];
int highPos[numServos];
int lowPos[numServos];
int cenPos[numServos];

int here[numServos]; //angle moving from
int there[numServos]; //angle moving to

float x[numServos]; //angle converted to radians to derive cosine wave
int angle; //angle derived from cosine function. sent to servo in loop
boolean doneMove[numServos];


int btwReadings = 20; //delay time between
int whlReading = 3; //delay time between analog readings of internal pot
int polT = whlReading * 20;

float vRef = 2.37;


int pb = 0; //used to hold push button reading
int pos;
int t;
int pulse = 1500; //first uS pulse used in range test

int feedBack;

void setup()
{

	Serial.begin(19200); // initialize serial output
	Serial.println("it's on!");
	analogReference(EXTERNAL);
	pinMode (BUTTON, INPUT);
	for (int i = 0; i < numServos; i++)
	{
		servo[i].attach(sPin[i]);
	}

	calibrate();


	//this routine attaches the servos with set limits, and maps position to degrees

	for(int i = 0; i < numServos; i++)
	{

		servo[i].writeMicroseconds(cenPulse[i]);

		feedBack = cenPos[i]; //take current reading from pot
		there[i] = map(feedBack, lowPos[i], highPos[i], 0, 180); //adjust feedback to degree output
		servo[i].attach(sPin[i], lowPulse[i], highPulse[i]); //attach this servo
		servo[i].write(there[i]); //send out pulse for where we are

		Serial.print(i);
		Serial.print(joint[i]);
		Serial.print(" attached");
		Serial.println();
		doneMove[i] = true;



	}


}

void loop()
{
	Serial.println("testing");
	moveTest();
	checkAngleRange();

}

void calibrate()
{
	Serial.print("vRef: ");
	Serial.print(vRef);
	Serial.println("V");
	Serial.print("polling time: ");
	Serial.print(polT);
	Serial.println("ms");
	for (int x = 0; x < numServos; x++)
	{
		Serial.print("Press button to calibrate Servo: ");
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		Serial.print(joint[x]);
		Serial.println();
		while(!pb)
		{
			pb = digitalRead(BUTTON);
		}
		pb = 0;
		Serial.print("Calibrating in ..3");
		for (int i = 2; i >= 0; i--) //count down three seconds
		{
			delay(1000);
			Serial.print("..");
			Serial.print(i);
		}

		// up range
		pulse = 1500;
		delay(20);
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		delay(2000); //wait for it to get there
		Serial.println();
		pos = getFeedback(x);
		pos -= 20;
		t = pos;

		Serial.print(joint[x]);
		pulse = 1500;
		delay(20);
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		delay(2000); //wait for it to get there


		pulse += 10;	//overcome the servos initial inertia before calibration
		servo[x].writeMicroseconds(pulse);



		Serial.println();
		Serial.print("High Pulse Range");
		Serial.println();
		do
		{

			t = pos;

			Serial.print(t);
			Serial.print(",");
			pulse += 15;
			servo[x].writeMicroseconds(pulse);
			delay(250);
			pos = getFeedback(x);
			Serial.print(pos);
			Serial.print(",");
			Serial.print(pulse);
			Serial.println();



		}
		while(pos > t);

		highPulse[x] = pulse - 20;
		highPos[x] = pos - 10;

		//down range

		pulse = 1500;
		delay(20);
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		delay(2000); //wait for it to get there
		Serial.println();
		pos = getFeedback(x);
		pos += 20;
		t = pos;

		Serial.print(joint[x]);
		pulse = 1500;
		delay(20);
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		delay(2000); //wait for it to get there

		pulse -= 10;	//overcome the servos initial inertia before calibration
		servo[x].writeMicroseconds(pulse);


		Serial.println();
		Serial.print("Low Pulse Range");
		Serial.println();
		do
		{

			t = pos;

			Serial.print(t);
			Serial.print(",");
			pulse -= 15;
			servo[x].writeMicroseconds(pulse);
			delay(200);
			pos = getFeedback(x);
			Serial.print(pos);
			Serial.print(",");
			Serial.print(pulse);
			Serial.println();


			
		}
		while(pos < t);

		lowPulse[x] = pulse + 20;
		lowPos[x] = pos + 10;
		cenPulse[x] = ((highPulse[x] - lowPulse[x]) / 2) + lowPulse[x];
		
		cenPos[x] = ((highPos[x] - lowPos[x]) / 2) + lowPos[x];
		Serial.print(lowPulse[x]);
		Serial.print(",");
		Serial.print(cenPulse[x]);
		Serial.print(",");
		Serial.print(highPulse[x]);
		Serial.println();
		Serial.print(lowPos[x]);
		Serial.print(",");
		Serial.print(cenPos[x]);
		Serial.print(",");
		Serial.print(highPos[x]);
		Serial.println();

	}
	//output results
	Serial.print("vRef: ");
	Serial.print(vRef);
	Serial.println("V");
	Serial.print("polling time: ");
	Serial.print(polT);
	Serial.println("ms");
	Serial.println();
	for (int x = 0; x < numServos; x++)
	{
		Serial.println(joint[x]);
		Serial.print("Pulse Range(us); ");
		Serial.print(lowPulse[x]);
		Serial.print(" <> ");
		Serial.println(highPulse[x]);
		Serial.print("Position Range(bits): ");
		Serial.print(lowPos[x]);
		Serial.print(" <> ");
		Serial.println(highPos[x]);
		Serial.println();

	}
}




void checkAngleRange()
{
	for (int x = 0; x < numServos; x++)
	{
		Serial.print("Press button to check angle range of Servo: ");
		servo[x].writeMicroseconds(pulse); //send servo to start of range
		Serial.print(joint[x]);
		Serial.println();
		while(!pb)
		{
			pb = digitalRead(BUTTON);
		}
		pb = 0;
		Serial.println("Low position");
		delay(1000);
		servo[x].writeMicroseconds(lowPulse[x]);
		delay(3000);
		Serial.println();
		Serial.println("Press button to set high position: ");
		while(!pb)
		{
			pb = digitalRead(BUTTON);
		}
		pb = 0;
		Serial.println("High position");
		delay(1000);
		servo[x].writeMicroseconds(highPulse[x]);
		delay(3000);
		Serial.println();
		Serial.println("Press button to set centre position: ");
		while(!pb)
		{
			pb = digitalRead(BUTTON);
		}
		pb = 0;
		Serial.println("Centre position");
		delay(1000);
		servo[x].writeMicroseconds(cenPulse[x]);
		delay(3000);

	}

}

void moveTest()
{
	Serial.print("Press button to start move test ");

	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}
	pb = 0;
	delay(100); //debounce
	Serial.println("Press button to stop move test ");
	Serial.println();
	while(!pb)
	{

		for (int i = 0; i < numServos; i++)
		{
			if(doneMove[i] == true)
			{
				doneMove[i] = false;
				here[i] = there[i];
				there[i] = random(180.1) + 0.5;
				if(there[i] == here[i])
				{
					there[i] = random(180.1) + 0.5;
				}
				if(here[i] < there[i])
				{
					x[i] = 0;
				}
				else
				{
					x[i] = 180;
				}
				Serial.print("Move servo ");
				Serial.print(i);
				Serial.print(" from ");
				Serial.print(here[i]);
				Serial.print(" to ");
				Serial.println(there[i]);
			}
		}
		//calcCos(current position, desired position, step, servo array position)
		for (int i = 0; i < numServos; i++)
		{
			angle = calcCos(here[i], there[i], 1.5, i);
			if (doneMove[i] == false)
			{
				servo[i].write(angle);
				delay(5);
			}
		}
		pb = digitalRead(BUTTON);
	}


}
/*
THIS FUNCTION READS THE INTERNAL SERVO POTENTIOMETER and smooths the data
*/
int getFeedback(int a)
{
	int j;
	int mean;
	int result;
	int test;
	int reading[20];
	boolean done;

	for (j = 0; j < 20; j++)
	{
		reading[j] = analogRead(a); //get raw data from servo potentiometer
		delay(whlReading);
	} // sort the readings low to high in array
	done = false; // clear sorting flag
	while(done != true)  // simple swap sort, sorts numbers from lowest to highest
	{
		done = true;
		for (j = 0; j < 19; j++)
		{
			if (reading[j] > reading[j + 1])  // sorting numbers here
			{
				test = reading[j + 1];
				reading [j + 1] = reading[j] ;
				reading[j] = test;
				done = false;
			}
		}
	}
	mean = 0;
	for (int k = 6; k < 14; k++) //discard the 6 highest and 6 lowest readings
	{
		mean += reading[k];
	}
	result = mean / 8; //average useful readings
	return(result);
}    // END GET FEEDBACK

/*
THIS FUNCTION CREATES SMOOTH (COSINE) MOVEMENT FROM HERE TO THERE
*/
//calcCos(current position, desired position, step, servo array position)
int calcCos(int h, int th, float s, int n)
{
	int r;
	int a;
	if(h < th)
	{
		x[n] += s;
		if (x[n] >= 181)
		{
			doneMove[n] = true;
		}
		r = (cos(radians(x[n])) * 100);
		a = map(r, 100, -100, h, t);
	}
	if(h > th)
	{
		x[n] -= s;
		if (x[n] <= -1)
		{
			doneMove[n] = true;
		}
		r = (cos(radians(x[n])) * 100);
		a = map(r, -100, 100, h, t);
	}
	return a;
}     //END CALC COS

