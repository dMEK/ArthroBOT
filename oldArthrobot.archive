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
#include <Wire.h>

//#include <ros.h> 		//ros libraries
//#include <std_msgs/String.h>

#define BUTTON 2

#define	M1 7	//Mode input bits from Motor Cortex
#define M2 8
#define M3 9
#define M4 10

#define COXIA 0 //servos array
#define FEMUR 1
#define TIBIA 2

#define ALPHA 1 //angle array
#define BETA 2
#define GAMA 0

#define X 0 //aep/pep axis array
#define Y 1
#define Z 2

#define LEFT 1
#define RIGHT -1

#define FORWARD 1
#define REVERSE -1

#define CL 28.0 //joint lengths
#define FL 86.0
#define TL 156.0

#define	EXCITE 11 //input and output from other legs
#define EXCITER 12

unsigned long time;

//IK variables
float L, L1;
float alpha, alpha1, alpha2, beta, gama;
const float dg = 180 / PI; //Radians >> Degrees
const float rd = PI / 180; //Degrees >> Radians


float a2; // float for direction angle trig

const int numServos = 3; //how many servos do you have?
const int sPin[numServos] = {3, 5, 6}; //what pins do they correlate to?
Servo servo[numServos]; //declare the servo array
String joint[numServos] = {"Coxia", "Femur", "Tibia"}; //name of each servo

int bodyZ = 72; //ideal body height

//arrays for servo data - enter data from table in description

//R3
const int side = RIGHT;

int highPulse[numServos] = {2175, 2152, 2183}; //pulse set for 180dg
int lowPulse[numServos] = {550, 550, 540}; //0 deg pulse
int cenPulse[numServos] = {1363, 1351, 1361}; //90 deg pulse
//int angleRange [numServos] = {187,189,188}; //actual movement range


int np[3] = {-110,155,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {66, 46, 132}; //angle on servo
int realMinA[numServos] = {116, 176, 22};


int maxA[numServos] = {-10, 180, 135}; //angle for local coordinates
int minA[numServos] = {-60, 50, 25};

#define Addr 6

int aep[3]; //xyz coordinates for aep
int pep[3]; //xyz coordinates for pep
//int cp[3]; //current position xyz
int tp[3]; //target position xyz
int ca[numServos]; //current angle
int ta[numServos]; //target angle

int travelAngle = 0; // variables for omnidirectional motion
int strokeL = 100; //length of leg stroke
int steps; // variable for how many IK targets are used in a given path
int lift = bodyZ * 2 / 5; //height foot steps in mm
int pathR = 3; //increment between IK points in leg path in mm
/*duty cycle as a percentage 80 = tetra gait 50 = tripod gait
the duty cycle is the propotion of time the leg spends stroke to swing
*/
int duty = 80;

int dir = FORWARD; //FORWARD = 1 REVERSE = -1





int pb = 0; //used to hold push button reading





void setup()
{

	Serial.begin(19200); // initialize serial output
	Serial.println("Press button to center joints...");
	
	Wire.begin(Addr); //initiate Wire, with leg address
	Wire.onReceive(receiveEvent); // register event
	Wire.onRequest(requestEvent);

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
	steps = strokeL / pathR;

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

	//set foot to neutral
	getAEP_PEP(strokeL, travelAngle);
	posFoot(np[X], np[Y], np[Z]);
	delay(1000);
	posFoot(aep[X], aep[Y], aep[Z]);
	delay(1000);
	posFoot(pep[X], pep[Y], pep[Z]);
	delay(1000);
	posFoot(np[X], np[Y], np[Z]);
	delay(1000);






}

void loop()
{

	Serial.println("Press button for single leg walk routine...");


	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}
	pb = 0;
	delay(100); //debounce

	

	steps = strokeL / pathR;

	getAEP_PEP(strokeL, travelAngle);

	swingToAEP();

	delay(500);




	
	
	delay(200);

	stride();

	delay(200);

	swing();

	delay(200);

	stride();

	delay(200);

	swing();

	delay(200);

	stride();

	delay(200);
	swing();
	delay(150);
	stride();
	delay(150);
	swing();
	delay(150);
	stride();
	delay(150);
	swing();


	delay(150);
	stride();


	delay(150);

	tp[X] = random((np[X] - 60), (np[X] + 60));
	tp[Y] = random((np[Y] - 60), (np[Y] + 60));
	posFoot(tp[X], tp[Y], np[Z]);
	delay(200);





}

//FUNCTIONS


//posFoot moved the foot position to the desired x,y,z coordinates
// by retreiving the servo angles using IKtrig() then maping to the
//true servo angle values

void posFoot(int x, int y, int z)
{
	tp[X] = x;
	tp[Y] = y;
	tp[Z] = z;
	IKtrig(x, y, z); //retrieve servo positions
	for(int i = 0; i < numServos; i++)
	{
		mapWriteServo(i); //map and write true servo positions
	}

}

/*
Swing moves the leg from AEP to PEP arcing through a sinosoidal path
to a height determined by the variable lift

*/

void swing()
{
	//int dx = (cos(travelAngle * rd) * strokeL) / steps;
	//int dy = (sin(travelAngle * rd) * strokeL) / steps;
	float dx = (aep[X] - pep[X])/float(steps);
	float dy = (aep[Y] - pep[Y])/float(steps);
	float sinLift;
	Serial.print("AEP x, y");
	Serial.print(aep[X]);
	Serial.print(", ");
	Serial.println(aep[Y]);
	Serial.print("PEP x, y");
	Serial.print(pep[X]);
	Serial.print(", ");
	Serial.println(pep[Y]);
	Serial.print("Steps: ");
	Serial.println(steps);
	Serial.print("dx: ");
	Serial.print(dx);
	Serial.print("  dy: ");
	Serial.println(dy);
	
	
	for(int j = 0; j < 3; j++)
	{
		tp[j] = pep[j];
		
		
	}
	posFoot(tp[X], tp[Y], tp[Z]);
	delay(10);
	
	Serial.print("Target x, y: ");
	Serial.print(tp[X]);
	Serial.print(", ");
	Serial.println(tp[Y]);



	for(int i = 0; i < steps; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);

		sinLift =  sin(PI * (float(i + 1) / steps)) * lift;

		tp[X] += dx ;
		tp[Y] += dy ;


		tp[Z] = np[Z] - sinLift ;

		delay(3);

	}
	Serial.print("Target x, y: ");
	Serial.print(tp[X]);
	Serial.print(", ");
	Serial.println(tp[Y]);

}

// swingToAEP lifts and places foot at AEP from any position

void swingToAEP()
{
	long delX = aep[X] - tp[X];
	long delY = aep[Y] - tp[Y];
	float strk = hypot(delX, delY);
	int stp = strk / pathR;
	int dx = delX / stp;
	int dy = delY / stp;

	for(int i = 0; i < stp; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);

		tp[X] += dx ;
		tp[Y] += dy ;
		tp[Z] = np[Z] - (sin(PI * (float(i) / steps)) * lift);
		delay(10);
	}


	posFoot(aep[X], aep[Y], aep[Z]);
	delay(20);

}

//stride algorithm:  from aep to pep, straight line trajectory through np

void stride()
{
	//int dx = (cos(travelAngle * rd) * strokeL) / steps;
	//int dy = (sin(travelAngle * rd) * strokeL) / steps;
	float dx = (aep[X] - pep[X])/float(steps);
	float dy = (aep[Y] - pep[Y])/float(steps);
	
	Serial.print("AEP x, y");
	Serial.print(aep[X]);
	Serial.print(", ");
	Serial.println(aep[Y]);
	Serial.print("PEP x, y");
	Serial.print(pep[X]);
	Serial.print(", ");
	Serial.println(pep[Y]);
	Serial.print("Steps: ");
	Serial.println(steps);
	Serial.print("dx: ");
	Serial.print(dx);
	Serial.print("  dy: ");
	Serial.println(dy);
	
	
	tp[X] = aep[X];
	tp[Y] = aep[Y];
	
	Serial.print("Target x, y: ");
	Serial.print(tp[X]);
	Serial.print(", ");
	Serial.println(tp[Y]);
	
	for(int i = 0; i < steps; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);


		tp[X] -= dx ;
		tp[Y] -= dy ;
		delay(3);
	}
	
	Serial.print("Target x, y: ");
	Serial.print(tp[X]);
	Serial.print(", ");
	Serial.println(tp[Y]);
}


// function converts IK angles to real angles and writes them to servo

int mapWriteServo (int s)//s = servo
{
	int x = map(ta[s], minA[s], maxA[s], realMinA[s], realMaxA[s]);
	servo[s].write(x);
}


//IK maths

void IKtrig(int x, int y, int z)
{


	float x1 = x;
	float y1 = y;
	float z1 = z;
	L1 = hypot(x1, y1);

	gama = atan2(x1, y1) * dg;

	L = hypot((L1 - CL), z1);

	beta = acos((sq(TL) + sq(FL) - sq(L)) / ( 2 * TL * FL)) * dg;

	alpha1 = acos(z1 / L) * dg;
	alpha2 = acos((sq(FL) + sq(L) - sq(TL)) / (2 * FL * L)) * dg;
	alpha = alpha1 + alpha2;

	ta[ALPHA] = alpha;
	ta[BETA] = beta;
	ta[GAMA] = gama;



}


//function retrieves AEP and PEP x and y values fom direction angle

void getAEP_PEP(int l, int a) //l - length of stroke - a - angle of direction in degrees
{
	a2 = a * rd; // convert a to Radians for trig
	aep[X] = np[X] + dir * (0.5 * l * cos(a2));
	aep[Y] = np[Y] + dir * (side * 0.5 * l * sin(a2));
	aep[Z] = np[Z];
	pep[X] = np[X] - dir * (0.5 * l * cos(a2));
	pep[Y] = np[Y] - dir * (side * 0.5 * l * sin(a2));
	pep[Z] = np[Z];
}


//Communication Functions

void receiveEvent(int HowMany)
{
	
	
}

void requestEvent() 
{
	
	
}

/*

this is the values for each leg assembly
//swap both coxia and tibia real max/min values

//L1 Front Left
const int side = LEFT;

int highPulse[numServos] = {2162, 2252, 2166}; //pulse set for 180dg
int lowPulse[numServos] = {550, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1356, 1396, 1358}; //90 deg pulse
//int angleRange [numServos] = {199, 191, 192}; //actual movement range

int np[3] = {110,155,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {103, 137, 44}; //angle on servo
int realMinA[numServos] = {53, 7, 154};


int maxA[numServos] = {60, 180, 135}; //angle for local coordinates
int minA[numServos] = {10, 50, 25};

#define Addr 1


//L2
const int side = LEFT;

int highPulse[numServos] = {2374, 2169, 2262}; //pulse set for 180dg
int lowPulse[numServos] = {570, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1472, 1355, 1406}; //90 deg pulse
//int angleRange [numServos] = {182, 200, 191}; //actual movement range

int np[3] = {-20,180,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {117, 135, 49}; //angle on servo
int realMinA[numServos] = {67, 5, 159};


int maxA[numServos] = {25, 180, 135}; //angle for local coordinates
int minA[numServos] = {-25, 50, 25};

#define Addr 2


//L3
const int side = LEFT;

int highPulse[numServos] = {2817, 2229, 2229}; //pulse set for 180dg
int lowPulse[numServos] = {630, 550, 550}; //0 deg pulse
int cenPulse[numServos] = {1724, 1390, 1390}; //90 deg pulse
//int angleRange [numServos] = {138, 193, 193}; //actual movement range

int np[3] = {-110,155,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {108, 130, 47}; //angle on servo
int realMinA[numServos] = {58, 0, 157};


int maxA[numServos] = {-10, 180, 135}; //angle for local coordinates
int minA[numServos] = {-60, 50, 25};

#define Addr 3


//R1
const int side = RIGHT;

int highPulse[numServos] = {2184, 2323, 2160}; //pulse set for 180dg
int lowPulse[numServos] = {550, 560, 550}; //0 deg pulse
int cenPulse[numServos] = {1367, 1441, 1355}; //90 deg pulse
//int angleRange [numServos] = {197, 186, 195}; //actual movement range


int np[3] = {150,140,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {66, 49, 133}; //angle on servo
int realMinA[numServos] = {116, 179, 23};


int maxA[numServos] = {60, 180, 135}; //angle for local coordinates
int minA[numServos] = {10, 50, 25};

#define Addr 4


//R2
const int side = RIGHT;

int highPulse[numServos] = {2121, 2117, 2148}; //pulse set for 180dg
int lowPulse[numServos] = {550, 540, 550}; //0 deg pulse
int cenPulse[numServos] = {1335, 1328, 1349}; //90 deg pulse
//int angleRange [numServos] = {202,199,202}; //actual movement range


int np[3] = {-20,180,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {60, 48, 136}; //angle on servo
int realMinA[numServos] = {110, 178, 26};


int maxA[numServos] = {25, 180, 135}; //angle for local coordinates
int minA[numServos] = {-25, 50, 25};

#define Addr 5

//R3
const int side = RIGHT;

int highPulse[numServos] = {2175, 2152, 2183}; //pulse set for 180dg
int lowPulse[numServos] = {550, 550, 540}; //0 deg pulse
int cenPulse[numServos] = {1363, 1351, 1361}; //90 deg pulse
//int angleRange [numServos] = {187,189,188}; //actual movement range


int np[3] = {-110,155,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {66, 46, 132}; //angle on servo
int realMinA[numServos] = {116, 176, 22};


int maxA[numServos] = {-10, 180, 135}; //angle for local coordinates
int minA[numServos] = {-60, 50, 25};

#define Addr 6

//this following sequence is the original test for foot positions

	getAEP_PEP(strokeL, travelAngle);

	Serial.println();
	Serial.print("NP Coordinates: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(np[i]);
		Serial.print(", ");
	}

	IKtrig(np[X], np[Y], np[Z]);
	Serial.println();
	Serial.print("NP Joint Angles: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(joint[i]);
		Serial.print(": ");
		Serial.print(ta[i]);
		Serial.print(", ");
		mapWriteServo(i);
	}
	delay(500);

	Serial.println();
	Serial.print("AEP Coordinates: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(aep[i]);
		Serial.print(", ");
	}

	IKtrig(aep[X], aep[Y], aep[Z]);
	Serial.println();
	Serial.print("AEP Joint Angles: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(joint[i]);
		Serial.print(": ");
		Serial.print(ta[i]);
		Serial.print(", ");
		mapWriteServo(i);
	}
	delay(500);




	Serial.println();
	Serial.print("PEP Coordinates: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(pep[i]);
		Serial.print(", ");
	}
	IKtrig(pep[X], pep[Y], pep[Z]);
	Serial.println();
	Serial.print("PEP Joint Angles: ");
	for(int i = 0; i < 3 ; i++)
	{
		Serial.print(joint[i]);
		Serial.print(": ");
		Serial.print(ta[i]);
		Serial.print(", ");
		mapWriteServo(i);
	}
	delay(2500);

//end of foot position test


*/







