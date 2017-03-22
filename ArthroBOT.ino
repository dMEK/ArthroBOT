/*

*/



#include <Servo.h> //import servo library
#include <Wire.h>
#include <PinChangeInt.h>

//#include <ros.h>     //ros libraries
//#include <std_msgs/String.h>

#define BUTTON 2

#define COR_IN 7  //Signal pins for Motor Cortex
#define COR_OUT 8

#define LAT_IN 9 	//Signal Pins between legs
#define LAT_OUT 10
#define FOR_IN 11
#define FOR_OUT 12

#define BUMP 13

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

#define CL 29.0 //joint lengths
#define FL 84.0
#define TL 152.0


unsigned long time;

//IK variables
float L, L1;
float alpha, alpha1, alpha2, beta, gama;
const float dg = 180 / PI; //Radians >> Degrees
const float rd = PI / 180; //Degrees >> Radians


float a2; // float for direction angle trig
float dx;
float dy;

const int numServos = 3; //how many servos do you have?
const int sPin[numServos] = {3, 5, 6}; //what pins do they correlate to?
Servo servo[numServos]; //declare the servo array
String joint[numServos] = {"Coxia", "Femur", "Tibia"}; //name of each servo

int bodyZ = 92; //ideal body height
int lastZ = bodyZ;

//arrays for servo data - enter data from table in description
int side;

int highPulse[numServos]; //pulse set for 180dg
int lowPulse[numServos]; //0 deg pulse

int np[3]; //xyz coordinates for neutral leg position

int realMaxA[numServos]; //angle on servo
int realMinA[numServos];


int maxA[numServos]; //angle for local coordinates
int minA[numServos];

#define ADDR 2


int leg = ADDR;



int aep[3]; //xyz coordinates for aep
int pep[3]; //xyz coordinates for pep

int tp[3]; //target position xyz

int ta[numServos]; //target angle

int travelAngle = 0; // variables for omnidirectional motion
int strokeL = 100; //length of leg stroke
int setStride = 100; //stores an ideal forward motion strude length

int steps = 30; // variable for how many IK targets are used in a given path
int lift = 30; //height foot steps in mm
int halfS = steps / 2;
int mode = 1;

int pathR = 3; //increment between IK points in leg path in mm
/*duty cycle as a value between 0 and 40 - 0 is trpod, 40 is tetra gait
*/
int duty = 40;
int IKdelay = 10;
int acker = 0;
int stOut = 0;

int dir = FORWARD; //FORWARD = 1 REVERSE = -1

bool walking = false; //states for walk()
bool striding = false;
bool ackman = false;
bool run = false;


int pb = 0; //used to hold push button reading
int cycles = 0;




void setup()
{

	getParams();
	Serial.begin(19200); // initialize serial output
	Serial.println("Press button to center joints...");

	//Wire.begin(ADDR); //initiate Wire, with leg address
	//Wire.onReceive(receiveEvent); // register event
	//Wire.onRequest(requestEvent);

	pinMode (BUTTON, INPUT);
	pinMode (LAT_IN, INPUT);
	pinMode (FOR_IN, INPUT);
	pinMode (COR_IN, INPUT);
	pinMode (BUMP, INPUT);
	pinMode (LAT_OUT, OUTPUT);
	pinMode (FOR_OUT, OUTPUT);
	pinMode (COR_OUT, OUTPUT);

	if (leg != 6)
	{
		PCintPort::attachInterrupt(FOR_IN, &reflex, FALLING);
		PCintPort::attachInterrupt(LAT_IN, &reflex, FALLING);
	}
	PCintPort::attachInterrupt(BUTTON, &halt, FALLING);

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


		Serial.print(i);
		Serial.print(joint[i]);
		Serial.print(" attached");
		Serial.println();




	}

	//set foot to neutral
	getAEP_PEP(strokeL, travelAngle);
//	Serial.println("go to np");
	posFoot(np[X], np[Y], np[Z]);

	delay(500);






}

void loop()
{

//	Serial.println("Press button for single leg walk routine...");





	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200); //debounce
//	Serial.println("Walk forward, 18 steps");
//	Serial.println();

	travelAngle = 0;
	getAEP_PEP(strokeL, travelAngle);
	cycles = -2;

	walk();

//		Serial.println("Press button for single leg walk routine...");





//	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200);

//	Serial.println("turn left 20");
	mode = 2;
	walk();
//		Serial.println("Press button for single leg walk routine...");





//	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200);

//	Serial.println("turn right 10 steps");
	mode = 3;
	walk();
	mode = 1;
	travelAngle = 0;

	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200);

	turn(LEFT, 8);


	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200);

	turn(RIGHT, 8);





}

//FUNCTIONS

void walk()
{
	duty = 40;
	int dutyInc = 2;
	int angleInc = 3;
	getAEP_PEP(strokeL, travelAngle);
	dx = (aep[X] - pep[X]) / float(steps);
	dy = (aep[Y] - pep[Y]) / float(steps);
	run = false;
	walking = true;
	swingTo(np[X], np[Y]);

	if (mode == 4)
	{
		duty = 20;
	}


	switch(leg)
	{
		case 1: //L1
		{
			swingTo(aep[X], aep[Y]);
			delay(50);
			stride();
		}
		break;
		case 2: //L2
		{
			swingTo(aep[X], aep[Y]);
			delay(100);
			stride();
		}
		break;
		case 3: //L3
		{
			delay(100);
			stride();
		}
		break;
		case 4: //R1
		{
			delay(100);
			stride();
		}
		break;
		case 5: //R2
		{
			delay(100);
			stride();
		}
		break;
		case 6: //R3
		{

//			swingTo(aep[X], aep[Y]);
//			delay(100);
//			swingTo(np[X], np[Y]);

			swingTo(pep[X], pep[Y]);
			delay(200);


			swingTo(aep[X], aep[Y]);
			delay(100);
			run = true;
			stride();

		}
		break;
	}

	while (walking)
	{

		if ((mode < 3) && (acker != 0))
		{
			ackman = true;
			ackerman();
		}
		if ((ackman == true) && (acker == 0))
		{
			ackman = false;
			travelAngle = 0;
			strokeL = setStride;
		}

		getAEP_PEP(strokeL, travelAngle);
		run = true;
		swingTo(aep[X], aep[Y]);


		if (walking) //in case a halt is called in swing
		{
			stride();
		}
		if (mode == 2)
		{
			duty -= dutyInc;
			if (duty <= 3)
			{
				dutyInc = -2;
			}
			else if (duty >= 37)
			{
				dutyInc = 2;
			}
		}
		if (mode == 3)
		{
			duty = 20;
			travelAngle -= angleInc;
			if (travelAngle <= -45)
			{
				angleInc = -2;
			}
			else if (travelAngle >= 45)
			{
				angleInc = 2;
			}
		}
		cycles--;
		if (cycles < -2)
		{
			cycles = -2;
		}

		else if (cycles == 0)
		{
			walking = false;
		}
	}
	run = false;
	delay(leg * 100);
	swingTo(np[X], np[Y]);
	delay(100);

}


void turn(int dr, int cyc) //direction, duty(speed), steps
{
//	if (du > 40)
//	{
//		du = 40;
//	}
//	duty = du;
	mode = 4;
	cycles = cyc;

	switch(leg)
	{
		case 1: //L1
		{

			travelAngle = -48;
			dir = dr * (-1);
		}
		break;
		case 2: //L2
		{
			travelAngle = 0;
			dir = dr * (-1);
		}
		break;
		case 3: //L3
		{

			travelAngle = 48;
			dir = dr * (-1);
		}
		break;
		case 4: //R1
		{

			travelAngle = 48;
			dir = dr;
		}
		break;
		case 5: //R2
		{
			travelAngle = 0;
			dir = dr;
		}
		break;
		case 6: //R3
		{

			travelAngle = -48;
			dir = dr;
			Serial.println(travelAngle);

		}
		break;
	}
	walk();
}

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



//swing along an eliptical path
void swing()
{

	float dx = (aep[X] - pep[X]) / float(steps);
	float dy = (aep[Y] - pep[Y]) / float(steps);
	float EllipseLift = 0;
	float cx = pep[X];
	float cy = pep[Y];
	float a;
//	Serial.println(steps);
//	Serial.println(halfS);


	for(int j = 0; j < 3; j++)
	{
		tp[j] = pep[j];


	}



	cx = tp[X];
	cy = tp[Y];

	for(int i = 0; i < steps; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);

		a = (halfS - float(i + 1)) / halfS;
		a = a * a;
//		Serial.println(i);
//		Serial.println(a);
		if (a > 1)
		{
			a = 1;
		}

		EllipseLift =  sqrt(1 - a) * lift;

		cx += dx;
		cy += dy;
		tp[X] = cx ;
		tp[Y] = cy ;


		tp[Z] = np[Z] - EllipseLift ;

//		Serial.println(tp[Z]);
//		Serial.println(EllipseLift);



		delay(IKdelay);

	}
	posFoot(tp[X], tp[Y], tp[Z]);
	delay(IKdelay);
}

// swingToAEP lifts and places foot at AEP from any position

void swingToAEP()
{
	long delX = aep[X] - tp[X];
	long delY = aep[Y] - tp[Y];
	float strk = hypot(delX, delY);
	int stp = strk / pathR;
	float dx = delX / float(stp);
	float dy = delY / float(stp);
	int hlfS = stp / 2;
	float EllipseLift = 0;
	float cx = tp[X];
	float cy = tp[Y];
	float a;

	for(int i = 0; i < stp; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);
		a = (hlfS - float(i + 1)) / hlfS;
		a = a * a;
		if (a > 1)
		{
			a = 1;
		}

		EllipseLift =  sqrt(1 - a) * lift;

		cx += dx ;
		cy += dy ;
		tp[X] = cx;
		tp[Y] = cy;
		tp[Z] = np[Z] - EllipseLift;
		delay(IKdelay);

	}


	posFoot(aep[X], aep[Y], aep[Z]);
	delay(IKdelay);

}

//This functions lifts and places the foot to a specified x y location at neutral foot height

void swingTo(int x, int y)
{
	long delX = x - tp[X];
	long delY = y - tp[Y];
	if (delX == 0)
	{
		delX = 1;
	}
	if (delY == 0)
	{
		delY = 1;
	}
	float strk = hypot(delX, delY);
	int stp = strk / pathR;
	dx = delX / float(stp);
	dy = delY / float(stp);
	int hlfS = stp / 2;
	float EllipseLift;
	float cx = tp[X];
	float cy = tp[Y];
	float a;
	int bump = 0;
	lift = 30;

	digitalWrite(FOR_OUT, HIGH);
	if (leg == 6)
	{
		digitalWrite(LAT_OUT, HIGH);
	}

	for(int i = 0; i < stp; i++)
	{
		posFoot(tp[X], tp[Y], tp[Z]);

		a = (hlfS - float(i + 1)) / hlfS;
		a = a * a;
		if (a > 1)
		{
			a = 1;
		}

		EllipseLift =  sqrt(1 - a) * lift;

		cx += dx ;
		cy += dy ;
		tp[X] = cx;
		tp[Y] = cy;
		tp[Z] = np[Z] - EllipseLift;
		delay(IKdelay);
		bump = digitalRead(BUMP);
		if (bump)
		{
			Serial.println("Bump!");
			stepUp();
		}

	}


	posFoot(x, y, np[Z]);
	delay(IKdelay);

}

//stride algorithm:  from aep to pep, straight line trajectory through np

void stride()
{

	dx = (aep[X] - pep[X]) / float(steps);
	dy = (aep[Y] - pep[Y]) / float(steps);

	float cx = aep[X]; //reduce rounding errors
	float cy = aep[Y];

	int del = IKdelay + duty; //duty cycle delay for striding

	int st = 0; //step count
	int ls = steps * duty / 100; // tmie for lateral signal from R3

	int lat = 0; //inputs for lateral and foraqwaed feedforward signal
	int fwd = 0;

	tp[X] = aep[X];
	tp[Y] = aep[Y];
	tp[Z] = lastZ;
	striding = true;


	while (striding)
	{
		st++; //count steps used

		posFoot(tp[X], tp[Y], tp[Z]);

		cx -= dx;
		cy -= dy;
		tp[X] = cx ;
		tp[Y] = cy ;
		delay(del);

		if (leg == 3)
		{
			fwd = digitalRead(LAT_IN);
		}
		else
		{
			fwd = digitalRead(FOR_IN);
		}



//		if (fwd == 1)
//		{
//			Serial.println("Forward Signal In!");
//		}


		if (run == true)
		{
			if (st >= 2)
			{

				digitalWrite(FOR_OUT, LOW);
			}


		}

		if (leg == 6) //driving leg R3
		{
			if (st >= ls)
			{
				Serial.println("Lateral Signal!");
				digitalWrite(LAT_OUT, LOW);
			}

			if (st >= steps)
			{
				striding = false;
			}
		}
		else if(st >= 26) //replace with ramping function
		{
			if ((st == 26) && (del < 29))
			{
				del = 29;
			}
			del++;
		}
		else if(st > 38)
		{
			dx = 0;
			dy = 0;

		}

		if ((st > steps) && (fwd == 0))
		{
			striding = false;
		}

//		if (lat || fwd)
//		{
//			striding = false;
//		}
	}
}

void ackerman()
{
	int radius; //radius of travel of outer centre foot
	int r;		//radius of travel of this foot
	int d;		//perpendicular offset of this foot
	int dr = 1; //+ or - direction of foot travel
	float b;	//holder for offset for maths
	float a;	//holder for trig angle
	int L = setStride; //outside centre leg stroke length
	radius = 480 + (abs(acker) * 20); // find desired turning raduis

	switch(leg)
	{
		case 1: //L1
		{
			if (acker > 0)
			{
				d = 328;
				dr = 1;
			}
			else
			{
				d = 85;
				dr = -1;
			}
		}
		break;
		case 2: //L2
		{
			if (acker > 0)
			{
				strokeL = L * (1 - (408 / radius));
				travelAngle = 0;
			}
			else
			{
				strokeL = setStride;
				travelAngle = 0;
			}
		}
		break;
		case 3: //L3
		{
			if (acker > 0)
			{
				d = 353;
				dr = -1;
			}
			else
			{
				d = 55;
				dr = 1;
			}
		}
		break;
		case 4: //R1
		{
			if (acker > 0)
			{
				d = 85;
				dr = 1;
			}
			else
			{
				d = 328;
				dr = -1;
			}
		}
		break;
		case 5: //R2
		{
			if (acker > 0)
			{
				strokeL = setStride;
				travelAngle = 0;
			}
			else
			{
				strokeL = L * (1 - (408 / radius));
				travelAngle = 0;
			}
		}
		break;
		case 6: //R3
		{
			if (acker > 0)
			{
				d = 55;
				dr = -1;
			}
			else
			{
				d = 353;
				dr = 1;
			}
		}
		break;

		if ((leg != 2) && (leg != 5)) // if not centre legs
		{
			b = radius - d;
			r = hypot(178, b);
			strokeL = L * r / radius;
			a = atan2 (178.0, b);
			travelAngle = a * dg * dr;
		}
	}
}

void stepUp()
{
	int exs;
	int wiy;
	int zed;

	zed = tp[Z] - 15;
	exs = tp[X];
	wiy = tp[Y];
	exs -= 6 * dx;
	wiy -= 6 * dy;

	posFoot(exs, wiy, zed);
	delay(IKdelay);
	delay(IKdelay);

	zed = zed - 5;
	exs += 7 * dx;
	wiy += 7 * dy;

	posFoot(exs, wiy, zed);

	delay(IKdelay);
	delay(IKdelay);

	tp[X] += 4 * dx;
	tp[Y] += 4 * dy;
	tp[Z] = zed;
	lift += 20;
}

void stepOut()
{
	int exs;
	int wiy;
	int zed;

	zed = tp[Z] - 15;
	exs = tp[X];
	wiy = tp[Y];
	exs -= 6 * dx;
	wiy -= 6 * dy;

	posFoot(exs, wiy, zed);
	delay(IKdelay);
	delay(IKdelay);

	zed = zed - 5;
	exs += 7 * dx;
	wiy += 7 * dy;

	posFoot(exs, wiy, zed);

	delay(IKdelay);
	delay(IKdelay);

	tp[X] += 4 * dx;
	tp[Y] += 4 * dy;
	tp[Z] = zed;
	lift += 20;
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

	for (int i = 0; i < numServos; i++)
	{
		if (ta[i] < minA[i])
		{
			ta[i] = minA[i];
		}
		if (ta[i] > maxA[i])
		{
			ta[i] = maxA[i];
		}
	}



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

//Interrupt Pin Functions

void reflex()
{
	Serial.println("Reflex!");
	striding = false;
}

void halt()
{
	if (walking == true)
	{
		setAll();
	}
}

void setFlags() // a quick function just to reset all walk flags
{
	bool walking = false; //states for walk()
	bool striding = false;
	bool ackman = false;
	bool run = false;
}

void setAll()
{
	setFlags();

	mode = 1;
	strokeL = setStride;
	travelAngle = 0;
	duty = 40;
	stOut = 0;
}

//Communication Functions

void receiveEvent(int HowMany)
{


}

void requestEvent()
{


}

//store parameters for each leg controller chip


void getParams()
{

	switch (leg)

	{
		case 1://L1 Front Left
		{
			side = LEFT;
			int Params [7][3] =
			{
				{2303, 2460, 2296},
				{611, 480, 568},
				{115, 80, bodyZ},
				{135, 170, 19},
				{85, 50, 139},
				{77, 180, 135},
				{27, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}

		}
		break;



		case 2://L2
		{
			side = LEFT;
			int Params [7][3] =
			{
				{2150, 2534, 2387},
				{650, 410, 605},
				{12, 140, bodyZ},
				{125, 160, 45},
				{65, 40, 165},
				{35, 180, 135},
				{ -25, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}
		}
		break;



		case 3://L3
		{
			side = LEFT;

			int Params [7][3] =
			{
				{2600, 2331, 2290},
				{260, 571, 580},
				{ -95, 110, bodyZ},
				{125, 125, 19},
				{75, 5, 139},
				{ -12, 180, 135},
				{ -62, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}

		}
		break;



		case 4://R1
		{
			side = RIGHT;

			int Params [7][3] =
			{
				{2313, 2423, 2361},
				{621, 515, 597},
				{115, 80, bodyZ},
				{35, 50, 144},
				{85, 170, 24},
				{77, 180, 135},
				{27, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}

		}
		break;



		case 5://R2
		{
			side = RIGHT;

			int Params [7][3] =
			{
				{2198, 2312, 2368},
				{488, 515, 586},
				{12, 140, bodyZ},
				{55, 39, 145},
				{115, 159, 25},
				{35, 180, 135},
				{ -25, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}

		}
		break;


		case 6://R3
		{
			side = RIGHT;

			int Params [7][3] =
			{
				{2077, 2430, 2414},
				{547, 486, 578},
				{ -94, 110, bodyZ},
				{15, 46, 143},
				{65, 166, 23},
				{ -12, 180, 135},
				{ -62, 60, 15}
			};

			for (int i = 0; i < numServos; i++)
			{
				highPulse[i] = Params[0][i]; //pulse set for 180dg
				lowPulse[i] = Params[1][i]; //0 deg pulse

				np[i] = Params[2][i]; //xyz coordinates for neutral leg position

				realMaxA[i] = Params[3][i]; //angle on servo
				realMinA[i] = Params[4][i];


				maxA[i] = Params[5][i]; //angle for local coordinates
				minA[i] = Params[6][i];


			}


		}
		break;


	}

}

/*

this is the values for each leg assembly
//swap both coxia and tibia real max/min values



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


/*
Swing moves the leg from AEP to PEP arcing through a sinosoidal path
to a height determined by the variable lift

*/

//void swing()
//{
//  //int dx = (cos(travelAngle * rd) * strokeL) / steps;
//  //int dy = (sin(travelAngle * rd) * strokeL) / steps;
//  float dx = (aep[X] - pep[X])/float(steps);
//  float dy = (aep[Y] - pep[Y])/float(steps);
//  float sinLift;
//  float cx;
//  float cy;
//
//  for(int j = 0; j < 3; j++)
//  {
//    tp[j] = pep[j];
//
//
//  }
//  posFoot(tp[X], tp[Y], tp[Z]);
//  //delay(10);
//
//  cx = tp[X];
//  cy = tp[Y];
//
//  for(int i = 0; i < steps; i++)
//  {
//    posFoot(tp[X], tp[Y], tp[Z]);
//
//    sinLift =  sin(PI * (float(i + 1) / steps)) * lift;
//
//    cx += dx;
//    cy += dy;
//    tp[X] = cx ;
//    tp[Y] = cy ;
//
//
//    tp[Z] = np[Z] - sinLift ;
//
//    //delay(3);
//
//  }
//
//
//}


