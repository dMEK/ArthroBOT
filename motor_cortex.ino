/* this is the main controller for realtime response from the sonsors and to
the legs of the Hexapod
Inputs include IMU data, distance sensors for obstacles and edges
data from the legs
navigation data from the "Brain"
Outputs include: - navigation status, direction, walk duty cycle,
angle of travel etc. some of these will be digital, analog or I2C data
Motor Cortex is an Arduino MEga 2560:

12PWM
16ADC
32 Digital I/O
1 I2C pair
3 RX/TX pair

Nano I2C:
A4 SDA
A5 SCL

Mega I2C:
20 SDA
21 SCL

modules that this unit needs to run will include I2C, IMU, Ping, and possibly ROS

*/

#include <Wire.h>
#include <NewPing.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(14, 15); //TX, RX respetively


//#include <ros.h> 		//ros libraries
//#include <std_msgs/String.h>

#define X 0				//for cartesian coordinate arrays
#define Y 1
#define Z 2

#define ULT1ECHO		//these are the digital pins for the Ultrasonic sensors
#define ULT1TRIG
#define ULT2ECHO
#define ULT2TRIG
#define ULT3ECHO
#define ULT3TRIG

#define IR_RANGE		//analog pin for Sharp IR Rangefinder

#define ACC_X			//analog pins for Accelerometer
#define ACC_Y
#define ACC_Z

#define ACC_SL			//digital pinout accelerometer low = sleep
#define ACC_0G			//digital pinin 0G (high = freefall)
#define ACC_GS			//digital pinout sensitivity high = 6G low = 1.5G

#define L1	0			//Array adresses for leg controllers
#define L2  1
#define L3	2
#define R1	3
#define R2	4
#define R3	5

#define COXIA 0			//Array addresses for leg param matrix
#define FEMUR 1
#define TIBIA 2

#define H_PULSE 0
#define L_PULSE 1
#define C_PULSE 2

#define NP 0
#define R_MAX_A 1
#define R_MIN_A 2
#define MAX_A 3
#define MIN_A 5

#define I2C_SDA	20		//I2C bus communication pins
#define I2C_SCL	21


#define M0	40			//Walk Mode Bit Pins
#define M1	41
#define	M2	42
#define M3	43
#define RESET	53		//reset signal

#define FORWARD 1		//signal values for direction and walk
#define REVERSE 0
#define WALK 1
#define STANCE 0

#define LEGS 6



int Addr[6] = {1, 2, 3, 4, 5, 6};			//I2C addresses for legs
int LegRec[6] = {22 , 23 , 24 , 25 , 26 , 27};	//signal pins for data from legs

int Diff = 0;
int Dir = FORWARD;
int Angle = 0;
int Mode = STANCE;



int legData [6][4]; //matrix for {X Y Z direction} data from legs

unsigned long time;

void setup()
{
	Wire.begin(0);//set master Ard with address 0
	//send callibration information to each leg?
	setLegParams();
	//check start up status
}

void loop()
{
	checkLegData();
	//check for obsacles or step (Ultrasonic)
	//check pose (accelerometer)
	//if obstacle , adjust action and send
	//check for status pins
	//adjust parameters and send

}



//This function checks the data availability from the legs, then enters the data
//into the approriate matrix position
void checkLegData ()
{
	//this is the routine to check for available data from the legs
	for (int i = 0; i < LEGS; i++)
	{
		bool a = digitalRead(LegRec[i]);
		if (a = true)
		{
			Wire.requestFrom(i, 6);    // request 6 bytes from slave device #8
			int j = 0;

			while (Wire.available())   // slave may send less than requested
			{

				legData[i][j] = Wire.read(); // receive a byte, enter it to matrix
				j++;

			}
		}
	}
}


void setLegParams ()
{
	//This stores the Leg Paramaters to the Flash Memory to be sent to each leg
	//wire will only send 32 bytes in one session
	
	//16 bit "words" - 18 bytes to be sent to each leg
	const PROGMEM  uint16_t legPulseParams[6][3][3] =
	{
		//L1
		{
			{2162, 2252, 2166}, 	//highPulse - pulse set for 180dg
			{550, 540, 550},		//lowPulse - 0 deg pulse
			{1356, 1396, 1358},		//cenPulse - 90 deg pulse
		},
		//L2
		{
			{2374, 2169, 2262},
			{570, 540, 550},
			{1472, 1355, 1406},
		},
		//L3
		{
			{2817, 2229, 2229},
			{630, 550, 550},
			{1724, 1390, 1390},
		},
		//R1
		{
			{2184, 2323, 2160},
			{550, 560, 550},
			{1367, 1441, 1355},
		},
		//R2
		{
			{2121, 2117, 2148},
			{550, 540, 550},
			{1335, 1328, 1349},
		},
		//R3
		{
			{2175, 2152, 2183},
			{550, 550, 540},
			{1363, 1351, 1361},
		}
	};

	
	//byte sized chunks - 15 bytes to be sent
	const PROGMEM  uint8_t legParams[6][5][3] =
	{
		//L1
		{
			{87, 124, 72},			//xyz coordinates for neutral leg position
			{103, 137, 44},			//realMaxA - angle on servo
			{53, 7, 154},			//realMinA
			{60, 180, 135},			//maxA -  angle for local coordinates
			{10, 50, 25}			//minA
		},
		//L2
		{
			{0, 151, 72},
			{117, 135, 49},
			{67, 5, 159},
			{25, 180, 135},
			{ -25, 50, 25}
		},
		//L3
		{
			{ -87, 124, 72},
			{58, 130, 157},
			{108, 0, 47},
			{ -10, 180, 135},
			{ -60, 50, 25}
		},
		//R1
		{
			{87, 124, 72},
			{66, 49, 133},
			{116, 179, 23},
			{60, 180, 135},
			{10, 50, 25}
		},
		//R2
		{
			{0, 151, 72},
			{60, 48, 136},
			{110, 178, 26},
			{25, 180, 135},
			{ -25, 50, 25}
		},
		//R3
		{
			{ -87, 124, 72},
			{66, 46, 132},
			{116, 176, 22},
			{ -10, 180, 135},
			{ -60, 50, 25}
		}
	};
	
}


