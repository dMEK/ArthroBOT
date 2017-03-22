#define LEFT 1
#define RIGHT -1


#define BUTTON 2

int bodyZ = 90; //ideal body height



const int numServos = 3; //how many servos do you have?
int pb = 0;

String joint[numServos] = {"Coxia", "Femur", "Tibia"};

//L1 Front Left
int side;

int highPulse[numServos]; //pulse set for 180dg
int lowPulse[numServos]; //0 deg pulse

int np[3]; //xyz coordinates for neutral leg position

int realMaxA[numServos]; //angle on servo
int realMinA[numServos];


int maxA[numServos]; //angle for local coordinates
int minA[numServos];

#define ADDR 1


int leg = ADDR;

void setup()
{
	Serial.begin(19200); // initialize serial output
	Serial.println("Starting Test...");
	getParams();
	pinMode (BUTTON, INPUT);
}

void loop()
{
		Serial.println("Press button for sample Params...");





	Serial.println();
	while(!pb)
	{
		pb = digitalRead(BUTTON);
	}

	pb = 0;
	delay(200); //debounce
	Serial.print("Case: ");
	Serial.println(leg);
	Serial.println();
		for (int i = 0; i < numServos; i++)
			{
				
				Serial.println(joint[i]);
				Serial.println();
				
				Serial.print("highPulse  ");
				Serial.println(highPulse[i]); //pulse set for 180dg
				
				Serial.print("lowPulse  ");
				Serial.println(lowPulse[i]); //0 deg pulse

				Serial.print("np  ");
				Serial.println(np[i]); //xyz coordinates for neutral leg position

				Serial.print("realMaxA  ");
				Serial.println(realMaxA[i]); //angle on servo
				
				Serial.print("realMinA  ");
				Serial.println(realMinA[i]);


				Serial.print(" ");
				Serial.println(maxA[i]); //angle for local coordinates
				
				Serial.print(" ");
				Serial.println(minA[i]);

				
			}
	
	leg = random(1,7);
	getParams();
	
}


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
				{2150, 2263, 2306},
				{650, 571, 578},
				{12, 140, bodyZ},
				{125, 125, 19},
				{65, 5, 139},
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
				{2265, 2430, 2414},
				{645, 486, 578},
				{ -94, 110, bodyZ},
				{55, 46, 143},
				{105, 166, 23},
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

//L1 Front Left
const int side = LEFT;

int highPulse[numServos] = {2303, 2460, 2296}; //pulse set for 180dg
int lowPulse[numServos] = {611, 480, 568}; //0 deg pulse

int np[3] = {115,80,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {135, 170, 19}; //angle on servo
int realMinA[numServos] = {85, 50, 139};


int maxA[numServos] = {77, 180, 135}; //angle for local coordinates
int minA[numServos] = {27, 60, 15};

#define ADDR 1


//L2
const int side = LEFT;

int highPulse[numServos] = {2150, 2263, 2306}; //pulse set for 180dg
int lowPulse[numServos] = {650, 571, 578}; //0 deg pulse

int np[3] = {12,140,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {125, 125, 19}; //angle on servo
int realMinA[numServos] = {65, 5, 139};


int maxA[numServos] = {35, 180, 135}; //angle for local coordinates
int minA[numServos] = {-25, 60, 15};

#define ADDR 2


//L3
const int side = LEFT;

int highPulse[numServos] = {2600, 2331, 2290}; //pulse set for 180dg
int lowPulse[numServos] = {260, 571, 580}; //0 deg pulse

int np[3] = {-95,110,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {125, 125, 19}; //angle on servo
int realMinA[numServos] = {75, 5, 139};


int maxA[numServos] = {-12, 180, 135}; //angle for local coordinates
int minA[numServos] = {-62, 60, 15};

#define ADDR 3


//R1
const int side = RIGHT;

int highPulse[numServos] = {2313, 2423, 2361}; //pulse set for 180dg
int lowPulse[numServos] = {621, 515, 597}; //0 deg pulse


int np[3] = {115,80,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {35, 50, 144}; //angle on servo
int realMinA[numServos] = {85, 170, 24};


int maxA[numServos] = {77, 180, 135}; //angle for local coordinates
int minA[numServos] = {27, 60, 15};

#define ADDR 4


//R2
const int side = RIGHT;

int highPulse[numServos] = {2198, 2312, 2368}; //pulse set for 180dg
int lowPulse[numServos] = {488, 515, 586}; //0 deg pulse

int np[3] = {12,140,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {55, 39, 145}; //angle on servo
int realMinA[numServos] = {115, 159, 25};


int maxA[numServos] = {35, 180, 135}; //angle for local coordinates
int minA[numServos] = {-25, 60, 15};

#define ADDR 5

//R3
const int side = RIGHT;

int highPulse[numServos] = {2265, 2430, 2414}; //pulse set for 180dg
int lowPulse[numServos] = {645, 486, 578}; //0 deg pulse

int np[3] = {-94,110,bodyZ}; //xyz coordinates for neutral leg position

int realMaxA[numServos] = {55, 46, 143}; //angle on servo
int realMinA[numServos] = {105, 166, 23};


int maxA[numServos] = {-12, 180, 135}; //angle for local coordinates
int minA[numServos] = {-62, 60, 15};

#define ADDR 6

*/

