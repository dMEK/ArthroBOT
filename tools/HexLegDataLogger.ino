/*

Dillon MacEwan 13/8/16
*/


#include <Servo.h> //import servo library

const int numServos = 3; //how many servos do you have?
const int sPin[numServos] = {3, 5, 6}; //what pins do they correlate to?
Servo servo[numServos]; //declare the servo array
String joint[numServos] = {"Coxia", "Femur", "Tibia"}; //name of each servo




#define Button 2

int btwReadings = 20; //delay time between
int whlReading = 2; //delay time between analog readings of internal pot


  int pb = 0; //used to hold push button reading
  

  int pos;

  int pulse = 750; //first uS pulse used in range test



void setup()
{

  Serial.begin(19200); // initialize serial output
  Serial.println("it's on!");
  analogReference(EXTERNAL);
  pinMode (Button, INPUT);
  for (int i = 0; i < numServos; i++)
  {
    //pinMode (sPin[i], OUTPUT);
    servo[i].attach(sPin[i]);
  }


  
}

void loop()
{
   
    Serial.print("vRef 2.48V");
    Serial.println("Polling 40ms, no caps");
    Serial.println();
    for (int x = 0; x < numServos; x++)
   {
        Serial.print("Press button to set range of Servo: ");
        servo[x].writeMicroseconds(pulse); //send servo to start of range
        Serial.print(joint[x]);
        Serial.println();
        while(!pb)
        {
          pb = digitalRead(Button);
        }
        pb = 0;
        Serial.print("Recording raw position data in ..3");
        for (int i = 2; i >= 0; i--) //count down three seconds
        {
          delay(1000);
          Serial.print("..");
          Serial.print(i);
        }
        Serial.println();
        Serial.print(joint[x]);
        pulse = 750;
        delay(20);
        servo[x].writeMicroseconds(pulse); //send servo to start of range
        delay(2000); //wait for it to get there
         for (int i = 0; i < 50; i++)
         {
          servo[x].writeMicroseconds(pulse);
          Serial.println();
          Serial.print("Pulse:");
          Serial.print(pulse);
          
//          Serial.println();
//          Serial.print("Raw Data");
//          Serial.println();
//          for (int i = 0; i < 50; i++) // this loop just sends 50 CSV raw pos reading for each pulse increment
//          {
//
//              pos = analogRead(x);
//              Serial.print(pos);
//              Serial.print(",");
//        
//              delay(30);
//              
//          }
//          pos = analogRead(x);
//          Serial.print(pos);
          Serial.println();
          Serial.print("Smoothed Data");
          Serial.println();
          for (int i = 0; i < 50; i++) // this loop just sends 50 CSV raw pos reading for each pulse increment
          {

              pos = getFeedback(x);
              Serial.print(pos);
              Serial.print(",");

  
        
          }
          
          Serial.print(pos);
          pulse += 30;
          Serial.println();
       }
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
    for (j = 0; j < 20; j++)
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



