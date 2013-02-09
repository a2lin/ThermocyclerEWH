//written by Adam Li and Gillie Agmon February 9th
#include <PID.h> //calls PID libraries and functions
#define ThermistorPIN A0 //defining Thermistor Pin on Arduino code
#define cw 4 //INA on motor driver
#define ccw 5 //INB on motor driver
#define out_pwm 3 //pwm for motor driver
#define temp1 95 //first step at 95C of 30 seconds
#define temp2 50 //second step at 50C of 30 seconds
#define temp3 72 //third step at 72C of 1 minute and hold for 10 minutes at end
#define temp4 4 //final temperature at 4C to hold
#define cycles 105 // 3 times number of cyles bec each change of temp = one cycle
#define delay1 30000 //length of time at temperature 1
#define delay2 30000 //length of time at temperature 2
#define delay3 60000 //lenght of time at temperature 3

//// Temperature Printing Method
const int D=1; // 0 for printing temperature, 1 to serial output graph
int row = 0; //for serial output
int del =500;// delay for serial output
long timed; // time for serial output

////PID algorithm section
PID_params par ={95,0,0,0,0,0,6,5,5,95}; //NM:First array cell is the temp setpoint
/*struct PID_params {
	double set;
	double input;
	double output;
	double accumulated;
	double previous;
	double error;

	double proportional; (6)
	double integral;
	double derivative;

	double accLimit;*/
PID pid = PID(&par);//enables PID functions
long curSpeed = 0;//defines current speed
void changeCurrent(int temp, long time1){ //function for calling PID
  pid.setInput(temp); //input into PID alg
  pid.process(time1); //calls PID calculation 
  curSpeed = pid.getOutput(); //curspeed gets output #
  if(curSpeed>255){ 
    curSpeed = 255;
  }
  else if(curSpeed<-255){
    curSpeed = 255;
  }
  analogWrite(out_pwm, curSpeed);
   if (D==1){ //set D=1 if serial output graph is wanted
    if ((millis()-timed)>del){ //makes data be recorded only in increments equal to "del"
    Serial.print("DATA,TIME,"); Serial.println(temp); //puts data in format the PLX-DAQ can interpret
    row++;
    timed=millis();
    }
}
}


//// Thermistor to Temperature Conversion
float pad = 10000; //thermistor resistance circuit
float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.
  Resistance=((1024 * pad / RawADC) - pad); 
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius
  return Temp;  
}



void heat(){ //code for heating
  digitalWrite(cw, HIGH); //H is heating, Wiring = A to red to black to B
  digitalWrite (ccw, LOW);
}
void cool (){ //code for cooling
  digitalWrite(cw, LOW); //H is heating, Wiring = A to red to black to B
  digitalWrite (ccw, HIGH);
}
void setup() //setup of Pins, and print data
{
  pinMode (cw, OUTPUT);
  pinMode (ccw, OUTPUT);
  pinMode (out_pwm, OUTPUT);
  Serial.begin (9600);
  Serial.println("CLEARDATA"); //for serial output PLX-DAQ
  Serial.println("LABEL,Time,Temperautre(C)"); //for serial output PLX-DAQ, labels columns
  timed=millis(); //used in serial output graph code
}
void loop()
{
  Serial.print ("hi");
float temp = Thermistor (analogRead(ThermistorPIN)); //reads temperature from Thermistor function
////Temperature Printing Methods
  if (D==0){ //set D=0 if only temperature is wanted
    Serial.print("Celsius: "); 
    Serial.print(temp,1);   // display Celsius
    Serial.println("");
  }
  if (D==1){ //set D=1 if serial output graph is wanted
    if ((millis()-timed)>del){ //makes data be recorded only in increments equal to "del"
    Serial.print("DATA,TIME,"); Serial.println(temp); //puts data in format the PLX-DAQ can interpret
    row++;
    timed=millis();
    }
    
////PID controlling current methods   
heat(); //sets direction for heating to 95C
long cyclenum = 0; //keeps track of the number of cycles done (e.g. from 95 to 72
long tempgoal = temp1; //sets temperature goal to 95C
long PIDtime; //keeps track of running time to pass dt to PID alg

while(temp<tempgoal){ //loops PID until reaches first temp goal
////Temperature Printing Methods
Serial.println ("Loop");
Serial.println (timed);
Serial.println (millis());
float temp = Thermistor (analogRead(ThermistorPIN)); //reads temperature from Thermistor function
  if (D==0){ //set D=0 if only temperature is wanted
    Serial.print("Celsius: "); 
    Serial.print(temp,1);   // display Celsius
    Serial.println("");
  }
  if (D==1){ //set D=1 if serial output graph is wanted
    if ((millis()-timed)>del){ //makes data be recorded only in increments equal to "del"
    Serial.print("DATA,TIME,"); Serial.println(temp); //puts data in format the PLX-DAQ can interpret
    row++;
    timed=millis();
    }
  temp= Thermistor (analogRead(ThermistorPIN));
  PIDtime = millis();
  changeCurrent(temp, PIDtime); //passes temp and time to change pwm current delivery based on PID alg
}
//once temp goal is reached, we will maintain the temperature at said temp goal
unsigned long runningtime = millis(); //get current running time of program
unsigned long desiredTime = runningtime + delay1; //running time + 30 second hold
while(runningtime<desiredTime){//run for 30 seconds
  runningtime = millis();
  temp= Thermistor (analogRead(ThermistorPIN));
  PIDtime = millis();
  changeCurrent(temp, PIDtime);
}
Serial.print(" temp reached");

pid.setSetPoint(temp2);//sets the new setpoint at 50C
tempgoal= temp2; //tempgoal is now 50C
cyclenum++; //counts cycle numbers 
cool(); //changes direction of current delivery to Peltier to cool down samples
while(temp>tempgoal){ //loops PID until reaches 2nd temp goal
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    changeCurrent(temp, PIDtime);
  }
  runningtime = millis();
  desiredTime=runningtime+delay2;
  heat();
while(runningtime<desiredTime){//run for 30 seconds
  runningtime = millis();
  temp= Thermistor (analogRead(ThermistorPIN));
  PIDtime = millis();
  changeCurrent(temp, PIDtime);
  }
  Serial.print(" temp reached");
  
 tempgoal= temp3; //tempgoal is now 3rd temp goal
 cyclenum++; 
 heat();
while(temp<tempgoal){ //loops PID until reaches 3rd temp goal
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    changeCurrent(temp, PIDtime);
  }
  runningtime = millis();
  desiredTime=runningtime+delay3;
while(runningtime<desiredTime){//run for 30 seconds
  runningtime = millis();
  temp= Thermistor (analogRead(ThermistorPIN));
  PIDtime = millis();
  changeCurrent(temp, PIDtime);
  }
   Serial.print(" temp reached");
   
}
  }
}
