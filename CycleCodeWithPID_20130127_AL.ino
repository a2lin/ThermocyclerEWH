#include <PID.h>
#define ThermistorPIN A0 
#define cw 7 //pin for clockwise motion (peltier temperature control)
#define ccw 8 //pin for counterclockwise motion (Peltier temperature control)
#define out_pwm 6
#define temp1 95 //first temperature in cycle
#define temp2 50 //second temperature in cycle
#define temp3 72 // third temperature in cycle
#define temp4 4 //holding temperature
#define cycles 105 // 3 times number of cyles bec each change of temp = one cycle
#define delay1 30000 //length of time at temperature 1
#define delay2 30000 //length of time at temperature 2
#define delay3 60000 //lenght of time at temperature 3

PID_params par ={95,0,0,0,0,0,5,0,0,95}; //NM:First array cell is the temp setpoint
/*struct PID_params {
	double set;
	double input;
	double output;
	double accumulated;
	double previous;
	double error;
	double proportional;
	double integral;
	double derivative;
	double accLimit;*/
PID pid = PID(&par);
const int D=1; // 0 for printing temperature, 1 to serial output graph
// do we need this?  int row = 0; //for serial output
// do we need this? int del =500;// delay for serial output
// do we need this? long timed; // time for serial output

float pad = 10000; //thermistor circuit 

float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.
  Resistance=((1024 * pad / RawADC) - pad); //used to calculated temperature from RawADC
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp)); //equation that finds temperature for thermistor
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
void changeCurrent(int temp, long time1){ //function for calling PID
  pid.setInput(temp);
  pid.process(0.1);
  curSpeed = pid.getOutput();
  analogWrite(out_pwm, curSpeed);
  Serial.println(par.output);
}

void setup()
{
  pinMode (cw, OUTPUT);
  pinMode (ccw, OUTPUT);
  pinMode (out_pwm, OUTPUT);
  Serial.begin (9600);
  time=millis();
  Serial.println("CLEARDATA"); //for serial output PLX-DAQ
  Serial.println("LABEL,Time,Temperautre(C)"); //for serial output PLX-DAQ, labels columns
  timed=millis(); //used in serial output graph code
}

void loop()
{
  long cyclenum = 0; //sets cyclenum at 0
  long tempgoal = temp1; //sets the first temperature goal
  long PIDtime; //used to define dt for error calc.
  float temp;
  temp= Thermistor (analogRead(ThermistorPIN));
  
  //figure out a way to incorporate this printing stuff into the PID loops
  //also figure out if cool and heat functions need to be written in/out of while loop
  if (D==0)//set D=0 if only temperature is wanted
  { 
    Serial.print("Celsius: "); 
    Serial.print(temp,1);   // display Celsius
    Serial.println("");
  }
  if (D==1)//set D=1 if serial output graph is wanted
  { 
    if ((millis()-timed)>del){ //makes data be recorded only in increments equal to "del"
    Serial.print("DATA,TIME,"); Serial.println(temp); //puts data in format the PLX-DAQ can interpret
    row++;
    timed=millis();
    }
  }
  
  while(temp<tempgoal){ //loops PID until reaches first temp goal
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    heat();
    changeCurrent(temp, PIDtime);
  }
//once temp goal is reached, we will maintain the temperature at said temp goal
  unsigned long time = millis(); //get current running time of program
  unsigned long desiredTime = time + delay1; //running time + 30 second hold
  while(time<desiredTime){ //hold temp for 30 seconds at 95
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    changeCurrent(temp, PIDtime);
  }

  tempgoal= temp2; //tempgoal is now 2nd temp goal of 50
  cyclenum++; 
  while(temp>tempgoal){ //loops PID until reaches 2nd temp goal of 50
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    cool();
    changeCurrent(temp, PIDtime);
  }
  time = millis(); //get current running time of program
  desiredTime = time + delay2; //running time + 30 second hold
  while(time<desiredTime){ //hold temp at 50 for 30 seconds
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    changeCurrent(temp, PIDtime);
  }
  
  tempgoal= temp3; //tempgoal is now 3rd temp goal of 72
  cyclenum++; 
  while(temp>tempgoal){ //loops PID until reaches 3rd temp goal of 72
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    heat();
    changeCurrent(temp, PIDtime);
  }
  time = millis(); //get current running time of program
  desiredTime = time + delay3; //running time + 60 second hold
  while(time<desiredTime){ //hold temp at 72 for 60 seconds
    temp= Thermistor (analogRead(ThermistorPIN));
    PIDtime = millis();
    changeCurrent(temp, PIDtime);
  }
}

