//written by Adam Li and Gillie Agmon February 9th
#include <PID.h> //calls PID libraries and functions
#define ThermistorPIN A0 //defining Thermistor Pin on Arduino code
#define cw 4 //INA on motor driver
#define ccw 5 //INB on motor driver
#define out_pwm 3 //pwm for motor driver
#define temp1 80 //first step at 95C of 30 seconds
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
PID_params par ={80,0,0,0,0,0,10,5,0,95}; //NM:First array cell is the temp setpoint
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

/*
 * A function to change the current output in analogWrite.
 * Function Prototype: void changeCurrent(int temp, longtime1)
 */
void changeCurrent(int temp, long time1){ //function for calling PID
  pid.setInput(temp); //input into PID alg
  pid.process(time1); //calls PID calculation 
  curSpeed = pid.getOutput(); //curspeed gets output #
  if(curSpeed>0){
		heat();
		if(curSpeed>255){ 
    	curSpeed = 255;
  	}
	}
	if(curSpeed<0){
		cool();
		if(curSpeed<-255){
    	curSpeed = 255;
  	}
		curSpeed *= -1;
	}
  analogWrite(out_pwm, curSpeed); 
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
   	}
	}

/*
 * A function to change temperature to temp goal based on new setpoint, and delay time
 * Function Prototype: void changeCurrent(int temp, longtime1)
 */
void reachTemp(int setPoint, long delayHold){ // by default flag = 0 is cooling, 1 is heating
	pid.setSetPoint(setPoint); //sets the set point for PID calc
	int currentTemp = Thermistor (analogRead(ThermistorPIN));
	int tempgoal = setPoint; //sets our goal as the setpoint temperature
	int flag = currentTemp<tempgoal; //flag is a 0 if cooling, and 1 if heating
	long PIDtime;
        if(flag == 1){ //flag = 1 says we want to heat to our temp goal
		while(currentTemp<tempgoal){ //loops PID until reaches temp goal
			currentTemp= Thermistor (analogRead(ThermistorPIN));
  		long PIDtime = millis();
  		changeCurrent(currentTemp, PIDtime); //passes temp and time to change pwm current delivery based on PID alg
			}
		}
	else if(flag == 0){ //flag determines that we want to cool to our temperature goal
		while(currentTemp>tempgoal){ //loops PID until reaches 3rd temp goal
    	currentTemp = Thermistor (analogRead(ThermistorPIN));
    	PIDtime = millis();
    	changeCurrent(currentTemp, PIDtime);
  		}
		}
	unsigned long runningtime = millis(); //get current running time of program
	unsigned long desiredTime = runningtime + delayHold; //running time + 30 second hold
	
	while(runningtime<desiredTime){ //keep temperature at temp goal until delay is reached
  	runningtime = millis();
  	currentTemp= Thermistor (analogRead(ThermistorPIN));
  	PIDtime = millis();
  	changeCurrent(currentTemp, PIDtime);
		}
	Serial.print(" temp reached");
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


/*
 * Switches direction of Motor Driver to heating
 * Function Prototype: void heat() 
 */
void heat(){ //code for heating
  digitalWrite(cw, HIGH); //H is heating, Wiring = A to red to black to B
  digitalWrite (ccw, LOW);
}

/*
 * Switches direction of Motor Driver to cooling
 * Function Prototype: void cool()
 */
void cool (){ //code for cooling
  digitalWrite(cw, LOW); //H is heating, Wiring = A to red to black to B
  digitalWrite (ccw, HIGH);
}

/*
 * Sets up the Arduino Interface (pinMode, Serial Printing, Graph)
 * Function Prototype: void setup()
 */
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

/*
 * Loop Code for Arduino: This main loop is run constantly.
 * Function Prototype: void loop()
 */
void loop()
{
Serial.print ("hi");
int temp = Thermistor (analogRead(ThermistorPIN)); //reads temperature from Thermistor function
long cyclenum = 0; //keeps track of the number of cycles done (e.g. from 95 to 72
long tempgoal = temp1; //sets temperature goal to 95C
long PIDtime; //keeps track of running time to pass dt to PID alg

reachTemp(temp1, delay1);
cyclenum++; //counts cycle numbers 

reachTemp(temp2, delay2);
cyclenum++; //counts cycle numbers 

reachTemp(temp3, delay3);
cyclenum++; //counts cycle numbers 

}
