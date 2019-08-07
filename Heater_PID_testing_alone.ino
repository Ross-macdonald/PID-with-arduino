//Setup thermocouple
#include "max6675.h"
int thermoDO = 7;
int thermoCS = 5;
int thermoCLK = 6;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
int vccPin = 3;
int gndPin = 2;
int heatpin = 11;

//Command Strings
String LVcommand;
String Heater_command;
String Thermo_command;
char Type;
char Heater_on = '0';
char Heater_temp_array[4];
String Heater_temp;
char thermo_enabled;
char thermo_unit;
char thermoyesno;

//variables
float set_temperature = 0;
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, TimePrev;
float PID_value = 0;

//PID constants
///////////////////////////////////////
int kp = 1; int ki = 1; int kd = 160;
///////////////////////////////////////
int PID_p = 0; int PID_i = 0; int PID_d = 0;
//////////////////////////////////////

void setup() {
    Serial.begin(9600);
    pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
    pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
    pinMode(heatpin, OUTPUT);
    delay(500);
    Time = millis(); //Initial time measurment. Used for calculating the 1st d value
}

void loop() {
  while(Serial.available()){
    LVcommand = Serial.readString(); // command line from port
  }
  if((LVcommand.length() != 0) && (LVcommand.length() != 1)){ //if a command is recived 
    Type = LVcommand.charAt(0); // clasifies the command
    if(Type == 'H'){ // H for heater
      Heater_command = LVcommand; 
      Heater_on = LVcommand.charAt(1);
      Heater_temp_array[0] = LVcommand.charAt(2);
      Heater_temp_array[1] = LVcommand.charAt(3);
      Heater_temp_array[2] = LVcommand.charAt(4);
      Heater_temp_array[4] = 0; //splits the command string up into it's important parts 
      Heater_temp = String(Heater_temp_array);
      set_temperature = Heater_temp.toInt(); // keeps the set temp around after LVcommand is cleared
      //Serial.println(set_temperature);
    }
    else if(Type == 'T'){ //just used for displaying the temprature to the port(easy debug) 
      Thermo_command = LVcommand;
      thermoyesno = Thermo_command.charAt(1);
      thermo_unit = Thermo_command.charAt(2);
      if (thermoyesno == '1'){
      //C = 1 F = 0
      if (thermo_unit == '1'){
        //Serial.println("C = ");
        Serial.println(thermocouple.readCelsius());
      }
      else if (thermo_unit == '0'){ //Why did i bother with Fahrenheit its a unit for people who use pounds per square inch (i.e. idiots)
        //Serial.println("F = ");
        Serial.println(thermocouple.readFahrenheit());
      }
      }
      }
      
  
  LVcommand = ""; //reset the command string
  }
  
  if(Heater_on == '1'){ // if the heater is turned on
  //Serial.println("here");
  //Serial.println(thermocouple.readCelsius());
  temperature_read = float(thermocouple.readCelsius()); //real measured temprature is take
  PID_error = set_temperature - temperature_read - 3; //the diffrence bettween the desired temperature and the real one with an error range of 3 degrees
  PID_p = 0.01*kp*PID_error; // convert the diffrence to the preportional term by multiplying by a constant
  PID_i = 0.01*PID_i +(ki*PID_error); //Integral term is the summ of the diffrences multiplied by a constant
  //D
  TimePrev = Time;
  Time = millis();
  elapsedTime = (Time - TimePrev)/1000; //find the time taken since last measurment and convert to seconds
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime); //Differential term is the rate of temp change multiplied by a constant
  PID_value = PID_p + PID_i + PID_d; //Final value is the 3 added together 
  previous_error = PID_error;
  Serial.println(thermocouple.readCelsius());
  
  
  //converting to power
  if(PID_value < 0){ //Make sure to limit the outputs to be within the range of possibility
    PID_value = 0;
    
  }
  else if (PID_value > 255){
    PID_value = 255;
  }
  //generating power
  Serial.println(PID_value);
  analogWrite(heatpin, PID_value); //turns on with a duty cycle = to the pid value
  
  }
  else{ //if the heater isn't on, turn it off
    analogWrite(heatpin,0);
    set_temperature = 0; //reset everything
    PID_p = 0;
    PID_d = 0;
    PID_i = 0;
    PID_value = 0;
    
  }
  delay(500);
}
