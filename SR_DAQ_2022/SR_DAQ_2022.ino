/*
 * Solar racing prototype DAQ firmware
 * Oringinal code by Xiniy Nam w/ Modifications by William Winters and others
*/


#include <LiquidCrystal.h>
#include <Wire.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <NeoSWSerial.h>

//change wiring and pins to free up interupt pin
const int rs = 9, en = 8, d4 = 5, d5 = 4, d6 = 3, d7 = 6,   //define pin numbers for LCD
          rows = 20, columns = 4,                           //define LCD dimensions
          solar_in = 0, batt_in = 1, motor_in = 2,          //define pin numbers for analog inputs
          solar_I_in = 3, batt_I_in = 4, motor_I_in = 5,    //define pin numbers for analog inputs
          ADC_Vref = 5, ADC_bit_res = 10,                   //initialize values for ADC
          tach_int = 2;                                     // def pin for interupts
const byte RF_address[6] = "00001";
//char text[];
float Solar_Volts = 0, Batt_Volts = 0, Motor_Volts = 0,           //declare vars to store voltage conversion results
      Solar_I_Volts = 0, Batt_I_Volts = 0, Motor_I_Volts = 0,     //declare vars to store voltage conversion results
      Batt_Percent = 0,                                     //declare vars to store processed data
      I_batt = 0, I_motor = 0, I_solar = 0,                 //  ''
      local_time_hr = 0;                                    //time in hours
float latitude_deg, longitude_deg, boat_speed_knots;        // S and W is neg
LiquidCrystal screen(rs, en, d4, d5, d6, d7);
RF24 transmitter(7,10);                                      //CE and CSN
int RXPin = 6, TXPin = 7,                                   //RX and TX pins on Arduino with GPS
    GPSBaud = 9600,                                         //GPS baud rate
    date_day = 1, date_month = 1, date_year = 2000,         //date variables
     RPM = 0, T_last_reading=0;                             //Tachometer Variables
volatile int revs = 0;                                      //interupt variable
TinyGPSPlus gps;
NeoSWSerial gpsSerial(RXPin, TXPin);


    
   /* while (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS detected: check wiring."));*/


void read_voltage() {
  //read analog inputs through ADC and convert back to voltage
  Solar_Volts = ((analogRead(solar_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));                                
  Batt_Volts = ((analogRead(batt_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));
  Motor_Volts = ((analogRead(motor_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));
  Batt_I_Volts = ((analogRead(batt_I_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));
  Solar_I_Volts = ((analogRead(solar_I_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));
  Motor_I_Volts = ((analogRead(motor_I_in) * ADC_Vref) / (pow(2,ADC_bit_res) - 1));
  
  
  //for testing accuracy of voltage readings
   
  Serial.println('A' + String(solar_in) + ": " + String(Solar_Volts));
  Serial.println('A' + String(batt_in) + ": " + String(Batt_Volts) );
  Serial.println('A' + String(motor_in) + ": " + String(Motor_Volts));
  Serial.println('A' + String(solar_I_in) + ": " + String(Solar_I_Volts));
  Serial.println('A' + String(batt_I_in) + ": " + String(Batt_I_Volts));
  Serial.println('A' + String(motor_I_in) + ": " + String(Motor_I_Volts));
  
  
}

void calculate_voltage_power() {
  //convert analog input back to voltage prior to steppping down
  Batt_Volts *= 38.126;                                     
  Batt_Volts /= 5.086;                                      
  //Serial.println('A' + String(batt_in) + ": " + String(Batt_Volts));

  // these constants need to be set manually
  Motor_Volts *= 38.604;                                       // multiply by total resistance
  Motor_Volts /= 5.093;                                        // divide by 'ground' resistance
  
  Solar_Volts *= 38.604;                                       // multiply by total resistance
  Solar_Volts /= 5.093;                                        // divide by 'ground' resistance
  
  Batt_I_Volts *= 13.63;                                       // multiply by total resistance
  Batt_I_Volts /= 3.59;                                        // divide by 'ground' resistance
  
  Motor_I_Volts *= 38.604;                                       // multiply by total resistance
  Motor_I_Volts /= 5.093;                                        // divide by 'ground' resistance
  
  Solar_I_Volts *= 38.604;                                       // multiply by total resistance
  Solar_I_Volts /= 5.093;                                        // divide by 'ground' resistance

  
  

  /*
   * calculate recorded parameters from raw voltages
  */
  Batt_Percent = ((Batt_Volts - 32.7) / (38.7 - 32.7)) * 100;
  if (Batt_Percent < 0) 
    Batt_Percent = 0; 

  //these constants need to be set manually
  I_batt = 2 * (Batt_I_Volts - 6.55) * 121.95 * 0.2;             // N coils * (V sensed - V zero current ) * gain (Amps / Volt)
  I_motor = 1 * (Motor_I_Volts - 6.0) * 0.7;                     // N coils * (V sensed - V zero current ) * gain (Amps / Volt)
  I_solar = 1 * (Solar_I_Volts - 6.0) * 0.7;                     // N coils * (V sensed - V zero current ) * gain (Amps / Volt)


  Serial.println('A' + String(solar_in) + "p: " + String(Solar_Volts));
  Serial.println('A' + String(batt_in) + "p: " + String(Batt_Volts) );
  Serial.println('A' + String(motor_in) + "p: " + String(Motor_Volts));
  Serial.println('A' + String(solar_I_in) + "p: " + String(Solar_I_Volts) + " current: " + String(I_solar));
  Serial.println('A' + String(batt_I_in) + "p: " + String(Batt_I_Volts,5) + " current: " + String(I_batt,5));
  Serial.println('A' + String(motor_I_in) + "p: " + String(Motor_I_Volts) + " current: " + String(I_motor));
}

void update_screen() {
  screen.clear();                                                                          //reset cursor to top left and clear screen
  screen.print("Batt: " + String(Batt_Volts, 2) + "V  " + String(I_batt, 2) + "A");  //print Battery charge percent
  screen.setCursor(0,1);                                                                   //move cursor to next line
  screen.print("Sol: " + String(Batt_Volts, 2) + "V " + String(I_solar, 2) + "A");                                              
  screen.setCursor(0,2);                                                                   
  screen.print("Motor: " + String(Motor_Volts,2) + "V " + String(I_motor, 2) + "A");
  screen.setCursor(0,3);
  screen.print("RPM: " + String(RPM));
  //print power consumed by load
  //screen.setCursor(0,3);                                                                   //move cursor to next line
  //screen.print("Speed: " + String(boat_speed_knots, 2) + " knots");                        //print boat speed in knots
}
/*
void transmit_data() {
  transmitter.write(&Batt_Volts, sizeof(Batt_Volts));
  //Serial.println("Batt_Volts Transmitted");
  transmitter.write(&P_Load, sizeof(P_Load));
  //Serial.println("P_Load Transmitted");
  transmitter.write(&P_Charge, sizeof(P_Charge));
  //Serial.println("P_Charge Transmitted");
  //transmitter.write(&boat_speed_knots, sizeof(boat_speed_knots));
  //Serial.println("Boat speed Transmitted");
}
*/
void tach_ISR(){                                            // runs on tach interupt
  revs++;
  
  /* 
   *  Either add debounce delay here or use capacitor. If using delay, may need to rewrite main loop, bc   
   *  timers don't update in ISRs
  */
}

void get_RPM(){                                                 // assumes delay in loop
  int T_elapsed = millis() - T_last_reading; 
  
  RPM =  revs * 30000.0 / (float) T_elapsed;                 // calculate RPM
  Serial.println("RPM: " + String(RPM) + " Revs: " + String(revs));
  revs = 0;                                                     // reset count
  T_last_reading = millis();                                    // reset time
}

//void update_GPS(){
//  if(gpsSerial.available() > 0){
//
//      //Serial.println(gps.charsProcessed()); //– the total number of characters received by the object
//      //Serial.println(gps.sentencesWithFix()); //– the number of $GPRMC or $GPGGA sentences that had a fix
//      //Serial.println(gps.failedChecksum()); //– the number of sentences of all types that failed the checksum test
//      //Serial.println(gps.passedChecksum()); //– the number of sentences of all types that passed the checksum test
//    if(gps.encode(gpsSerial.read())){
//      //read latitude and longitude
//      if(gps.location.isValid()){    
//        latitude_deg = gps.location.lat();
//        longitude_deg = gps.location.lng();
//        /*
//        Serial.print("Latitude: ");
//        Serial.println(latitude_deg);
//        Serial.print("Longitude: ");
//        Serial.println(longitude_deg); */
//      }
//
//      //read boat speed in knots
//      if (gps.speed.isUpdated()) {
//        boat_speed_knots = gps.speed.knots();
//        /* Serial.print("Boat Speed: ");
//        Serial.print(boat_speed_knots);
//        Serial.println(" knots"); */
//      }
//      
//      if(gps.date.isUpdated()){ 
//        date_month = gps.date.month();
//        date_day = gps.date.day();
//        date_year = gps.date.year();
//        
//        /*Serial.print(date_month);
//        Serial.print('/');
//        Serial.print(date_day);
//        Serial.print('/');
//        Serial.println(date_year);   */
//      }
//    
//      if(gps.time.isUpdated()){
//        local_time_hr = gps.time.hour() + (gps.time.minute()/60);
//        
//        //Serial.print("Local Time: ");
//        //Serial.println(local_time_hr);
//      }
//    }
//      }
//}



void setup() {
  //pinMode(7, OUTPUT);
  //pinMode(9, OUTPUT);
  //gpsSerial.begin(GPSBaud);
  screen.begin(rows,columns);                               //set LCD dimensions
  
  //transmitter.begin();
  //transmitter.openWritingPipe(RF_address);
  //transmitter.stopListening();

  attachInterrupt(digitalPinToInterrupt(tach_int),tach_ISR,FALLING);     //set up Interupt on pin 2 for tachometer
  
  Serial.begin(9600); //used for debugging 
}

void loop() {
  read_voltage();
  calculate_voltage_power();
  //update_GPS();
  get_RPM();
  update_screen();
  
  //transmit_data();
  delay(5000);                                               //delay for 5 sec in ms
}
