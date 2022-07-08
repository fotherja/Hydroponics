/*
  Hydronponics control system:

  On startup the pump waits for 10 minutes (To ensure the growbed is drained)
  It then fills the resevoir tank until the float sensor is triggered.

  We then enter the main loop:
    1) We record a pre-pump level recording
    2) Switch on the pump to fill the grow bed
    3) When FILL_VOLUME is reached according to the pressure level sensor
    4) We then wait until the next fill cycle and repeat

  If a fault occurs we indicate this using a buzzer
*/

#include        <Wire.h>
#include        "Adafruit_MPRLS.h"
#include        "Average.h"
#include        "LowPower.h"

#define         FLOAT_NEGATIVE_PIN            2
#define         RESEVOIR_PUMP_PIN             3
#define         FLOAT_SENSOR_PIN              4
#define         PUMP_PIN                      6
#define         BUZZER_GND                    12
#define         LED_PIN                       13
#define         V_SENSE_PIN                   A1
#define         MPRLS_SDA                     A4
#define         MPRLS_CLK                     A5

#define         FAULT_HANDLE                  0
#define         RESEVOIR_PUMP_TIMEOUT_FAULT   1
#define         GROWBED_PUMP_TIMEOUT_FAULT    2
#define         MPRLS_FAULT                   3

#define         STARTUP_DELAY_TIME            600000                                 // Delay after reset                                           
#define         PUMP_TIMEOUT_TIME             220000                                 // We never run the pump for longer than this despite sensor readings
#define         TARGET_FILL_VOLUME            1300                                   // Amount of water to pump into growbed each cycle
 
#define         EBB_FLOW_PERIOD_60MIN         3600000        // 1 HOURLY

#define         PUMP_OFF                      0
#define         PUMP_RUNNING                  1

#define         FLOAT_SENSOR                  digitalRead(FLOAT_SENSOR_PIN)
#define         TURN_PUMP_ON                  digitalWrite(PUMP_PIN, HIGH)
#define         TURN_PUMP_OFF                 digitalWrite(PUMP_PIN, LOW)
#define         DEACTIVATED                   0

Adafruit_MPRLS  mpr = Adafruit_MPRLS(-1, -1);
Average         Pressure_Filter(20);

int Fill_Resevoir_Tank();
int Read_Water_Level();
void Fault(int Fault_Code);
unsigned long Calculate_Next_Fill_Time(float B_V);

//---------------------------------------------------------------
void setup() 
{                 
  Serial.begin(115200);
  delay(1000);
  
  pinMode(PUMP_PIN, OUTPUT); 
  digitalWrite(PUMP_PIN, LOW);   
  
  pinMode(RESEVOIR_PUMP_PIN, OUTPUT); 
  digitalWrite(RESEVOIR_PUMP_PIN, LOW); 
    
  pinMode(FLOAT_SENSOR_PIN, INPUT_PULLUP);
  
  pinMode(FLOAT_NEGATIVE_PIN, OUTPUT);   
  digitalWrite(FLOAT_NEGATIVE_PIN, LOW);   
  
  if (!mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    Fault(MPRLS_FAULT);
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
  Serial.println("Found MPRLS sensor");
}

//---------------------------------------------------------------
void loop() 
{
  static unsigned long    Pump_Toggle_Time        = millis() + STARTUP_DELAY_TIME; 
  static unsigned long    Display_Time            = 0;
  static int              Pre_Pump_Water_Volume   = 0;
  static byte             Pump_Status             = PUMP_OFF;

  Fault(FAULT_HANDLE);

  long Time_Remaining = Pump_Toggle_Time - millis();

  if(Time_Remaining <= 0)                                                       // If it's time to toggle the pump
  {     
    if(Pump_Status == PUMP_OFF)                                                 // If pump is currently off, turn it on for PUMP_TIMEOUT_TIME milliseconds
    {            
      Pre_Pump_Water_Volume = Fill_Resevoir_Tank();
      TURN_PUMP_ON;
      Pump_Status = PUMP_RUNNING;

      Pump_Toggle_Time = millis() + PUMP_TIMEOUT_TIME;
      Serial.print("Pump On. Pre-pump Pressure: "); Serial.println(Pre_Pump_Water_Volume);
    }     
    else                                                                        // Fault condition. We shouldn't have reached the PUMP_TIMEOUT_TIME
    {      
      TURN_PUMP_OFF;
      Pump_Status = PUMP_OFF;
      
      Serial.println("FAULT! Pump timeout exceeded");
      Fault(GROWBED_PUMP_TIMEOUT_FAULT);
      Pump_Toggle_Time = millis() + EBB_FLOW_PERIOD_60MIN;
    }
  }  

  int Resevoir_Water_Volume = Read_Water_Volume();
  int Pumped_Water_Volume = Pre_Pump_Water_Volume - Resevoir_Water_Volume;
  if(Pump_Status == PUMP_RUNNING and Pumped_Water_Volume > TARGET_FILL_VOLUME)  // We have pumped the target volume of water into the grow tank 
  {
    TURN_PUMP_OFF;
    Pump_Status = PUMP_OFF;
      
    Serial.println("Grow container filled. Turning pump off.");
    Pump_Toggle_Time = millis() + EBB_FLOW_PERIOD_60MIN;  
  }

  // -------
  
  long Next_Display_Time = Display_Time - millis();
  if(Next_Display_Time <= 0)
  {
    Display_Time = millis() + 1000;
        
    digitalWrite(LED_PIN, HIGH);
      
    Serial.print("Time: "); Serial.print(Time_Remaining / 1000); Serial.print(",");
    Serial.print(Resevoir_Water_Volume); Serial.print(","); Serial.println(Pumped_Water_Volume);

    digitalWrite(LED_PIN, LOW);
  }
}

//---------------------------------------------------------------
int Fill_Resevoir_Tank()
{
  Serial.println("Filling resevoir tank...");
  
  digitalWrite(RESEVOIR_PUMP_PIN, HIGH);

  int Counter = 0;
  while(FLOAT_SENSOR == DEACTIVATED)
  {
    delay(500);
    
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    Counter++;
    if(Counter >= 40)
    {
      Fault(RESEVOIR_PUMP_TIMEOUT_FAULT);
      break;
    }
  }

  digitalWrite(RESEVOIR_PUMP_PIN, LOW);
  delay(1000);                                                                  // Short delay to ensure no water flowing anymore
  
  int V;                                                                        // Read and take average of tank water volume
  for(int i = 0; i < 32; i++) {
    V = Read_Water_Volume();
  }

  return(V);
}

//---------------------------------------------------------------
int Read_Water_Volume()
{
  delay(50);
  float pressure_hPa = (mpr.readPressure() - 1000.0) * 100.0;
  int P = (int)pressure_hPa; 

  P = Pressure_Filter.Rolling_Average(P);

  return(P);
}

//---------------------------------------------------------------
void Fault(int Fault_Code)
{
  static int Active_Faults = 0;
  pinMode(BUZZER_GND, OUTPUT);
  
  if(Fault_Code == FAULT_HANDLE)
  {
    switch(Active_Faults)  {
      case RESEVOIR_PUMP_TIMEOUT_FAULT:
        Serial.println("RESEVOIR_PUMP_TIMEOUT_FAULT");
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(490);
      break;
      
      case GROWBED_PUMP_TIMEOUT_FAULT:
        Serial.println("GROWBED_PUMP_TIMEOUT_FAULT");
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(190);
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(290);
      break;
      
      case MPRLS_FAULT:
       Serial.println("MPRLS_FAULT");
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(190);
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(190);
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        delay(90);
      break;
    }    
  }
  else
  {
    Active_Faults = Fault_Code; 
  }

  pinMode(BUZZER_GND, INPUT);
}
