#include <PID_v1.h>

#include <MFTP_Motores.h>

#include <MFTP_Sensores.h>

#include <ModbusSlave.h>

ModbusSlave mbs;
MFTP_Sensores Sens;
MFTP_Motores Mots;

/* slave registers */
enum {        
        EMERGENCY_STOP=0,
        SENSOR1,
        SENSOR2,
        SENSOR3,
        SENSOR4,
        SENSOR5,
        SENSOR6,
        SENSOR7,
        SENSOR8,
        SENSOR9,
        SENSOR10,
        SENSOR11,
        SENSOR12,
        SENSOR13,
        SENSOR14,
        SENSOR15,
        SENSOR16,
        SENSOR17,
        SENSOR18,
        SENSOR19,
        SENSOR20,
        TYPE_RECORD,
        TYPE_ID,
        TYPE_PARA,
        TYPE_PARB,
        TYPE_MULT,
        NOT_USED1,
        SENSOR_RECORD,
        SENSOR_ID,
        SENSOR_TYPE,
        SENSOR_ADDRESS,
        NOT_USED2,
        MOTOR_RECORD,
        MOTOR_ID,
        MOTOR_PWM1,
        MOTOR_PWM2,
        MOTOR_CNA,
        MOTOR_CNB,
        MOTOR_PPV,
        MOTOR_COURSE,
        MOTOR_REDUCTION,
        MOTOR_VMAX,
        MOTOR_KP,
        MOTOR_TP,
        MOTOR_POSINI,
        NOT_USED3,
        NOT_USED4,
        SPEED_RECORD,
        SPEED_MOTOR_ID,
        SPEED_VALUE,
        NOT_USED5,
        TARGET_RECORD,
        TARGET_MOTOR_ID,
        TARGET_VALUE,
        MOVE_RECORD,
        MOVE_MOTOR_ID,
        MOVE_VALUE,
        NOT_USED6,
        READPOS_MOTOR_ID,
        READPOS_READY,
        READPOS_VALUE,
        MB_REGS        /* required by ModbusSlave */
};

int regs[MB_REGS]; // here are all the registers 
int result = 0; // return of the update function

#define ledPin 13 // led

void setup() 
{
        
  /* the Modbus slave configuration parameters */
  const unsigned char SLAVE = 1; // my address
  
  const long BAUD = 9600;
  const char PARITY = 'e'; // even
  const char TXENPIN = 0; // not RS485
  
  mbs.configure(SLAVE,BAUD,PARITY,TXENPIN);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,LOW);
          
  for(int i = 0;i<MB_REGS;i++) {
    regs[i] = 0; // initialize registers as 1
  }
        
}

void loop()
{
  result = mbs.update(regs, MB_REGS);
  Sens.get_all_values(regs); // be careful
  Mots.refresh();
  
  if(regs[TYPE_RECORD] != 0) { // ready to record a new type of sensor
    Sens.add_type(regs[TYPE_ID],regs[TYPE_PARA], regs[TYPE_PARB], regs[TYPE_MULT]);
    
    regs[TYPE_ID]=0;
    regs[TYPE_PARA]=0;
    regs[TYPE_PARB]=0;
    regs[TYPE_MULT]=0;
    regs[TYPE_RECORD]=0;
  }
  if(regs[SENSOR_RECORD] != 0) { // ready to record a new sensor
    Sens.add_sensor(regs[SENSOR_ID],regs[SENSOR_TYPE], regs[SENSOR_ADDRESS]);
    
    regs[SENSOR_ID]=0;
    regs[SENSOR_TYPE]=0;
    regs[SENSOR_ADDRESS]=0;
    regs[SENSOR_RECORD]=0;
  }
  if(regs[SENSOR_RECORD] != 0) { // ready to record a new sensor
    Sens.add_sensor(regs[SENSOR_ID],regs[SENSOR_TYPE], regs[SENSOR_ADDRESS]);
    
    regs[SENSOR_ID]=0;
    regs[SENSOR_TYPE]=0;
    regs[SENSOR_ADDRESS]=0;
    regs[SENSOR_RECORD]=0;
  }
  if(regs[MOTOR_RECORD] != 0) { // ready to record a new motor
  
  Mots.add_motor(
        regs[MOTOR_ID],
        regs[MOTOR_PWM1],
        regs[MOTOR_PWM2],
        regs[MOTOR_CNA],
        regs[MOTOR_CNB],
        regs[MOTOR_PPV],
        regs[MOTOR_COURSE],
        regs[MOTOR_REDUCTION],
        regs[MOTOR_VMAX],
        regs[MOTOR_KP],
        regs[MOTOR_TP],
        regs[MOTOR_POSINI]);
    
        regs[MOTOR_PWM1]=0;
        regs[MOTOR_PWM2]=0;
        regs[MOTOR_CNA]=0;
        regs[MOTOR_CNB]=0;
        regs[MOTOR_PPV]=0;
        regs[MOTOR_COURSE]=0;
        regs[MOTOR_REDUCTION]=0;
        regs[MOTOR_VMAX]=0;
        regs[MOTOR_KP]=0;
        regs[MOTOR_TP]=0;
        regs[MOTOR_POSINI]=0;
        regs[MOTOR_ID]=0;
  }
  
}


