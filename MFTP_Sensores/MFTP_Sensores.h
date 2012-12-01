#ifndef MFTP_SENSORES_H
#define MFTP_SENSORES_H
/****************************************************************************
 * 
 ****************************************************************************/

/****************************************************************************
 * BEGIN MFTP SENSORES FUNCTIONS
 ****************************************************************************/
#include "Arduino.h"

class MFTP_Sensores {
private:

enum { 
        MAX_SENSORS = 20, 
        MAX_SENSORS_TYPES = 40, 
};
  bool _active_sensors[MAX_SENSORS];
  int _sensors_matrix[MAX_SENSORS][2]; // type & address
  int _sensors_type_matrix[MAX_SENSORS_TYPES][3]; // parameter A & parameter B & multiplier

  
public:

/* 
 * constructor
 */
  MFTP_Sensores();

/*
 * add_type (id, parA, parB, mult)
 *
 * adds a new sensor type
 *
 */
  void add_type(int id, int parA, int parB, int mult);

/*
 * add_sensor (id, type, address)
 *
 * adds a new sensor
 *
 */
  void add_sensor(int id, int type_id, int address);

/*
 * get_value(sensor_id)
 * 
 * returns the value read by the sensor sensor_id at that time
 */
  int get_value(int sensor_id); 
  
/*
 * get_all_values()
 * 
 * return all active sensors values
 */
  void get_all_values(int *sensor_values); 


// future implementations : kill_sensor, kill_sensor_type, add timestamp to value read !

};

#endif
