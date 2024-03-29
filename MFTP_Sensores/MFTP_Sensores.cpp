#include "Arduino.h"
#include "MFTP_Sensores.h"
/****************************************************************************
 * 
 ****************************************************************************/

/****************************************************************************
 * BEGIN MFTP SENSORES FUNCTIONS
 ****************************************************************************/

/* 
 * constructor
 */
  MFTP_Sensores::MFTP_Sensores() {

	for(int i = 0;i<MAX_SENSORS;i++) {
		_active_sensors[i] = false;
		_sensors_matrix[i][0] = 0;
		_sensors_matrix[i][1] = 0;
	}
	for(int i = 0;i<MAX_SENSORS_TYPES;i++) {
		_sensors_type_matrix[i][0] = 0;
		_sensors_type_matrix[i][1] = 0;
		_sensors_type_matrix[i][2] = 0;
	}

 }

/*
 * add_type (id, parA, parB, mult)
 *
 * adds a new sensor type
 *
 */
  void MFTP_Sensores::add_type(int id, int parA, int parB, int mult){

	_sensors_type_matrix[id-1][0] = parA;
	_sensors_type_matrix[id-1][1] = parB;
	_sensors_type_matrix[id-1][2] = mult;
	
  }

/*
 * add_sensor (id, type, address)
 *
 * adds a new sensor
 *
 */
  void MFTP_Sensores::add_sensor(int id, int type_id, int address){

	_sensors_matrix[id-1][0] = type_id;
	_sensors_matrix[id-1][1] = address;
	_active_sensors[id-1] = true;
	
  }

/*
 * get_value(sensor_id)
 * 
 * returns the value read by the sensor sensor_id at that time
 */
  int MFTP_Sensores::get_value(int sensor_id){
	
	if(_active_sensors[sensor_id-1]) {
		int type = _sensors_matrix[sensor_id-1][0];
		int parA = _sensors_type_matrix[type-1][0];
		int parB = _sensors_type_matrix[type-1][1];
		int multiplier = _sensors_type_matrix[type-1][2];
		int value = (parA+ parB*analogRead(_sensors_matrix[sensor_id-1][1]))/multiplier;
		return value;
	}
	else return 0;

  } // get_value
  
/*
 * get_all_values()
 * 
 * return all active sensors values
 */
  void MFTP_Sensores::get_all_values(int *sensor_values){

	for(int id=1;id<=MAX_SENSORS;id++) {
			
		int value = get_value(id);
		sensor_values[id] = value;
	}

  } // get_values


