#ifndef MFTP_MOTORES_H
#define MFTP_MOTORES_H
/****************************************************************************
 * 
 ****************************************************************************/

/****************************************************************************
 * BEGIN MOTORES FUNCTIONS
 ****************************************************************************/
#include "Arduino.h"
#include <PID_v1.h>

class MFTP_Motores {
private:

  enum { 
    MAX_MOTORS = 2, 
  };


  int * _encoder_pos[MAX_MOTORS];
  boolean _moving_motors[MAX_MOTORS];
  boolean _active_motors[MAX_MOTORS];
  int _motor_matrix[MAX_MOTORS][11]; // pwm1 & pwm2 & cnA & cnB & pulses/revolution & course $ reduction & vmax & Kp & Tp & posini
  int _motor_dynamic_matrix[MAX_MOTORS][7]; // set_speed & target_position & position & revolutions & angular_position & current_speed & controller_output

  PID _motors_PID[MAX_MOTORS];

  static void doEncoder0(); 
  static void doEncoder1();
  
public:
  static int _encoder0, _encoder0_pinA, _encoder0_pinB,
	  _encoder1, _encoder1_pinA, _encoder1_pinB;

/* 
 * constructor
 */
  MFTP_Motores();

/*
 * add_motor
 *
 * adds a new motor
 *
 */
  void add_motor(int id, int pwm1, int pwm2, int cnA, int cnB, int ppv, int curso, int reducao, int vmax, int Kp, int invTp, int posini);

/* 
 * emergency_stop
 *
 * stops all motors
 *
 */
  void emergency_stop();

/*
 * set_speed
 *
 * sets the speed of a specific motor
 *
 */
  void set_speed(int motor_id, int speed);

/*
 * set_target
 *
 * sets the target position for a specific motor
 *
 */
  void set_target(int motor_id, int target);

/*
 * set_move
 *
 * sets if a motor should be moving or not
 *
 */
  void set_move(int motor_id, boolean move);

/*
 * get_pos
 *
 * returns the position of a motor (its car, actually)
 *
 */
  int get_pos(int motor_id);

/* 
 * refresh
 *
 * refreshes the position of the motor (revolutions)
 *
 */
  void refresh();

};

#endif
