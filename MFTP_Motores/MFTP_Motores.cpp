#include "Arduino.h"
#include "MFTP_Motores.h"
#include <PID_v1.h>
/****************************************************************************
 * 
 ****************************************************************************/

/****************************************************************************
 * BEGIN MFTP MOTORES FUNCTIONS
 ****************************************************************************/

int _encoder0, _encoder0_pinB,
_encoder1, _encoder1_pinB;

void doEncoder0(); 
void doEncoder1();

/* 
 * constructor
 */
  MFTP_Motores::MFTP_Motores() {

  	analogReference(DEFAULT);

	_encoder0=0;
	_encoder1=0;
	_encoder0_pinB=0;
	_encoder1_pinB=0;
	_lastMillis = millis();

	for(int i = 0;i<MAX_MOTORS;i++) {

	  _moving_motors[i] = false;
	  _active_motors[i] = false;

	  _motor_matrix[i][0] = 0;
	  _motor_matrix[i][1] = 0;
	  _motor_matrix[i][2] = 0;
	  _motor_matrix[i][3] = 0;
	  _motor_matrix[i][4] = 0;
	  _motor_matrix[i][5] = 0;
	  _motor_matrix[i][6] = 0;
	  _motor_matrix[i][7] = 0;
	  _motor_matrix[i][8] = 0;

	  _motor_dynamic_matrix[i][0] = 0;
	  _motor_dynamic_matrix[i][1] = 0;
	  _motor_dynamic_matrix[i][2] = 0;
	  _motor_dynamic_matrix[i][3] = 0;
	  _motor_dynamic_matrix[i][4] = 0;
	  _motor_dynamic_matrix[i][5] = 0;
	  _motor_dynamic_matrix[i][6] = 0;

      //_motors_PID[i] = &_PID0;

	}

 }
////////////////////////////////////////////

/*
 * add_motor
 *
 * adds a new motor
 *
 */
void MFTP_Motores::add_motor(int id, int pwm1, int pwm2, int cnA, int cnB, int ppv, int curso, int reducao, int vmax, int Kp, int invTp, int posini){

	  _motor_matrix[id-1][0] = pwm1;
	  _motor_matrix[id-1][1] = pwm2;
	  _motor_matrix[id-1][2] = cnA;
	  _motor_matrix[id-1][3] = cnB;
	  _motor_matrix[id-1][4] = ppv;
	  _motor_matrix[id-1][5] = curso;
	  _motor_matrix[id-1][6] = reducao;
	  _motor_matrix[id-1][7] = vmax;
	  _motor_matrix[id-1][8] = posini;

  	  _motor_dynamic_matrix[id-1][2] = posini; 

	  pinMode(pwm1, OUTPUT); //Saida pwm para um terminal do motor
	  pinMode(pwm2, OUTPUT); //Saida pwm para o outro terminal do motor

	  pinMode(cnA, INPUT); 
	  digitalWrite(cnA, HIGH);       // turn on pullup resistor
	  pinMode(cnB, INPUT); 
	  digitalWrite(cnB, HIGH);       // turn on pullup resistor

//////////////
	if(cnA == 2) {
		_encoder_pos[id-1] = &_encoder0;
		_encoder0_pinB = cnB;
		attachInterrupt(0, doEncoder0, CHANGE);
		
		static PID _PID0(&_motor_dynamic_matrix[id-1][5], &_motor_dynamic_matrix[id-1][6], &_motor_dynamic_matrix[id-1][0],(double)Kp/(double)invTp,Kp,0, DIRECT);
		_PID0.SetSampleTime(0.089/Kp);
        _motors_PID[id-1] = _PID0;
	}
	if(cnA == 3) {
		_encoder_pos[id-1] = &_encoder1;
		_encoder1_pinB = cnB;
		attachInterrupt(1, doEncoder1, CHANGE);

		static PID _PID1(&_motor_dynamic_matrix[id-1][5], &_motor_dynamic_matrix[id-1][6], &_motor_dynamic_matrix[id-1][0],(double)Kp/(double)invTp,Kp,0, DIRECT);
		_PID1.SetSampleTime(0.089/Kp);
        _motors_PID[id-1] = _PID1;
	}

	  _active_motors[id-1] = true;
  }

/* 
 * emergency_stop
 *
 * stops all motors
 *
 */
  void MFTP_Motores::emergency_stop(){

	for(int i = 0;i<MAX_MOTORS;i++) {

	  _moving_motors[i] = false;
	}
  }

/*
 * set_speed
 *
 * sets the speed of a specific motor
 *
 */
  void MFTP_Motores::set_speed(int motor_id, int speed) {

	_motor_dynamic_matrix[motor_id-1][0] = 255*speed/_motor_matrix[motor_id-1][7];

  }

/*
 * set_target
 *
 * sets the target position for a specific motor
 *
 */
  void MFTP_Motores::set_target(int motor_id, int target) {

	_motor_dynamic_matrix[motor_id-1][1] = target;

  }

/*
 * set_move
 *
 * sets if a motor should be moving or not
 *
 */
  void MFTP_Motores::set_move(int motor_id, boolean move) {

	_moving_motors[motor_id-1] = move;
  	if(move) _motors_PID[motor_id-1].SetMode(AUTOMATIC);
	else {
	  if(_active_motors[motor_id-1]) {
	  	_motors_PID[motor_id-1].SetMode(MANUAL);
  		analogWrite(_motor_matrix[motor_id-1][0],255);
  		analogWrite(_motor_matrix[motor_id-1][1],255);
	  }
	}
  }

/*
 * get_pos
 *
 * returns the position of a motor (its car, actually)
 *
 */
  int MFTP_Motores::get_pos(int motor_id) {

  	return  _motor_dynamic_matrix[motor_id-1][2];

  }

/* 
 * refresh
 *
 * refreshes the position of the motor (revolutions) and the output (pwm)
 *
 */
  void MFTP_Motores::refresh() {

	for(int id = 1;id<=MAX_MOTORS;id++) {

	  if(_active_motors[id-1]) {

		if(*_encoder_pos[id-1] > _motor_matrix[id-1][4]) {

		 _motor_dynamic_matrix[id-1][3]++;
		 *_encoder_pos[id-1] -= _motor_matrix[id-1][4];

		} else if (*_encoder_pos[id-1] < _motor_matrix[id-1][4]) {

		 _motor_dynamic_matrix[id-1][3]--;
		 *_encoder_pos[id-1] += _motor_matrix[id-1][4];

		}

		int angular_pos = (_motor_dynamic_matrix[id-1][3]*_motor_matrix[id-1][4]+*_encoder_pos[id-1]);

		if(angular_pos>_motor_dynamic_matrix[id-1][4]+ANGULAR_HISTERESIS ||
		angular_pos<_motor_dynamic_matrix[id-1][4]-ANGULAR_HISTERESIS) {
		
			unsigned int time = millis()-_lastMillis;
			_lastMillis = millis();
			_motor_dynamic_matrix[id-1][5] = (angular_pos - _motor_dynamic_matrix[id-1][4])*1000/time; // speed =  current angular pos - last angular pos / time

			_motor_dynamic_matrix[id-1][4] = angular_pos;
		}
		_motor_dynamic_matrix[id-1][2] = _motor_matrix[id-1][8] + angular_pos/_motor_matrix[id-1][6]; // position = pos_ini + (revolutions*ppv + pulses)*reduction

		if(_moving_motors[id-1]) {

		  _motors_PID[id-1].Compute();
		  if (_motor_dynamic_matrix[id-1][6]==0) { //stop
	  		analogWrite(_motor_matrix[id-1][0],255);
	  		analogWrite(_motor_matrix[id-1][1],255);
		  }
		  else if(_motor_dynamic_matrix[id-1][6]>0) { // move forward
	  		analogWrite(_motor_matrix[id-1][0],_motor_dynamic_matrix[id-1][6]);
	  		analogWrite(_motor_matrix[id-1][1],0);
		  } 
		  else { // move backwards
	  		analogWrite(_motor_matrix[id-1][0],0);
	  		analogWrite(_motor_matrix[id-1][1],_motor_dynamic_matrix[id-1][6]);
		  }

		}
		else { //stop
  		  analogWrite(_motor_matrix[id-1][0],255);
  		  analogWrite(_motor_matrix[id-1][1],255);

		}

	  }

	}

  }
  
////////////////////////


void doEncoder0() { // problem : there are only interrupts for pin 2 and 3 !!!
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
	if (digitalRead(2) == digitalRead(_encoder0_pinB)) {
	_encoder0++;
	} else {
	_encoder0--;
	}

}

void doEncoder1() { // problem : there are only interrupts for pin 2 and 3 !!!
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
	if (digitalRead(3) == digitalRead(_encoder1_pinB)) {
	_encoder1++;
	} else {
	_encoder1--;
	}

}



