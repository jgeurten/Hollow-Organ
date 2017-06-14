#include "Servo.h"
#include "Arduino.h"

using namespace std;

Servo::Servo(int en, int rst, int sleep, int ms1, int ms2, int ms3, int step, int dir)
{
	this->ENABLE= en   ;
	this->RST   = rst  ;
	this->SLEEP = sleep;
	this->MS1   = ms1  ;
	this->MS2   = ms2  ;
	this->MS3   = ms3  ;
	this->STEP  = step ;
	this->DIR   = dir  ;

	pinMode(this->ENABLE, OUTPUT);
	pinMode(this->RST, OUTPUT);
	pinMode(this->SLEEP, OUTPUT);
	pinMode(this->MS1, OUTPUT);
	pinMode(this->MS2, OUTPUT);
	pinMode(this->MS3, OUTPUT);
	pinMode(this->STEP, OUTPUT);
	pinMode(this->DIR, OUTPUT);

	this->motorOn = false;
	this->direction = CCW;
}


Servo::~Servo()
{
	if (this->motorOn) disableMotor();
	delete this;
}

void Servo::enableMotor()
{
	if (this->motorOn) return;

	digitalWrite(this->ENABLE, HIGH);
	digitalWrite(this->SLEEP, HIGH);
	digitalWrite(this->RST, HIGH);
}

void Servo::disableMotor()
{
	if (!this->motorOn) return;
	digitalWrite(this->ENABLE,LOW);
	digitalWrite(this->SLEEP, LOW);
	digitalWrite(this->RST, LOW);
	this->motorOn = false;
}




