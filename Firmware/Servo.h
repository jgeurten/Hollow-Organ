#pragma once
#ifndef SERVO_H
#define SERVO_H

class Servo {
private:

	Servo(int en, int rst, int sleep, int ms1, int ms2, int ms3, int step, int dir);
	~Servo();
	void setSpeed(int speed); //in RPM
	int readEncoder();	//returns actual speed
	void resetEncoder();

	void enableMotor();
	void disableMotor();
	float calcError();
	void includeInterrupts();

	//properties
	bool motorOn; 
	std::string direction;
	int ENABLE;
	int RST;
	int SLEEP;
	int MS1;
	int MS2;
	int MS3;
	int STEP;
	int DIR;
};
#endif // !SERVO_H
