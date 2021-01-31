/*
    Name:       3DPA_X01.ino
	Description:3D Printed Actuator (3.5mm Bearing Ball Screw)
    Created:	02/18/2019 9:40:17 PM
	Revised:	07/03/2019
    Author:     DESIGNCELL-PC\DesignCell

	Ball Screw Details:
	3.5mm Bearing
	4.5mm Pitch
	Gearmotor 560rpm
	Belt 11/20T
	Encoder 16/47T Off Ball Screw Nut
	Encoder PEC12R-4017F
		24PPR -> 48PPR Up/Down
	48P/Rev x 47T / 4.5mm/Rev == 501.3333 p/mm (Float?)
	Limits NC
	Travel Limit: ???

	Wiring:
	Retract Limit:	C = BlueWhite	NC = Blue
	Extend Limit:	C = GreenWhite	NC = Green
	Encoder:		C = Brown?	Retract =	Extend =		BrownWhite & Orange
	Gearmotor:		(-) = White		(+) = Red

	System Layout (From Top Down)
	X = (-)Left (+)Right
	Y = (-)REV (+)FWD
	Z = (-)Down (+)Up
	XX = Roll (-)Left (+)Right
	YY = Pitch (-)Nose Down (+)Nose Up
	ZZ = Yaw (-)Left (+)Right

	1.1 FL Sholder X
	1.2 FL Sholder Y
	1.3 FL Knee
	2.1 FR Shoulder X
	2.2 FR Sholder Y
	2.3 FR Knee


	*/

#include <PID_v1.h>

//PID Varibles
double Set_1, In_1, Out_1;				
double Set_2, In_2, Out_2;			
//PID Parameters		
double Kp = 2.5, Ki = 0, Kd = 0.05;
int16_t Spd_Limit = 255;
PID PID_1(&In_1, &Out_1, &Set_1, Kp, Ki, Kd, P_ON_E, DIRECT);
PID PID_2(&In_2, &Out_2, &Set_2, Kp, Ki, Kd, P_ON_E, DIRECT);

#include <Encoder.h>

//Encoder Ref: https://www.pjrc.com/teensy/td_libs_Encoder.html
Encoder Encoder_1(2, 4);								
Encoder Encoder_2(3, 5);


int32_t Enc_Pos_1;
int32_t Enc_Pos_2;

//Limit Switches Pins
int8_t Ret_Limit_1 = 6;	
int8_t Ext_Limit_1 = 7;
int8_t Ret_Limit_2 = 8;
int8_t Ext_Limit_2 = 9;
bool Ext_1, Ext_2, Ret_1, Ret_2;

//TB6612FNG
uint8_t PWM_1 = 10; //Pro Mini PWM Pin
uint8_t DIR_1 = 12; 
uint8_t PWM_2 = 11; //Pro Mini PWM Pin
uint8_t DIR_2 = 13;

// defines for setting and clearing register bits
//#ifndef cbi
//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#endif
//#ifndef sbi
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
//#endif

//Analog Read Interval
int16_t	ar1;
uint16_t arMicros;
uint32_t pMicros = 0;
int16_t interval = 25000; //Microsecond method interval

void setup()
{
	Serial.begin(115200);
	//PID Parameters from Analog Read Potentimeter for discovery
	PID_1.SetMode(AUTOMATIC);
	PID_1.SetTunings(Kp, Ki, Kd);
	PID_1.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -Lim to +Lim allows negative rotation
	PID_2.SetMode(AUTOMATIC);
	PID_2.SetTunings(Kp, Ki, Kd);
	PID_2.SetOutputLimits(-Spd_Limit, Spd_Limit); //Default 0-255. -Lim to +Lim allows negative rotation


	Encoder_1.write(6000);
	Encoder_2.write(6000);

	// set prescale to 16
	// Eratic measurements input when other inputs change?
	//sbi(ADCSRA, ADPS2);
	//cbi(ADCSRA, ADPS1);
	//cbi(ADCSRA, ADPS0);

	// Limit Switch pin mode Input_Pullup to send to HIGH when switch open using NC limit to ground.
	pinMode(Ret_Limit_1, INPUT_PULLUP);
	pinMode(Ret_Limit_2, INPUT_PULLUP);
	pinMode(Ext_Limit_1, INPUT_PULLUP);
	pinMode(Ext_Limit_2, INPUT_PULLUP);

}

void serialprint()
{
	char sPrint[40];
	uint16_t num1 = Set_1;
	uint16_t num2 = Enc_Pos_1;
	int16_t num3 = Out_1;
	int16_t	num4 = Set_1 - Enc_Pos_1;
	sprintf(sPrint, "%d,%d,%d,%d", num1, num2, num3, num4);

	//Print
	Serial.println(sPrint);
}

void Analog_Read()
{
	Set_1 = map(analogRead(A0), 0, 1023, -100.0, 6000.0);
	//	Change this to a joystick method that adds/subtracts to the setpoint to allow analog positioning.
	//	example: joystick range mapped to cycle +/-, so joystick adder scales with how far joystick is moved.

	// Read Potentiometer for Kp, Ki, & Kd
	//Kp = map(analogRead(A1), 0, 1023, 0, 100) / 100.00; // Divide by 100.00 to output float 0-1
	//Ki = map(analogRead(A2), 0, 1023, 0, 100) / 100.00; // Divide by 100.00 to output float 0-1
	//Kd = map(analogRead(A3), 0, 1023, 0, 100) / 100.00; // Divide by 100.00 to output float 0-1
	//posPID.SetTunings(Kp, Ki, Kd);
}

void Limits()
{
	// Actuator 1 Retract Limit
	if (digitalRead(Ret_Limit_1) == HIGH)
	{
		Encoder_1.write(0); //Reset homing
		Ret_1 = false;
		PID_1.SetOutputLimits(0, Spd_Limit); //Limit no retraction
	}
	else
	{
		Ret_1 = true;
		PID_1.SetOutputLimits(-Spd_Limit, Spd_Limit); //Reset to open limit range
	}

	// Actuator 2 Retract Limit
	if (digitalRead(Ret_Limit_2) == HIGH)
	{
		Encoder_2.write(0);
		Ret_2 = false;
		PID_2.SetOutputLimits(0, Spd_Limit);
	}
	else
	{
		Ret_2 = true;
		PID_2.SetOutputLimits(-Spd_Limit, Spd_Limit);
	}

	// Actuator 1 Extend Limit
	if (digitalRead(Ext_Limit_1) == HIGH)
	{
		Ext_1 = false;
		PID_1.SetOutputLimits(-Spd_Limit, 0);
	}
	else
	{
		Ext_1 = true;
		PID_1.SetOutputLimits(-Spd_Limit, Spd_Limit);
	}

	// Actuator 2 Extend Limit
	if (digitalRead(Ext_Limit_2) == HIGH)
	{
		Ext_2 = false;
		PID_2.SetOutputLimits(-Spd_Limit, 0);
	}
	else
	{
		Ext_2 = true;
		PID_2.SetOutputLimits(-Spd_Limit, Spd_Limit);
	}
}

void loop()
{

	// Limit Method
	Limits();

	// Method Interval
	uint32_t cMicros = micros();
	if (cMicros - pMicros > interval)
	{
		pMicros = cMicros;	//Reset time interval
		Analog_Read();		//Run Analog Read Method
		serialprint();		//Run Print Method
	}

	//Drive 1
	Enc_Pos_1 = Encoder_1.read();
	In_1 = Enc_Pos_1;
	PID_1.Compute();

	//Drive 2
	Enc_Pos_2 = Encoder_2.read();
	In_2 = Enc_Pos_2;
	PID_2.Compute();

	//Determine Drive 1 Rotation Sense
	if (Out_1 > 20 && Ext_1 == true)
	{
		digitalWrite(DIR_1, HIGH);
		analogWrite(PWM_1, Out_1);
	}
	else if (Out_1 < -20 && Ret_1 == true) 
	{
		digitalWrite(DIR_1, LOW);
		analogWrite(PWM_1, -Out_1);
	}
	else 
	{ 
		digitalWrite(DIR_1, LOW);
		analogWrite(PWM_1, 0);
	}

}
