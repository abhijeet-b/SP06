#include <SimpleFOC.h>
HallSensor sensor = HallSensor(22, 42, 40, 11); // pole pairs 
BLDCMotor motor = BLDCMotor(11); // pole pairs 
BLDCDriver6PWM driver = BLDCDriver6PWM(59, 61, 63, 65, 67, 68);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() {

  sensor.init();
  motor.linkSensor(&sensor);

  driver.pwm_frequency = 20000; 
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 24; 
  driver.init();
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Q axis
  // PID parameters - default 
  motor.PID_current_q.P = 5;   
  motor.PID_current_q.I = 1000;  
  motor.PID_current_q.D = 0;
  motor.PID_current_q.limit = motor.voltage_limit; 
  motor.PID_current_q.ramp = 1e6;  
  // Low pass filtering - default 
  motor.LPF_current_q.Tf= 0.005; 
  
  // D axis
  // PID parameters - default 
  motor.PID_current_d.P = 5;    
  motor.PID_current_d.I = 1000;
  motor.PID_current_d.D = 0;
  motor.PID_current_d.limit = motor.voltage_limit; 
  motor.PID_current_d.ramp = 1e6;
  // Low pass filtering - default 
  motor.LPF_current_d.Tf= 0.005; 

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  // define the motor id
  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
}


void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
}
