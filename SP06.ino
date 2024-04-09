#define PIN_DC_VOLTAGE PC_3C
#define PIN_THERMISTOR PB_15 // external thermistor
#define PIN_CURRENT PA_0
#define PIN_PACK_TEMP PA_6 // mosfet thermistor 

#define PIN_U_PHASE PA_4
#define PIN_V_PHASE PC_3
#define PIN_W_PHASE PC_2

#define PIN_U_NEG_PHASE PA_0C 
#define PIN_V_NEG_PHASE PA_1C
#define PIN_W_NEG_PHASE PC_2C

#define GATE_6 PA_8
#define GATE_5 PC_6
#define GATE_4 PC_7
#define GATE_3 PG_7
#define GATE_2 PJ_11
#define GATE_1 PH_6

void setup() {
  Serial.begin(9600);
}

void loop() {
  
  // float read_dc_voltage = analogRead(PIN_DC_VOLTAGE);
  // Serial.print("dc_voltage : ");
  // Serial.println(read_dc_voltage);

  // float read_thermistor_mst = analogRead(PIN_PACK_TEMP);
  // Serial.print("mos_temp : ");
  // Serial.println(read_thermistor_mst);
  
  // float read_thermistor_ext = analogRead(PIN_THERMISTOR);
  // Serial.print("motor_temp : ");
  // Serial.println(read_thermistor_ext);
    
  // float read_dc_current = analogRead(PIN_CURRENT);
  // Serial.print("dc_current : ");
  // Serial.println(read_dc_current);

  // float read_u_current = analogRead(PIN_U_PHASE);
  // Serial.print("phase_current : ");
  // Serial.println(read_u_current);

  digitalWrite(GATE_5, HIGH);
  digitalWrite(GATE_1, HIGH);
  delay(500);

  digitalWrite(GATE_5, LOW);
  digitalWrite(GATE_1, LOW);
  delay(500);

}
