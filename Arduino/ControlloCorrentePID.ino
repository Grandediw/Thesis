#define ENCA 3
#define ENCB 5
#define PWM 9
#define IN2 4
#define IN1 2
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

float current_setpoint = 100;
float current_error = 0;
float current_integral = 0;
float current_derivative = 0;
float last_current_error = 0;
float current = 0;
float Previouscurrent = 0;
float alpha = 0.80;
long prevT = 0;
int i = 0;
int iteratore = 0;



void misure(float x) {

  // Filtro media mobile
  // current = x * ina219.getCurrent_mA() + (1.0 - x)*Previouscurrent;
  // delay(1);
  //Previouscurrent = current;
  // for (int i = 0; i < 50; i++) {
  //   current += ina219.getCurrent_mA();
  //   delay(1);
  // }
  // current = current / 50;

  //Prova Nuovo Filtro Media Mobile
if (i==0) {
current = ina219.getCurrent_mA();
}
i++;
  Previouscurrent = ina219.getCurrent_mA();
  current = x * current + (1.0 - x) * Previouscurrent;
  delay(1);
}



const float PWM_MAX = 255;  // massimo valore PWM

const float KP = 0.16999;    // costante proporzionale del regolatore
const float KI = 9.5575;    // costante integrale del regolatore
const float KD = 0;       // costante derivativa del regolatore

void setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_1A();
  delay(1000);
  Serial.begin(9600);
}

void loop() {

  if (iteratore++ > 699) {
     iteratore = 0;
   }
   if (iteratore<0) {
   current_setpoint = 80;
   }else if (iteratore < 210) {
     current_setpoint = 80;
   } else if (iteratore < 410) {
    current_setpoint = -80;
   }
   else {
    current_setpoint = -80*sin(prevT/1e5);
   }


  // current_setpoint = 100*sin(prevT/1e5);
  // if (iteratore++ > 199) {
  //   iteratore = 0;
  // }

  // if (iteratore == 199) {
  // current_setpoint = - current_setpoint;
  // }

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  
  misure(alpha);
  // Calcola l'errore di corrente
  current_error = current_setpoint - current;

  // Calcola l'integrale dell'errore di corrente
  current_integral = current_integral + current_error * deltaT;

  // Limita l'integrale dell'errore di corrente
  if (current_integral > PWM_MAX / KI) {
    current_integral = PWM_MAX / KI;
  } else if (current_integral < -PWM_MAX / KI) {
    current_integral = -PWM_MAX / KI;
  }

  // Calcola la derivata dell'errore di corrente
  current_derivative = (current_error - last_current_error) / deltaT;

  // Calcola il valore di controllo del motore
  float control_value = KP * current_error + KI * current_integral + KD * current_derivative;

  // Limita il valore di controllo del motore
  if (control_value > PWM_MAX) {
    control_value = PWM_MAX;
  } else if (control_value < -PWM_MAX) {
    control_value = -PWM_MAX;
  }

  // Imposta il segnale PWM
  if (control_value >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM, control_value);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, -control_value);
  }

  if (current_integral > PWM_MAX / KI) {
    current_integral = PWM_MAX / KI;
  } else if (current_integral < -PWM_MAX / KI) {
    current_integral = -PWM_MAX / KI;
  }

  // Memorizza l'errore di corrente attuale
  last_current_error = current_error;

  // Stampa i valori sulla seriale
 // Serial.print("Current:");
  Serial.print(current_setpoint);
  Serial.print(",");
//  Serial.print("Control.value:");
 // Serial.print(control_value);
 // Serial.print(",");
//  Serial.print("Current.error:");
 // Serial.print(current_error);
 // Serial.print(",");
//  Serial.print("Current.setpoint:");
  Serial.println(current);
//  Serial.print(",");
}
