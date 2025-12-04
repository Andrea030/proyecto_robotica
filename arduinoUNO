#include <math.h>

// --- Definición de Pines ---
// Motor A (Asumiremos que está a 0 grados o "frente")
int aMotorSpeedPin = 3;
int aMotorDirection1 = 4;
int aMotorDirection2 = 5;

// Motor B (Asumiremos que está a 120 grados)
int bMotorSpeedPin = 9;
int bMotorDirection1 = 8;
int bMotorDirection2 = 10;

// Motor C (Asumiremos que está a 240 grados)
int cMotorSpeedPin = 11;
int cMotorDirection1 = 7;
int cMotorDirection2 = 6;

// Velocidad máxima PWM (0 - 255)
const int MAX_PWM = 200; // Ajusta esto si va muy rápido

void setup() {
  Serial.begin(9600); // Iniciamos comunicación serial
  
  // Motor A
  pinMode(aMotorSpeedPin, OUTPUT);
  pinMode(aMotorDirection1, OUTPUT);
  pinMode(aMotorDirection2, OUTPUT);

  // Motor B
  pinMode(bMotorSpeedPin, OUTPUT);
  pinMode(bMotorDirection1, OUTPUT);
  pinMode(bMotorDirection2, OUTPUT); // CORREGIDO: Antes tenías Direction1 repetido

  // Motor C
  pinMode(cMotorSpeedPin, OUTPUT);
  pinMode(cMotorDirection1, OUTPUT);
  pinMode(cMotorDirection2, OUTPUT); // CORREGIDO: Antes tenías Direction1 repetido

  // Detener todo al inicio
  moverMotor(0, aMotorSpeedPin, aMotorDirection1, aMotorDirection2);
  moverMotor(0, bMotorSpeedPin, bMotorDirection1, bMotorDirection2);
  moverMotor(0, cMotorSpeedPin, cMotorDirection1, cMotorDirection2);
  
  Serial.println("Robot Omnidireccional Listo.");
  Serial.println("Ingresa coordenadas formato: x,y (ejemplo: 100,200)");
}

void loop() {
  if (Serial.available() > 0) {
    // Leemos los valores X e Y separados por coma
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    
    // Limpiamos el buffer restante (saltos de línea)
    while(Serial.available()) { Serial.read(); }

    if (x == 0 && y == 0) {
      // Si recibimos 0,0 paramos los motores
      detenerMotores();
    } else {
      calcularMovimiento(x, y);
    }
  }
}

void calcularMovimiento(float x, float y) {
  // 1. Calcular el ángulo del vector objetivo (en radianes)
  float theta = atan2(y, x);

  // 2. Calcular la magnitud (velocidad general)
  float magnitud = sqrt(x*x + y*y);
  
  // Mapeamos la magnitud para que no exceda la velocidad máxima del motor
  // Asumimos que la entrada X,Y máxima esperada es aprox 512 (puedes cambiar esto)
  if (magnitud > 255) magnitud = 255;
  magnitud = constrain(magnitud, 0, 255);

  // 3. Calcular velocidad individual para cada rueda
  // La fórmula general para 3 ruedas a 120 grados:
  // V_rueda = Magnitud * cos(DireccionDeseada - AnguloRueda)
  
  // Ajuste de ángulos de las ruedas (en radianes):
  // Rueda A: 0 radianes (0 grados)
  // Rueda B: 2.09 radianes (120 grados)
  // Rueda C: 4.18 radianes (240 grados)
  
  float vA = magnitud * cos(theta - 0);
  float vB = magnitud * cos(theta - (120 * PI / 180.0)); 
  float vC = magnitud * cos(theta - (240 * PI / 180.0));

  // Escalar velocidades al máximo permitido
  // (Opcional: normalizar si alguna velocidad excede 255)

  // 4. Mover los motores
  moverMotor((int)vA, aMotorSpeedPin, aMotorDirection1, aMotorDirection2);
  moverMotor((int)vB, bMotorSpeedPin, bMotorDirection1, bMotorDirection2);
  moverMotor((int)vC, cMotorSpeedPin, cMotorDirection1, cMotorDirection2);

  // Debugging para ver qué pasa
  Serial.print("Obj X:"); Serial.print(x);
  Serial.print(" Y:"); Serial.print(y);
  Serial.print(" -> VA:"); Serial.print(vA);
  Serial.print(" VB:"); Serial.print(vB);
  Serial.print(" VC:"); Serial.println(vC);
}

// Función auxiliar para controlar dirección y velocidad (H-Bridge)
void moverMotor(int velocidad, int pinSpeed, int pinDir1, int pinDir2) {
  // Si la velocidad es positiva, giramos en un sentido
  if (velocidad > 0) {
    digitalWrite(pinDir1, HIGH);
    digitalWrite(pinDir2, LOW);
    // Aseguramos no pasarnos de 255
    if(velocidad > MAX_PWM) velocidad = MAX_PWM;
    analogWrite(pinSpeed, velocidad);
  } 
  // Si la velocidad es negativa, invertimos los pines de dirección
  else if (velocidad < 0) {
    digitalWrite(pinDir1, LOW);
    digitalWrite(pinDir2, HIGH);
    // Convertimos velocidad a positivo para el PWM
    velocidad = abs(velocidad);
    if(velocidad > MAX_PWM) velocidad = MAX_PWM;
    analogWrite(pinSpeed, velocidad);
  } 
  // Si es 0, frenamos
  else {
    digitalWrite(pinDir1, LOW);
    digitalWrite(pinDir2, LOW);
    analogWrite(pinSpeed, 0);
  }
}

void detenerMotores() {
  moverMotor(0, aMotorSpeedPin, aMotorDirection1, aMotorDirection2);
  moverMotor(0, bMotorSpeedPin, bMotorDirection1, bMotorDirection2);
  moverMotor(0, cMotorSpeedPin, cMotorDirection1, cMotorDirection2);
  Serial.println("Motores detenidos.");
}
