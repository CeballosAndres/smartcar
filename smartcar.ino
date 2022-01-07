#include <ArduinoOTA.h>  // Actualizaciones OTA
#include <ESP8266WiFi.h> // Soporte para conectividad WiFi
#include <NewPing.h> // Librería para realizar mediciones con el sensor de proximidad
#include <Servo.h> // Librería para controlar el servomotor

#include "config.h"              // Archivo con configuraciones WiFi

#include "ESP8266_Utils.hpp"     // Lógica para conexión WiFi
#include "ESP8266_Utils_OTA.hpp" // Lógica para actualizaciones OTA

// Definición de entradas/salidas
#define TRIGGER_PIN 12 // Sensor proximidad trigger D6
#define ECHO_PIN 12    // Sensor proximidad echo D6
#define SERVO 13       // Servomotor
#define MPA 5          // Velocidad para motor A
#define MPB 4          // Velocidad para motor B
#define MDA 0          // Dirección motor A
#define MDB 2          // Dirección motor B
#define LED_RIGHT 15   // Led derecho
#define LED_LEFT 14    // Led izquierdo

// Constantes para control del vehiculo
#define MAX_DISTANCE 50.0 // Distancia desde la que empieza a actuar la NN
#define INTERVAL 25       // Intervalos cada x milisegundos (default: 25)
#define INTERVAL_LED 500       // Intervalos cada x milisegundos (default: 25)
#define MIN_ANGLE 30      // Angulo mínimo
#define MAX_ANGLE 150     // Angulo máximo
#define SPEED 350         // Velocidad del coche de las 4 ruedas a la vez.
#define INPUT_NODES 3     // Incluye neurona de BIAS
#define HIDDEN_NODES 4    // Incluye neurona de BIAS
#define OUTPUT_NODES 2
// Ver archivo Red Neuronal.ipynb
const float HIDDEN_WEIGHTS[3][4] = {{0.7993371118882053, -1.4047014502226984, 0.9414582048050834, 0.07642009631120551}, {-0.13297965743408063, -3.341301702562721, 0.6131066138656246, -2.3255056135814103}, {0.23604578914209137, 1.6221600044678037, -0.2315280391669793, -0.7173392995794886}};
const float OUTPUT_WEIGHTS[4][2] = {{0.889885617059632, 0.44366950248439924}, {-0.5935876267502912, 2.0265443012303415}, {1.1108713149135987, 2.0667292793021828}, {2.1970641405729454, 0.10949879437586214}};

// Variables de configuración para la red neuronal
int i, j;
double accum;
double hidden[HIDDEN_NODES];
double output[OUTPUT_NODES];

// Variables para control del vehiculo
unsigned long previousMillis = 0; // Para medir ciclos de tiempo
unsigned long previousMillisLed = 0; // Para medir ciclos de tiempo para leds
int servoAngle = 90; // Posición del servoMotor que mueve el sensor ultrasónico
bool clockwise = true; // Sentido de giro del servoMotor
int increments = 9;    // Incrementos por ciclo de posición del servoMotor
int currentAction = 1; // Cantidad de ciclos ejecutando una acción (aumenta al
                       // detectar objeto cercano)
int multiplier = 500 / INTERVAL; // Multiplica los ciclos para dar tiempo a que
                                 // el coche pueda girar (def: 1000)
int flag_led_left = 0;
int flag_led_right = 0;

// Instanciar objetos
Servo servoMotor;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void stop();

void setup() {
  //ConnectWiFi_STA(); // Habilitar la actualización por OTA
  //InitOTA();
  Serial.begin(115200); // Iniciar comunicación serial
  pinMode(MDA, OUTPUT); // Definición de pines como salidas
  pinMode(MDB, OUTPUT);
  pinMode(MPA, OUTPUT);
  pinMode(MPB, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  stop();                   // Detiene los motores al inicio
  servoMotor.attach(SERVO); // Define pin 13 para controlar servomotor
  servoMotor.write(90);     // Posición inicial en el centro
  delay(5000); // Delay para acomodar el sensor ultrasónico sobre el servomotor
}

void loop() {
  //ArduinoOTA.handle(); // Llamada a método para iniciar actualización OTA

  unsigned long currentMillis = millis(); // Captura tiempo actual
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;
    // Manejar movimiento del servomotor
    if (servoAngle <= MIN_ANGLE || servoAngle >= MAX_ANGLE) {
      clockwise = !clockwise; // Cambio de sentido
      servoAngle = constrain(servoAngle, MIN_ANGLE, MAX_ANGLE);
    }
    if (clockwise)
      servoAngle = servoAngle + increments;
    else
      servoAngle = servoAngle - increments;

    if (currentAction > 0) {
      currentAction = currentAction - 1;
    } else {
      drive(); // Llama a la función de coste
    }
    servoMotor.write(servoAngle);
  }

  currentMillis = millis();
  if ((flag_led_left == 1) || (flag_led_right == 1)){
    if (currentMillis - previousMillisLed >= INTERVAL_LED){
      previousMillisLed = currentMillis;

      if (flag_led_left == 1)
        digitalWrite(LED_LEFT, !digitalRead(LED_LEFT));
      if (flag_led_right == 1)
        digitalWrite(LED_RIGHT, !digitalRead(LED_RIGHT));

    }
  } else {
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
  }

}

void stop() {
  digitalWrite(MPA, LOW); // Desactivamos los motores
  digitalWrite(MPB, LOW); // Desactivamos los motores
  Serial.println("Detener motores");
}

// Usa la red neuronal previamente entrenada
void drive() {
  double inputOne = 0, inputTwo = 0;

  // Obtener distancia de objeto
  double distance = double(sonar.ping_cm()); // Realiza medición de distancia
  if (distance == 0.00) {
    // El sensor reporta 0.0 cuando la medición sale del rango
    distance = MAX_DISTANCE;
  }
  distance = double(constrain(distance, 0.0, MAX_DISTANCE));
  // Uso una función lineal para obtener cercania
  inputOne = ((-2.0 / MAX_DISTANCE) * double(distance)) + 1.0;
  // Si está muy cerca del obstaculo, necesita más tiempo de reacción
  currentAction = ((inputOne + 1) * multiplier) + 1;

  // Obtener la dirección de obstaculo basandose en el angulo del servomotor
  inputTwo = map(servoAngle, MIN_ANGLE, MAX_ANGLE, -100, 100);
  inputTwo = double(constrain(inputTwo, -100.00, 100.00));
  inputTwo = inputTwo / 100.0;

  // Llama da a la red Feedforward con las entradas
  feedForward(1.0, inputOne, inputTwo);
  // Recuperar valores modificados por función feedForward
  int outputOne = round(abs(output[0]));
  int outputTwo = round(abs(output[1]));
  // Visualizar parametros en serial
  Serial.println("\nDistancia cm: " + String(distance));
  Serial.println("Entrada1: " + String(inputOne));
  Serial.println("Grados servo: " + String(servoAngle));
  Serial.println("Entrada2: " + String(inputTwo));
  Serial.println("Salida1: " + String(outputOne));
  Serial.println("Salida2: " + String(outputTwo));

  // Control de giro de motores con salidas de red neuronal
  int carSpeed = SPEED; // hacia adelante o atras
  // si es giro, necesita mayor fuerza en los motores
  if ((outputOne + outputTwo) == 1) {
    carSpeed = SPEED + 100;
  }
  analogWrite(MPA, carSpeed);
  analogWrite(MPB, carSpeed);
  // outputOne=1: 1*HIGH=HIGH / ouputOne=0: 0*HIGH=LOW
  digitalWrite(MDA, outputOne * HIGH);
  digitalWrite(MDB, outputTwo * HIGH);
  // LEDS frontales (prender o apagar) 
  flag_led_left = outputOne * 1;
  flag_led_right = outputTwo * 1;
}

void feedForward(double In1, double In2, double In3) {
  double TestInput[] = {In1, In2, In3};

  // Calcular las activaciones en las capas ocultas
  for (i = 0; i < HIDDEN_NODES; i++) {
    accum = 0; // HIDDEN_WEIGHTS[INPUT_NODES][i];
    for (j = 0; j < INPUT_NODES; j++) {
      accum += TestInput[j] * HIDDEN_WEIGHTS[j][i];
    }
    hidden[i] = tanh(accum); // Función de activación tangente hiperbólica
  }

  // Calcular activación y error en la capa de Salida
  for (i = 0; i < OUTPUT_NODES; i++) {
    accum = 0; // OUTPUT_WEIGHTS[HIDDEN_NODES][i];
    for (j = 0; j < HIDDEN_NODES; j++) {
      accum += hidden[j] * OUTPUT_WEIGHTS[j][i];
    }
    output[i] = tanh(accum); // Función de activación tangente hiperbólica
  }
}
