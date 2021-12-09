#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Servo.h>
#include <NewPing.h>
// Add support for OTA
#include "config.h"
#include "ESP8266_Utils.hpp"
#include "ESP8266_Utils_OTA.hpp"

// Ultrasonic
#define TRIGGER_PIN 12     // D6
#define ECHO_PIN 12        // D6 
#define MAX_DISTANCE 50.00 // Distancia máxima
// Servomotor
#define SERVO 13
// Motor pins
#define MPA 5
#define MPB 4
#define MDA 0
#define MDB 2
// Led
#define LED_RIGHT 15
#define LED_LEFT 14

// Instantiate objects
Servo servoMotor;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/******************************************************************
   Network Configuration
 ******************************************************************/
const int InputNodes = 3;  // incluye neurona de BIAS
const int HiddenNodes = 4; //incluye neurona de BIAS
const int OutputNodes = 2;
int i, j;
double Accum;
double Hidden[HiddenNodes];
double Output[OutputNodes];
// See Red Neuronal.ipynb
float HiddenWeights[3][4] = {{0.04440296939274621, 1.2575842236118704, -1.4228826803616692, -0.9443902694433634}, {1.787373799878464, -0.2881988744916164, -3.480517880453659, -0.6622089396247408}, {0.5137921009738449, 0.17101392965012976, 1.6724646296450323, 0.1671049832045934}};
float OutputWeights[4][2] = {{-2.2239559511040072, -0.12001955844317441}, {1.698740776004665, 0.4488962776209059}, {-1.0125387310778333, 1.9345850845199677}, {0.2162354268453128, -1.868337340934714}};

/******************************************************************
   End Network Configuration
 ******************************************************************/

void stop()
{
  digitalWrite(MPA, LOW); //Desactivamos los motores
  digitalWrite(MPB, LOW); //Desactivamos los motores
  Serial.println("Stop!");
}

void setup()
{
  // Enable OTA support
  ConnectWiFi_STA();
  InitOTA();

  servoMotor.attach(SERVO); // attach servoMotor on pin 13 to servoMotor object
  Serial.begin(115200);
  pinMode(MDA, OUTPUT);
  pinMode(MDB, OUTPUT);
  pinMode(MPA, OUTPUT);
  pinMode(MPB, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  stop(); // Call function to stop all motors
  servoMotor.write(90); //posición inicial en el centro
  delay(5000);
}

unsigned long previousMillis = 0; // para medir ciclos de tiempo
const long interval = 25;         // intervalos cada x milisegundos (default: 25)
int grados_servo = 90;            // posicion del servoMotor que mueve el sensor ultrasonico
bool clockwise = true;            // sentido de giro del servoMotor
const long ANGULO_MIN = 30;
const long ANGULO_MAX = 150;
double distanciaMaxima = MAX_DISTANCE; // distancia de lejania desde la que empieza a actuar la NN
int incrementos = 9;                   // incrementos por ciclo de posicion del servoMotor
int accionEnCurso = 1;                 // cantidad de ciclos ejecutando una accion
int multiplicador = 500 / interval;    // multiplica la cant de ciclos para dar tiempo a que el coche pueda girar
const int SPEED = 350;                 // velocidad del coche de las 4 ruedas a la vez.

void loop()
{
  ArduinoOTA.handle();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    /******************************************************************
    MANEJAR GIRO de SERVO
  ******************************************************************/
    if (grados_servo <= ANGULO_MIN || grados_servo >= ANGULO_MAX)
    {
      clockwise = !clockwise; // cambio de sentido
      grados_servo = constrain(grados_servo, ANGULO_MIN, ANGULO_MAX);
    }
    if (clockwise)
      grados_servo = grados_servo + incrementos;
    else
      grados_servo = grados_servo - incrementos;

    if (accionEnCurso > 0)
    {
      accionEnCurso = accionEnCurso - 1;
    }
    else
    {
      conducir(); // Llama da a función de coste
    }
    servoMotor.write(grados_servo);
  }
}

//USA LA RED NEURONAL YA ENTRENADA
void conducir()
{
  double TestInput[] = {0, 0, 0};
  double entrada1 = 0, entrada2 = 0;

  /******************************************************************
    OBTENER DISTANCIA DEL SENSOR
  ******************************************************************/
  double distance = double(sonar.ping_cm());
  if (distance == 0.00)
    distance = distanciaMaxima;
  distance = double(constrain(distance, 0.0, distanciaMaxima));
  entrada1 = ((-2.0 / distanciaMaxima) * double(distance)) + 1.0; //uso una funcion lineal para obtener cercania
  accionEnCurso = ((entrada1 + 1) * multiplicador) + 1;           // si esta muy cerca del obstaculo, necesitia mas tiempo de reaccion

  /******************************************************************
    OBTENER DIRECCION SEGUN ANGULO DEL SERVO
  ******************************************************************/
  entrada2 = map(grados_servo, ANGULO_MIN, ANGULO_MAX, -100, 100);
  entrada2 = double(constrain(entrada2, -100.00, 100.00));

  /******************************************************************
    LLAMAMOS A LA RED FEEDFORWARD CON LAS ENTRADAS
  ******************************************************************/
  Serial.print("\nDistancia cm: ");
  Serial.println(distance);
  Serial.print("Entrada1: ");
  Serial.println(entrada1);
  Serial.print("Grados servo: ");
  Serial.println(grados_servo);
  Serial.print("Entrada2: ");
  Serial.println(entrada2 / 100.0);

  TestInput[0] = 1.0; //BIAS UNIT
  TestInput[1] = entrada1;
  TestInput[2] = entrada2 / 100.0;

  InputToOutput(TestInput[0], TestInput[1], TestInput[2]); //INPUT to ANN to obtain OUTPUT

  int out1 = round(abs(Output[0]));
  int out2 = round(abs(Output[1]));
  Serial.print("Salida1:");
  Serial.println(out1);
  Serial.print("Salida2:");
  Serial.println(out2);

  /******************************************************************
    IMPULSAR MOTORES CON LA SALIDA DE LA RED
  ******************************************************************/
  int carSpeed = SPEED; //hacia adelante o atras
  if ((out1 + out2) == 1)
  { // si es giro, necesita doble fuerza los motores
    carSpeed = SPEED + 100;
  }
  analogWrite(MPA, carSpeed);
  analogWrite(MPB, carSpeed);
  digitalWrite(MDA, out1 * HIGH);
  digitalWrite(MDB, out2 * HIGH);
}

void InputToOutput(double In1, double In2, double In3)
{
  double TestInput[] = {0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;

  /******************************************************************
    Calcular las activaciones en las capas ocultas
  ******************************************************************/

  for (i = 0; i < HiddenNodes; i++)
  {
    Accum = 0; //HiddenWeights[InputNodes][i];
    for (j = 0; j < InputNodes; j++)
    {
      Accum += TestInput[j] * HiddenWeights[j][i];
    }
    //Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ; //Sigmoid
    Hidden[i] = tanh(Accum); //tanh
  }

  /******************************************************************
    Calcular activacion y error en la capa de Salida
  ******************************************************************/

  for (i = 0; i < OutputNodes; i++)
  {
    Accum = 0; //OutputWeights[HiddenNodes][i];
    for (j = 0; j < HiddenNodes; j++)
    {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = tanh(Accum); //tanh
  }
}
