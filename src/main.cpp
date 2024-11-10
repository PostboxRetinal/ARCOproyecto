// Desarrollado por: Maryori Lasso y Sebastian Balanta

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial HC05(2);

#define RX 16
#define TX 17
#define KEY 26
#define BAUDS 38400

#define LED1 18
#define LED2 19
#define LED3 23
#define LED4 5
#define MOTOR_VENTILADOR 4 

byte data; // Variable para almacenar el dato recibido, lo mismo que char

int velocidadAnterior = 0; // Variable para almacenar la velocidad anterior del motor
int velocidad = 0; // Variable para almacenar la velocidad del motor

bool motorActivo = false; // Variable para saber si el motor está activo o no
bool cicloActivo = false; // Variable para saber si el ciclo automático está activo o no

unsigned long tiempoAnterior = 0; // Variable para almacenar el tiempo anterior, long porque son miliegundos

const int VMAX = 31; // Velocidad máxima del motor

void setup() {
  pinMode(KEY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(MOTOR_VENTILADOR, OUTPUT);

  digitalWrite(KEY, HIGH);
  
  Serial.begin(BAUDS, SERIAL_8N1);
  HC05.begin(BAUDS, SERIAL_8N1, RX, TX);
  
  Serial.println("Envia comandos AT para HC-05: ");
}

void blink(int LED){
  // Parpadeo de un LED en 125 ms
  digitalWrite(LED, HIGH );
  delay(125);
  digitalWrite(LED, LOW );
  delay(125);
}

void detenerMotor() {
  if (motorActivo) {
    Serial.println("ALERTA: Deteniendo motor");
    velocidadAnterior = velocidad;
    analogWrite(MOTOR_VENTILADOR, 0);
    digitalWrite(LED1, HIGH); // Encender LED1 para indicar que el motor se detuvo  
    motorActivo = false; // Motor detenido

  } else {
    Serial.println("ERROR: Motor detenido");
  }
  
  
}

void arrancarMotor() {
  if (!motorActivo) {
    motorActivo = true; // Motor activo
    if (velocidad == 0 && velocidadAnterior == 0) {
      velocidad = 1; // Si la velocidad es 0, se arranca en 1
      Serial.println("ALERTA: Arrancando motor en ralenti");
    } else {
      velocidad = velocidadAnterior; // Si la velocidad no es 0, se arranca en la velocidad anterior
      Serial.println("INFO: Reanudando motor a " + String(velocidad));
    }
    analogWrite(MOTOR_VENTILADOR, map(velocidad, 0, VMAX, 0, 255));
    digitalWrite(LED1, LOW);  // Apagar LED1 para indicar que el motor arrancó
    
  } else{
    Serial.println("ERROR: Motor ya está en marcha");
  }
  
}

void incrementarVelocidad() {
  if (velocidad < VMAX && motorActivo) {
    velocidad++;
    analogWrite(MOTOR_VENTILADOR, map(velocidad, 0, VMAX, 0, 255));
    velocidadAnterior = velocidad;
    Serial.println("INFO: V aumentada a " + String(velocidad));
    blink(LED2); // D2 titila al incrementar

  } else if (velocidad == VMAX && motorActivo){
    Serial.write("ERROR: Max RPMs alcanzadas\n");
  }

  if (!motorActivo){
    Serial.println("ERROR: Motor detenido");
  }
}

void reducirVelocidad() {
  if (velocidad > 0 && motorActivo) {
    velocidad--;
    analogWrite(MOTOR_VENTILADOR, map(velocidad, 0, VMAX, 0, 255));
    velocidadAnterior = velocidad;
    Serial.println("INFO: V reducida a " + String(velocidad));
    blink(LED4); // D4 titila al reducir
  }

  if (!motorActivo){
    Serial.println("ERROR: Motor detenido");
  }
}

void cicloAutomatico() {
  cicloActivo = true; // Activar ciclo automático
  const byte intervaloParpadeo = 125; // Intervalo de parpadeo del LED
  bool estadoLED = LOW;

  for (int i = 0; i <= VMAX; i++) {
    velocidad = i;
    analogWrite(MOTOR_VENTILADOR, map(velocidad, 0, VMAX, 0, 255));
    Serial.println("INFO: V aumentada a " + String(velocidad));

    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnterior >= intervaloParpadeo) {
      tiempoAnterior = tiempoActual;
      estadoLED = !estadoLED; // Cambiar el estado del LED
      digitalWrite(LED2, estadoLED); // Actualizar el estado del LED
    }

    delay(645); // Incremento en 20 s, para 31 VELOCIDADES
  }
  digitalWrite(LED2, LOW); // Apagar LED2

  Serial.println("ALERTA: Max RPMs alcanzada"); 
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) { // parpadear LED3 por 10 s
    blink(LED3); // D3 titila
  }
  
  Serial.println("Reduciendo RPMs...");
  for (int i = VMAX; i >= 0; i--) {
    velocidad = i;
    analogWrite(MOTOR_VENTILADOR, map(velocidad, 0, VMAX, 0, 255));
    Serial.println("INFO: V reducida a " + String(velocidad));

    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnterior >= intervaloParpadeo) {
      tiempoAnterior = tiempoActual;
      estadoLED = !estadoLED; // Cambiar el estado del LED
      digitalWrite(LED4, estadoLED); // Actualizar el estado del LED
    }

    delay(968); // Reducción en 30 s
  }
  velocidad = 0; // Velocidad en 0
  cicloActivo = false; // Desactivar ciclo automático
}

void menu(byte data, byte serial) {
  // Serial 1 - USB
  // Serial 2 - HC05
  // Aprovechanding que el código es el mismo, se puede hacer un menú para ambos con switch, ventaja de C++ ;)
  if (serial == 1) {
    switch (data) {
      case 'P':
        detenerMotor(); 
        Serial.println("Motor OFF");
        break;
      case 'A': 
        arrancarMotor(); 
        break;
      case 'I':
        Serial.println("ncrementando velocidad...");
        incrementarVelocidad(); 
        break;
      case 'R':
        Serial.println("educiendo velocidad...");
        reducirVelocidad();
        break;
      case 'C':
        Serial.println("iclo automático...");
        cicloAutomatico();
        Serial.println("Ciclo automático finalizado");
        break;
      default:
        break;
    }
  } else if (serial == 2) {
    switch (data) {
      case 'P':
        detenerMotor(); 
        HC05.write("Motor OFF");
        break;
      case 'A': 
        arrancarMotor(); 
        break;
      case 'I':
        HC05.write("ncrementando velocidad...");
        incrementarVelocidad(); 
        break;
      case 'R':
        HC05.write("educiendo velocidad...");
        reducirVelocidad();
        break;
      case 'C':
        HC05.write("iclo automático...");
        cicloAutomatico();
        HC05.write("Ciclo automático finalizado");
        break;
      default:
        break;
    }
  } else{
    Serial.println("ERROR: Terminal SERIAL inválida");
  }
}

void loop() {
  // Si se recibe un dato por el puerto serial, se lee y se envía por el puerto serial y por el HC05
  if (Serial.available() && !cicloActivo) {
    data = Serial.read();
    Serial.write(data);
    HC05.write(data);

    menu(data,1); // Se llama a la función menú usando el puerto serial 1
    blink(LED_BUILTIN);
  }

  if (HC05.available() && !cicloActivo) {
    data = HC05.read();
    HC05.write(data);
    Serial.write(data);

    menu(data,2); // Se llama a la función menú usando el puerto serial 2
    blink(LED_BUILTIN); // Parpadeo del LED INTEGRADO
  }

  if (motorActivo == false){  
      blink(LED1); // Parpadeo del LED1
  }
}