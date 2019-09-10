/*  
 *  Controlando la velocidad de un DC Fan con un Control Remoto de TV
 *  
 *  Modificado IRrecvDemo ejemplo de Ken Shirriff IRremote Library
 *  Ken Shirriff
 *  
 *  
 */
// utilizo un control remoto LG para controlar el motor rueda lab giroscopico
#include <IRremote.h>                //libreria para el sensor IR
int RECV_PIN = 8;                   // IR Receiver - Arduino Pin Numero 8
int pwmPin = 6;                    // Arduino Pin Numero 6 a la compuerta del Mosfet
int pwmValue;
IRrecv irrecv(RECV_PIN);
decode_results results;
 
void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();             // Inicializo el receiver
  pinMode( pwmPin, OUTPUT);  
  pwmValue = 0;                  // Empiezo el programa con el motor apagado
}
 
void loop() {
  
  if (irrecv.decode(&results)) {   
    
    analogWrite(pwmPin, pwmValue);
  
    if (results.value == 0x20DF0DF2) { // Boton PLAY
        pwmValue = 255;       // 100% Duty Cycle | Maxima Velocidad
        }
    if (results.value == 0x20DF8D72) { // Boton STOP
        pwmValue = 0;         // 0% Duty Cycke | Apago el motor
        }
    if (results.value == 0x20DF718E) { // Boton NEXT
        pwmValue = 128;       // 50% Duty Cycke | Velocidad media
        }
   if (results.value == 0x20DF00FF) { // Boton Ch+
        if(pwmValue <= 245){  
        pwmValue = pwmValue + 5;     // Incremento el Duty Cycle de la señal PWM
        delay(20);     
      }   
    }
    if (results.value == 0x20DF807F) { // Boton Ch-
      if(pwmValue >= 20){
        pwmValue = pwmValue - 5; // Reduzco el Duty Cycle de la señal PWM
        delay(20);    
      }
    }   
  Serial.print(pwmValue);
  Serial.print(" ");
  Serial.println(results.value, HEX);
   
  irrecv.resume(); // Recibo el proximo valor
  }
  delay(100);
}
 

