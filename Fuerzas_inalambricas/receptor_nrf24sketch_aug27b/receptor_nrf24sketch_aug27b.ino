// ###################################################################//
//----Autor Nelson Castillo
//-----------------Codigo del receptor-------------------------------
//Conexiones ARDUINO UNO/NANO, bus SPI:
//** MOSI -> D11
//** MISO -> D12
//** CLK  -> D13
//** CE   -> D9
//** CSN  -> D10
//Alimentacíon del modulo adaptador que vende max electronica
//Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
////##########################################################
// 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "HX711.h"     //libreria hx711
#define DOUT A1        //pin data hx711
#define CLK A0         //pin reloj hx711
float t0;
HX711 balanza(DOUT, CLK); //creo objeto
long val=0; 
//long peso[1];

#define CE_PIN 9
#define CSN_PIN 10
 
RF24 radio(CE_PIN, CSN_PIN); // creamos el objeto radio (NRF24L01)
const byte address[6] = "00001";   //tuberia de comunicacion
float peso;     //variable a medir peso
void setup() { 
  Serial.begin(9600);       // inicializamos el puerto serie
  Serial.println("#################################");
  Serial.println(" Escuchando datos de radio");    //mensaje x mo serial
  Serial.println("#################################");
  radio.begin();                      // inicializamos el NRF24L01 
  radio.openReadingPipe(0, address); // Abrimos un canal de lectura
  radio.setDataRate(RF24_250KBPS);   //tasa bits per second
  radio.setPALevel(RF24_PA_MIN);     //potencia
  radio.startListening();            // empezamos a escuchar por el canal
}
 
void loop() {
  if (radio.available()) {          //si hay datos disponibles
    
    //char peso[32] = "";
    radio.read(&peso, sizeof(peso));  //leer lo que llega
    Serial.println(peso);              // imprimir x m serial
 
  }else{
    //Serial.println("No hay datos de radio disponibles");//mensaje x m serial
 }
 //delay(100);      // retardo de 100 milisegundos
}
