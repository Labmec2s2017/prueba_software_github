//* ##############################################################
//autor Nelson Castillo
///////////COdigo del transmisor(emisor)---------------------------------
//Conexiones ARDUINO UNO/NANO, bus SPI:
//** MOSI -> D11
//** MISO -> D12
//** CLK  -> D13
//** CE   -> D9
//** CSN  -> D10
//Alimentacíon del modulo usar adaptador
//Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
//* ##############################################################
//*/
 
#include <SPI.h>      //libreria para spi
#include <nRF24L01.h>
#include <RF24.h>
#include "HX711.h"     //libreria hx711
#define CE_PIN 9        //pin CE arduino nano
#define CSN_PIN 10      //pin csn arduino nano
#define DOUT A1        //pin data hx711
#define CLK A0         //pin reloj hx711
float t0;
HX711 balanza(DOUT, CLK); //se crea el objeto
long val=0; 
float peso;//[1];      //variable a medir peso un flotante
const byte address[6] = "00001";   //se crea una tuberia de comunicacion
 
RF24 radio(CE_PIN, CSN_PIN); // creamos el objeto radio (NRF24L01)
 
void setup() {
  t0=millis();
  Serial.begin(9600);              //iniciar comunicacion serial
  radio.begin();                    // inicializamos el NRF24L01 
  radio.setDataRate(RF24_250KBPS);   //tasa de bits per second
  radio.openWritingPipe(address);   // Abrimos un canal de escritura
  radio.setPALevel(RF24_PA_MIN);    //configurar power nivel minimo
  radio.stopListening();            // Deja de escuchar para que podamos transmitir.
    
}
void loop() {
  peso = balanza.read();           //leemos con la libreria hx711
  //peso[0]= balanza.read();
  //Serial.print(millis()-t0);
  //Serial.print("\t");
  //Serial.println(val);
  //const char text[] = " nRF24L01 chile";
  //radio.write(&text, sizeof(text));             // Manda mensaje
  radio.write(&peso, sizeof(peso));      //escribimos x el canal del nrf24l01 
  Serial.println(peso);          // escribimos x mo serial
  //delay(1000);               // retardo de 1000 milisegundos
}
