//********************************************************Autor: Nelson Castillo rev1.0 (agosto 2019)******************************************************************************************************
//********************************************************PENDULO PARA ANGULOS GRANDES(PENDULO INSTRUMENTADO)***************************************************
//**************************************************************************************************************************************************************
//************************* placa de desarrollo arduino nano conectado al modulo bluetooth (hc-05)*************************************************************
//*************************sistema alimentado con una bateria de 9 [v] para evitar la frecuencia de la red de 220[v]******************************************


#define STATUS_PIN                      13             //LED INDICADOR de estado del arduino
//************************************************************************************************
//******************************Registros del IMU MPU6050*****************************************
//************************************************************************************************
#define MPU_POWER_REG                   0x6B             //registro del modo de energia
#define MPU_POWER_CYCLE                 0b00000000       //
#define MPU_READ_TIMEOUT                2000             //
#define MPU_SAMP_FREQ                   250              //

#define MPU_GYRO_CFG_REG                0x1B             //registro del giroscopio
#define MPU_GYRO_READ_REG               0x43
#define MPU_GYRO_READ_REG_SIZE          6
#define MPU_GYRO_CFG_250DEG             0b00000000
#define MPU_GYRO_READINGSCALE_250DEG    131.0            //Escala 250 del giroscopio
#define MPU_GYRO_CFG_500DEG             0b00001000
#define MPU_GYRO_READINGSCALE_500DEG    65.5             //Escala 500 del giroscopio
#define MPU_GYRO_CFG_1000DEG            0b00010000
#define MPU_GYRO_READINGSCALE_1000DEG   32.8             //Escala 1000 del giroscopio
#define MPU_GYRO_CFG_2000DEG            0b00011000       
#define MPU_GYRO_READINGSCALE_2000DEG   16.4             //Escala 2000 del giroscopio
#define MPU_CALIBRATE_READING_NUM       2000             //NUMERO de muestras para la calibración

#define MPU_TEMP_READ_REG               0x41             //Registro de temperatura
#define MPU_TEMP_READ_REG_SIZE          2                //Numero de registros del sensor de temperatura

#define MPU_ACCEL_CFG_REG               0x1C             //Registro acelerometro
#define MPU_ACCEL_READ_REG              0x3B
#define MPU_ACCEL_READ_REG_SIZE         6                //Numero de registros del acelerometro SON 6 EN ORDEN valor high and low.
#define MPU_ACCEL_CFG_2G                0b00000000
#define MPU_ACCEL_READINGSCALE_2G       16384.0          //Escala 2g del acelerometro
#define MPU_ACCEL_CFG_4G                0b00001000
#define MPU_ACCEL_READINGSCALE_4G       8192.0           //Escala 4g del acelerometro
#define MPU_ACCEL_CFG_8G                0b00010000
#define MPU_ACCEL_READINGSCALE_8G       4096.0           //Escala 8g del acelerometro
#define MPU_ACCEL_CFG_16G               0b00011000
#define MPU_ACCEL_READINGSCALE_16G      2048.0           //Escala 16g del acelerometro

#define MPU1_I2C_ADDRESS                0b1101000        //adress del IMU
#define MPU2_I2C_ADDRESS                0b1101001
//*****************************************************************************************************
//***************************configuracion HX711*******************************************************
//*****************************************************************************************************
#define DOUT  A0                                         //pin data del hx711
#define CLK  A1                                          //pin clk del hx711
#include <Wire.h>                                        //libreria necesaria para protocolo i2c
#include "HX711.h"                                       //libreria necesaria para el modulo Hx711
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////-----------------------------------BLUETOOTH-------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>                              //libreria para el bluetooth hc05
SoftwareSerial BTserial(0,1);                            //RX,TX   linea del bluetooth
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////-------------------------------fin--bluetooth----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HX711 balanza(DOUT, CLK);                               //crear objeto balanza(hx711)

long val = 0;                                           //inicializar variable val en cero para valor adc del hx711
float gForceX, gForceY, gForceZ;                        //aceleracion x,y,z(imu)
float rotX, rotY, rotZ;                                 //variables velocidad angular x,y,z (IMU)
float calibX, calibY, calibZ;
//float pitch, roll, yaw;
float mpuTemp;                                           //variable de temperatura del IMU
long loopTimer = 0;
float t0;                                                //variable tiempo

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///---------------------------------------setup-----------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  t0=millis();                         //cronometro 
  Serial.begin(115200);                //(57600);
  BTserial.begin(115200);              //baud del modulo bluetooth
  //balanza.set_scale(55.1215);        //hx711
  //balanza.tare(10);                   //hx711
  pinMode(STATUS_PIN,OUTPUT);           //configuro led pin 13
  digitalWrite(STATUS_PIN, LOW);        //led apagado
  Wire.begin();
  SetupMPU();                          //llamo a la funcion que configura al mpu6050                     
  delay(3000);                         //pausa de 3000 milisegundos
  calibrateGyro();                     //llamo a la funcion de calibracion del giroscopio
  Serial.println("configuracion completa");  //envio mensaje por monitor serial configuración completada
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////-------------------------------función de configuración MPU------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetupMPU()
{
  Wire.beginTransmission(MPU1_I2C_ADDRESS);   //iniciar la comunicacion con el IMU
  Wire.write(MPU_POWER_REG);                   //escribir en los registros 
  Wire.write(MPU_POWER_CYCLE); 
  Wire.endTransmission();                     //finalizar la comunicación con el IMU
  Wire.beginTransmission(MPU1_I2C_ADDRESS);   //iniciar la comunicacion con el IMU 
  Wire.write(MPU_GYRO_CFG_REG);
  Wire.write(MPU_GYRO_CFG_1000DEG);         //configura escala de giros
  Wire.endTransmission();                    //finalizar la comunicación con el IMU
  Wire.beginTransmission(MPU1_I2C_ADDRESS);   //iniciar la comunicacion con el IMU
  Wire.write(MPU_ACCEL_CFG_REG);
  Wire.write(MPU_ACCEL_CFG_16G);            //configura escala de G
  Wire.endTransmission();                   //finalizar la comunicación con el IMU
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///--------------------------------LOOP----------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if(ReadMPU())
  {                          
    while(micros() - loopTimer < 4000);
    loopTimer = micros();
  }
}

bool ReadMPU()
{
  if(MPUReadAccel() && MPUReadGyro())
  {
//    calcRotation();
    printData();
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------------ACELEROMETROS(IMU)-------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MPUReadAccel()
{
  Wire.beginTransmission(MPU1_I2C_ADDRESS);         //inicializar la comunicacion con el IMU
  Wire.write(MPU_ACCEL_READ_REG);                   //escribir registro
  Wire.endTransmission();                           //finalizar la comunicacion
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_16G; //configura escala de G
  gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_16G;
  gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_16G;
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------------TEMPERATURA(IMU)----------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MPUReadTemp()
{
  Wire.beginTransmission(MPU1_I2C_ADDRESS);     //inicializar comunicación con el IMU
  Wire.write(MPU_TEMP_READ_REG);                //escribir registro
  Wire.endTransmission();                       //finalizar comunicación
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_TEMP_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_TEMP_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  mpuTemp = (long)(Wire.read() << 8 | Wire.read())/340 + 36.53;  //formula según datasheet del mpu6050
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////----------------------------------------GIROSCOPIOS(IMU)------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MPUReadGyro()
{
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  rotX = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_1000DEG;              //configura girosco
  rotY = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_1000DEG;
  rotZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_1000DEG;
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////-----------------------------------------CALIBRACION GIROSCOPIO---------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrateGyro()
{
  loopTimer = 0;
  
  digitalWrite(STATUS_PIN, HIGH);          //enciendo led 13
  Serial.println("Calibracion Gyroscopio");    //mensaje x serial 
  
  calibX = 0;                                  //valores iniciales en cero
  calibY = 0;
  calibZ = 0;
  
  
  for(int i=0; i<MPU_CALIBRATE_READING_NUM;i++)
  {
    if(MPUReadGyro())
    {
      calibX += rotX;
      calibY += rotY;
      calibZ += rotZ;

      //wait for the next sample cycle
      while(micros() - loopTimer < 4000);
      loopTimer = micros();
    }
    else
    {
      i--;
    }
  }
  calibX = calibX / MPU_CALIBRATE_READING_NUM;
  calibY = calibY / MPU_CALIBRATE_READING_NUM;
  calibZ = calibZ / MPU_CALIBRATE_READING_NUM;

  Serial.print("x: ");
  Serial.print(calibX);                                     
  Serial.print("y: ");
  Serial.print(calibY);
  Serial.print("z: ");
  Serial.println(calibZ);

  Serial.println("Calibracion exitosa");                     //mensaje por monitor serial
  
  digitalWrite(STATUS_PIN, LOW);                            //se apaga el led 13
}

//void calcRotation()
//{
//  pitch += (rotX - calibX) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//  roll += (rotY - calibY) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//  yaw += (rotZ - calibZ) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////------------------------------------DATOS A MOSTRAR X MONITOR SERIAL-------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printData() 
{
  val = balanza.read();
  //Serial.print("Temp (deg c) ");
  //Serial.print(mpuTemp);
  //Serial.print(" Gyro (deg/s)");
  Serial.print(millis()-t0);                     //DATO de tiempo en milisegundos.
  //Serial.print(" X=");
  Serial.print("\t");                            //espacio entre datos
  Serial.print(val);                             //(balanza.get_units(1),0);
  Serial.print("\t");                             // un tab del teclado (espacio en blanco)
  Serial.print(rotX - calibX);                   //velocidad angular en el eje x osea respecto del eje del pendulo
  //Serial.print(" Y=");
  //Serial.print("\t");
  //Serial.print(rotY - calibY);
  //Serial.print(" Z=");
  //Serial.print("\t");
  //Serial.println(rotZ - calibZ);
  //Serial.print(" Accel (g)");
  //Serial.print(" X=");
  //Serial.print("\t");
  //Serial.print(gForceX);
  //Serial.print(" Y=");
  Serial.print("\t");                            // un tab del teclado (espacio en blanco)
  Serial.print(gForceY*9.81);                    //aceleracion en metros /s2 eje y
  //Serial.print(" Z=");
  Serial.print("\t");                            // un tab del teclado (espacio en blanco)
  Serial.println(gForceZ*9.81);                  //aceleracion en metros/s2 eje z 
//  Serial.print(" RT:");
//  Serial.print(" Roll:");
//  Serial.print(roll);
//  Serial.print(" Pitch:");
//  Serial.print(pitch);
//  Serial.print(" Yaw:");
//  Serial.println(yaw);
  
}
