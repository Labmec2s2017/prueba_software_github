//************************************************************************************************************************************************
//*******************************************************Codigo para el ensayo del efecto coriolis************************************************+
//************************************************************************************************************************************************
#define STATUS_PIN                      13

#define MPU_POWER_REG                   0x6B
#define MPU_POWER_CYCLE                 0b00000000
#define MPU_READ_TIMEOUT                2000
#define MPU_SAMP_FREQ                   250

#define MPU_GYRO_CFG_REG                0x1B
#define MPU_GYRO_READ_REG               0x43
#define MPU_GYRO_READ_REG_SIZE          6
#define MPU_GYRO_CFG_250DEG             0b00000000
#define MPU_GYRO_READINGSCALE_250DEG    131.0
#define MPU_GYRO_CFG_500DEG             0b00001000
#define MPU_GYRO_READINGSCALE_500DEG    65.5
#define MPU_GYRO_CFG_1000DEG            0b00010000
#define MPU_GYRO_READINGSCALE_1000DEG   32.8
#define MPU_GYRO_CFG_2000DEG            0b00011000
#define MPU_GYRO_READINGSCALE_2000DEG   16.4
#define MPU_CALIBRATE_READING_NUM       1000            //valor para calibracion giroscopio

#define MPU_TEMP_READ_REG               0x41
#define MPU_TEMP_READ_REG_SIZE          2

#define MPU_ACCEL_CFG_REG               0x1C
#define MPU_ACCEL_READ_REG              0x3B
#define MPU_ACCEL_READ_REG_SIZE         6
#define MPU_ACCEL_CFG_2G                0b00000000
#define MPU_ACCEL_READINGSCALE_2G       16384.0
#define MPU_ACCEL_CFG_4G                0b00001000
#define MPU_ACCEL_READINGSCALE_4G       8192.0
#define MPU_ACCEL_CFG_8G                0b00010000
#define MPU_ACCEL_READINGSCALE_8G       4096.0
#define MPU_ACCEL_CFG_16G               0b00011000
#define MPU_ACCEL_READINGSCALE_16G      2048.0

#define MPU1_I2C_ADDRESS                0b1101000
#define MPU2_I2C_ADDRESS                0b1101001

const int MPU2 = 0x69, MPU1=0x68;  
#include <Wire.h>      //libreria necesaria para protocolo i2c
/////-----------------------------bluetooh----------------------------
#include <SoftwareSerial.h> //libreria para el bluetooth hc05
SoftwareSerial BTserial(0,1);//RX,TX   linea del bluetooth
//-------------------------------fin--bluetooth----------------------
//--------------------inicio-codigo---dos--hx711-------------------------------------------------------------------
#include "HX711.h"

#define DOUT  A1
#define CLK  A0


HX711 balanza(DOUT, CLK);

//HX711 balanza2(10, 11);

long val = 0;
//long val2 = 0;
//--------------------fin-codigo---dos--hx711--------------------------------------------------------
float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ,rotX2,rotZ2,rotY2;
float calibX, calibY, calibZ;
float calibX2, calibY2, calibZ2;
//float pitch, roll, yaw;
float mpuTemp;
long loopTimer = 0;
float t0;
long timeout;
void setup() {
  //balanza.set_scale(720);//(989.497);   //nuevo metodo de calibracion de balanza
  //balanza.tare();//(10);             //nuevo metodo de calibracion de balanza
  t0=millis();
  Serial.begin(115200);//(57600);
  BTserial.begin(115200);
  pinMode(STATUS_PIN,OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  Wire.begin();
  SetupMPU();
  delay(3000);
  calibrateGyro();
  //Serial.println("configuracion completa");
  
}

void SetupMPU()
{
//*************************************************************************MPU1*********************************************************************************************
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_POWER_REG);
  Wire.write(MPU_POWER_CYCLE); 
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_CFG_REG);
  Wire.write(MPU_GYRO_CFG_250DEG);                    //configuracion rango giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_CFG_REG);
  Wire.write(MPU_ACCEL_CFG_2G);                       //configuracion rango acelerometro
  Wire.endTransmission();
//**************************************************************************************************************************************************************************
//************************************************************************MPU2**********************************************************************************************
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_POWER_REG);
  Wire.write(MPU_POWER_CYCLE); 
  Wire.endTransmission();
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_GYRO_CFG_REG);
  Wire.write(MPU_GYRO_CFG_250DEG);                  //configuracion rango giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_CFG_REG);
  Wire.write(MPU_ACCEL_CFG_2G);                     //configuracion rango acelerometro
  Wire.endTransmission();  
}

void loop() {
  if(ReadMPU())
  {                          
    while(micros() - loopTimer < 4000);//4000);
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

bool MPUReadAccel()
{
//*********************************************Lectura de aceleracion**********************************************************************
//*****************************************************mpu1*************************************************************************
//**********************************************************************************************************************************
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G; //rango de aceleracion en valores de g
  gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  return true;
//****************************************************mpu2************************************************************************
//********************************************************************************************************************************
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU2_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  timeout = millis() + MPU_READ_TIMEOUT;//long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G; //rango de aceleracion en valores de g
  gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  return true;
}

bool MPUReadTemp()
{
//*******************************************Lectura de temperatura***************************************************************************************
//**************************************************mpu1*******************************************************************
//*************************************************************************************************************************
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_TEMP_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_TEMP_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_TEMP_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  mpuTemp = (long)(Wire.read() << 8 | Wire.read())/340 + 36.53; //formula pag 30 register map
  return true;
//************************************************mpu2***********************************************************************
//***************************************************************************************************************************
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_TEMP_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU2_I2C_ADDRESS, MPU_TEMP_READ_REG_SIZE);
   timeout = millis() + MPU_READ_TIMEOUT;//long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_TEMP_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  mpuTemp = (long)(Wire.read() << 8 | Wire.read())/340 + 36.53; //formula pag 30 register map
  return true;
}

bool MPUReadGyro()
{
//**********************************************Lectura de giroscopio**********************************************************************  
//********************************************************mpu1******************************************************************
//******************************************************************************************************************************
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  rotX = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;  //vel ang en grados/seg
  rotY = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;
  rotZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;
  
  //return true;
//*****************************************************mpu2**********************************************************************
//*******************************************************************************************************************************
  Wire.beginTransmission(MPU2_I2C_ADDRESS);
  Wire.write(MPU_GYRO_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU2_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);
  timeout = millis() + MPU_READ_TIMEOUT;//long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  rotX2 = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;  //vel ang en grados/seg
  rotY2 = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;
  rotZ2 = (long)(Wire.read() << 8 | Wire.read()) / MPU_GYRO_READINGSCALE_250DEG;
  
  return true;
}

void calibrateGyro()
{
//*****************************************************************************************************  
//*************************************Funcion de calibracion giroscopios******************************
//*****************************************************************************************************  
  loopTimer = 0;
  
  digitalWrite(STATUS_PIN, HIGH);
  Serial.println("Calibracion Gyroscopio");
  
  calibX = 0;
  calibY = 0;
  calibZ = 0;
  calibX2 = 0;
  calibY2 = 0;
  calibZ2 = 0;
  
  
  for(int i=0; i<MPU_CALIBRATE_READING_NUM;i++)
  {
    if(MPUReadGyro())
    {
      calibX += rotX;
      calibX2 += rotX2;
      calibY += rotY;
      calibY2 += rotY2;
      calibZ += rotZ;
      calibZ2 += rotZ2;

      //wait for the next sample cycle
      while(micros() - loopTimer < 4000);
      loopTimer = micros();
    }
    else
    {
      i--;
    }
  }
//*******************************mpu1****************************  
  calibX = calibX / MPU_CALIBRATE_READING_NUM;
  calibY = calibY / MPU_CALIBRATE_READING_NUM;
  calibZ = calibZ / MPU_CALIBRATE_READING_NUM;

//********************************mpu2****************************
  calibX2 = calibX2 / MPU_CALIBRATE_READING_NUM;
  calibY2 = calibY2 / MPU_CALIBRATE_READING_NUM;
  calibZ2 = calibZ2 / MPU_CALIBRATE_READING_NUM;

  Serial.print("x: ");
  Serial.print(calibX);
  Serial.print("y: ");
  Serial.print(calibY);
  Serial.print("z: ");
  Serial.println(calibZ);

  Serial.println("Calibration Done.");

//**************************************mpu2****************************************************
  Serial.print("x: ");
  Serial.print(calibX2);
  Serial.print("y: ");
  Serial.print(calibY2);
  Serial.print("z: ");
  Serial.println(calibZ2);

  Serial.println("Calibration Done.");
  
  digitalWrite(STATUS_PIN, LOW);
}

//void calcRotation()
//{
//  pitch += (rotX - calibX) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//  roll += (rotY - calibY) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//  yaw += (rotZ - calibZ) * (1/(MPU_SAMP_FREQ/MPU_GYRO_READINGSCALE_250DEG));
//}

void printData() 
{
  val = balanza.read();
  //val2 = balanza2.read();
  //Serial.print("Temp (deg c) ");
  //Serial.print(mpuTemp);
  //Serial.print(" Gyro (deg/s)");
  Serial.print(millis()-t0);  //****************cronometro
  //Serial.print(" X=");
  Serial.print("\t");
  Serial.print(val);//(balanza.get_units(1),0); // nuevo metodo de calibracion de la balanza
  //Serial.print(rotX - calibX);//    (deg/s)
  //Serial.print(" Y=");
  Serial.print("\t");
  //Serial.print(val2);
  Serial.print(rotX - calibX);//    (deg/s));
  //Serial.print(rotY - calibY);  //    (deg/s)
  //Serial.print(" Z=");
  Serial.print("\t");
  Serial.print(rotZ - calibZ); //    (deg/s)
  //Serial.print(" Accel (g)");
  //Serial.print(" X=");
  Serial.print("\t");
  Serial.print(rotX2-calibX2);
  Serial.print("\t");
  Serial.println(rotZ2 - calibZ2); //    (deg/s)
  //Serial.print(gForceX);
  //Serial.print(" Y=");
  //Serial.print("\t");
  //Serial.print(gForceY);
  //Serial.print(" Z=");
  //Serial.print("\t");
  //Serial.println(gForceZ);
//  Serial.print(" RT:");
//  Serial.print(" Roll:");
//  Serial.print(roll);
//  Serial.print(" Pitch:");
//  Serial.print(pitch);
//  Serial.print(" Yaw:");
//  Serial.println(yaw);
  
}
