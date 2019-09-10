//****************************************************************************************************************************************************
//***************************************************************************************************************************************************
//************************************CODIGO CON SENSOR AUTONICS PARA MEDIR RPM RUEDA MOTOR BRUSHLESS*************************************************
//****************************************************************************************************************************************************
//****************************************************************************************************************************************************
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
#define MPU_CALIBRATE_READING_NUM       500

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

HX711 balanza2(10, 11);

long val = 0;
long val2 = 0;

//--------------------fin-codigo---dos--hx711--------------------------------------------------------
//************************************INICIO AUTONICS*********************************************************************************************
volatile byte rev;
volatile long dTime, timeold;
unsigned int  freq;
float rpm;
//************************************FIN VARIABLES AUTONICS***************************************************************************************
float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;
float calibX, calibY, calibZ;
//float pitch, roll, yaw;
float mpuTemp;
long loopTimer = 0;
float t0;
void setup() {
  t0=millis();
  Serial.begin(115200);//(57600);
  BTserial.begin(115200);
//**********************************************INCIO AUTONICS*****************************************************************************************
  attachInterrupt(1, revInterrupt, RISING);        //pin 3 is our interrupt
  dTime=0, rev=0, rpm=0, freq=0;
  timeold=0;
//**********************************************FIN AUTONICS*********************************************************************************************  
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
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_POWER_REG);
  Wire.write(MPU_POWER_CYCLE); 
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_GYRO_CFG_REG);
  Wire.write(MPU_GYRO_CFG_250DEG); 
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_CFG_REG);
  Wire.write(MPU_ACCEL_CFG_2G); 
  Wire.endTransmission();
}

void loop() {
  if(ReadMPU())
  {                          
    while(micros() - loopTimer < 10000);//4000);
    loopTimer = micros();
  }

//*****************************************************INICIO AUTONICS****************************************************************************************
//***********************************************************************************************************************************************************
  if (rev > 2)  //antes tenia 20
   {
    cli();                           //desabilita interrupcion(disable interrupts while we're calculating)
    if (dTime > 0)                   //check for timer overflow
    {
      rev-=1;                        //subtract one since the first revolution is not measured
      rpm=(60000000.*rev)/(dTime*1.000636);
      freq=rpm/60;
//      Serial.print(rev);
//      Serial.print(" ");
      //Serial.print(rpm);            //a bit of serial for the debugging (not really needed at this point, perhaps one day for some graphs)
//      Serial.print(" ");
//      Serial.println(dTime);
      rev=0;
    }
    sei();                          //habilita interrupcion (re-enable interrupts)
   } 
//****************************************************FIN AUTONICS*******************************************************************************************
//***********************************************************************************************************************************************************    
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
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_ACCEL_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  gForceX = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G; //aceleracion en valores de g
  gForceY = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  gForceZ = (long)(Wire.read() << 8 | Wire.read()) / MPU_ACCEL_READINGSCALE_2G;
  return true;
}

bool MPUReadTemp()
{
  Wire.beginTransmission(MPU1_I2C_ADDRESS);
  Wire.write(MPU_TEMP_READ_REG);
  Wire.endTransmission();
  Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_TEMP_READ_REG_SIZE);
  long timeout = millis() + MPU_READ_TIMEOUT;
  while(Wire.available() < MPU_TEMP_READ_REG_SIZE && timeout < millis());
  if (timeout <= millis()) return false;
  mpuTemp = (long)(Wire.read() << 8 | Wire.read())/340 + 36.53; //formula pag 30 register map
  return true;
}

bool MPUReadGyro()
{
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
  
  return true;
}


//**********************************INICIO AUTONICS******************************************************************************
//******************************************************************************************************************************
void revInterrupt (){
  if (rev == 0) 
  {
    timeold=micros();              //first measurement is unreliable since the interrupts were disabled
    rev++;
  }
  else 
  {
    dTime=(micros()-timeold);      //'micros()' is not incrementing while inside the interrupt so it should be safe like this right?
    rev++;
  }
}
//**************************************FIN AUTONICS****************************************************************************
//******************************************************************************************************************************
void calibrateGyro()
{
  loopTimer = 0;
  
  digitalWrite(STATUS_PIN, HIGH);
  //Serial.println("Calibrating Gyro");
  
  calibX = 0;
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
      while(micros() - loopTimer < 10000);//4000);
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

  //Serial.print("x: ");
  //Serial.print(calibX);
  //Serial.print("y: ");
  //Serial.print(calibY);
  //Serial.print("z: ");
  //Serial.println(calibZ);

  //Serial.println("Calibration Done.");
  
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
  val2 = balanza2.read();
  //Serial.print("Temp (deg c) ");
  //Serial.print(mpuTemp);
  //Serial.print(" Gyro (deg/s)");
  Serial.print(millis()-t0);
  //Serial.print(" X=");
  Serial.print("\t");
  Serial.print(val);
  //Serial.print(rotX - calibX);//    (deg/s)
  //Serial.print(" Y=");
  Serial.print("\t");
  Serial.print(val2);
  //Serial.print(rotY - calibY);  //    (deg/s)
  //Serial.print(" Z=");
  Serial.print("\t");
  Serial.print(rotZ - calibZ); //    (deg/s)
  //Serial.print(" Accel (g)");
  //Serial.print(" X=");
  Serial.print("\t");
  Serial.println(rpm);
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

