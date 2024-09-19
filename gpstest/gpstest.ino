 //codigo donde se muestran que pines se utilizaron para el shield de la lilygo para la lectura del GPS

#include <SoftwareSerial.h>

SoftwareSerial gps(44,43);

char dato=' ';

void setup()
{
 Serial.begin(115200);            
 gps.begin(9600); 
}


void loop()
{
  if(gps.available())
  {
    dato=gps.read();
    Serial.print(dato);
  }
}