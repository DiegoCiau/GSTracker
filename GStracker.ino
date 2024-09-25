#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
XPowersPMU PMU;
#define TINY_GSM_MODEM_SIM7080
//Pines para las entradas, salidas e ignición
#define ign 10
#define in0 11
#define in1 12
//#define BAT 12//utilizado para la conexión y desconexión de bateria(La segunda placa no utiliza este pin)
#define rele0 13
#define rele1 14
bool  level = false;

//Pines para la comunicación RX,TX
#define RX_PIN0 18  // Pin de recepción sensor ultrasónico
#define TX_PIN0 17  // Pin de transmisión sensor ultrasónico
#define RX_PIN1 44  // Pin de recepción
#define TX_PIN1 43  // Pin de transmisión
SoftwareSerial gps(RX_PIN1,TX_PIN1);
char dato=' ';//datos GPS
#define BAUD_RATE 9600
HardwareSerial SerialSensor(1);

//Pines para lecturas de voltaje y configuración
#define measureCarBat=2   //lectura de batería del auto
#define measureBackupBat=1//lectura bateria de backup


bool debug = false;
const int R1car = 4700;//we read the voltage acrosst this resistor (car resistors)
const int R2car = 43000;
const int R1carbackup = 5100;//we read the voltage acrosst this resistor (backup resistors)
const int R2carbackup = 1800;

const int VinPinbackup = 1; // GPIO1 Backup battery
const int VinPincar = 9; // GPIO2 car battery

// PWM settings
const int freq = 5000; // PWM frequency
const int resolution = 12; // PWM resolution (bits)
const int channelcar = 1; // PWM channelcar
const int channelbackup = 0; //PWM channelcar

float VB1;
float VBCar;
int mV1;
int mV2;

/*Pines para lecturas de voltaje (Segunda placa)
#define measureCarBat=8;
#define measureBackupBat=9;
*/

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(3000);

  // Inicialización del PMU y configuración del voltaje del módem
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("Failed to initialize power.....");
    while (1) {
      delay(5000);
    }
  }
  PMU.setDC3Voltage(3000); // Configura el voltaje del módem
  PMU.enableDC3(); // Habilita el canal de alimentación principal del módem
  // TS Pin detection must be disable, otherwise it cannot be charged
  PMU.disableTSPinMeasure();
    // Inicialización del puerto serial para el módem
  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);

  pinMode(ign,INPUT_PULLUP);
  pinMode(in0,INPUT_PULLUP);
  pinMode(in1,INPUT_PULLUP);
  pinMode(rele0, OUTPUT);
  pinMode(rele1, OUTPUT);
  ledcAttach(channelcar, freq, resolution);
  ledcAttach(channelbackup, freq, resolution);
  
}

void loop() {
  readVoltage();
  Serial.print("VB1 Voltage: ");
  Serial.print(VB1); // Convert millivolts to volts
  Serial.println("V ");
  Serial.print("VBCar Voltage: ");
  Serial.print(VBCar); // Convert millivolts to volts
  Serial.println("V ");
  
  readPinouts();
  MonitorSerialRele();
  
  if(gps.available()){
    dato=gps.read();
    Serial.print(dato);
  }
  //RelayaSMS();

 /* digitalWrite(rele0, HIGH); // Enciende el LED
  Serial.println("Relé encendido");
  delay(3000); 
  digitalWrite(rele0, LOW); // Apaga el LED
  Serial.println("Relé apagado");
  */

if (SerialSensor.available()){
    // Lee los datos entrantes
    String DataSensor = SerialSensor.readString();
    // Imprime los datos recibidos en el puerto serial de depuración
    Serial.print("Received: ");
    Serial.println(DataSensor);
}
  delay(10);

}


void readVoltage()
{
  uint32_t voltage_mV1 = analogReadMilliVolts(VinPinbackup); // Read the voltage in millivolts
  uint32_t voltage_mV2 = analogReadMilliVolts(VinPincar); // Read the voltage in millivolts
  if(debug)
  {
    maxVoltage();
    Serial.print("PinVoltage: ");
    Serial.print(voltage_mV1);
    Serial.println("mV");
    Serial.println();//this adds a new line
    Serial.print("PinVoltage2: ");
    Serial.print(voltage_mV2);
    Serial.println("mV");
    Serial.println();//this adds a new line
  }
  mV1 = voltage_mV1;
  VB1 = (((float) voltage_mV1) / 1000.0)  * (1 + (float)R2carbackup/(float)R1carbackup);
  mV2 = voltage_mV2;
  VBCar = (((float) voltage_mV2) / 1000.0)  * (1 + (float)R2car/(float)R1car);
}

/*
prints maximum volage
*/
void maxVoltage()
{
  float maxVoltage = ( 3.1)  * (1 + (float)R2car/(float)R1car);
    Serial.print("****Maximum Voltage: ");
    Serial.print(maxVoltage);
    Serial.println("V");
  float maxVoltage2 = ( 3.1)  * (1 + (float)R2carbackup/(float)R1carbackup);
    Serial.print("****Maximum Voltage: ");
    Serial.print(maxVoltage2);
    Serial.println("V");
}

void readPinouts()
{
if(digitalRead(ign)==LOW){
     Serial.println("Señal de ignición");
     delay(1000); 
  }
  else {
     Serial.println("Sin ignición");
     delay(1000); 
  }

  if(digitalRead(in0)==LOW){
     Serial.println("detección entrada pin 11"); 
     delay(1000); 
  }
 
  if(digitalRead(in1)==LOW){
     Serial.println("detección entrada pin 12");
     delay(1000); 
  }
}
/*
void RelaySMS()
{
  
  if (Serial1.available()) {
    //Serial.write(Serial1.read());
    valor = Serial1.readString(); //Guardar en la var valor el sms que recibe el Arduino
    Serial.println("Nuevo SMS: "+ valor); //Imprime ese SMS en el monitor Serial
  }

  if(valor.indexOf("ON")>=0){  //Si la var valor tiene la palabra ON hace esto:
    digitalWrite(rele0, HIGH);    //Se enciende el pin 13.
    Serial.println("Relé activado");  //Immprime el mensaje
    valor="";
    delay(15000);
  }
  if(valor.indexOf("OFF")>=0){
    digitalWrite(rele0, LOW);    //Se apaga el pin 13.
    Serial.println("Relé desactivado");  //Immprime el mensaje
    valor = "";
    delay(15000);
  }
}*/

void MonitorSerialRele(){
// Verifica si hay datos disponibles en el monitor serial
  if (Serial.available() > 0) {
    // Lee lo que se haya ingresado en el monitor serial
    String command = Serial.readStringUntil('\n');  // Lee hasta un salto de línea

    // Quita los espacios en blanco o saltos de línea extra
    command.trim();

    // Verifica si el comando es 'ON' para encender el relé
    if (command.equalsIgnoreCase("ON")) {
      digitalWrite(rele0, HIGH);  // Enciende el relé
      Serial.println("Relé encendido");
    }
    // Verifica si el comando es 'OFF' para apagar el relé
    else if (command.equalsIgnoreCase("OFF")) {
      digitalWrite(rele0, LOW);  // Apaga el relé
      Serial.println("Relé apagado");
    } else {
      Serial.println("Comando no reconocido. Usa 'ON' o 'OFF'.");
    }
  }
  delay(100); 
}