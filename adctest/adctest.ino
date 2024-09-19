
bool debug = false;
//const int R1 = 4700;//we read the voltage acrosst this resistor ()
//const int R2 = 43000;
const int R1 = 5100;//we read the voltage acrosst this resistor (backup resistors)
const int R2 = 1800;

const int VinPin = 1; // GPIO1 Backup battery
//const int VinPin = 2; // GPIO2 car battery

// PWM settings
const int freq = 5000; // PWM frequency
const int resolution = 12; // PWM resolution (bits)
const int channel = 0; // PWM channel

float VB1;
int mV;

void setup() {
  Serial.begin(115200);
  // Configure PWM
  ledcAttach(channel, freq, resolution);
  //calculatResistor(R1, 32);//10000 ohm and 35.0V

}



void loop() {
   readVoltage();

  Serial.print("VB1 Voltage: ");
  Serial.print(VB1); // Convert millivolts to volts
  Serial.println("V ");


  delay(500);
}

void readVoltage()
{
  uint32_t voltage_mV = analogReadMilliVolts(VinPin); // Read the voltage in millivolts
  if(debug)
  {
    maxVoltage();
    Serial.print("PinVoltage: ");
    Serial.print(voltage_mV);
    Serial.println("mV");
    Serial.println();//this adds a new line
  }
  mV = voltage_mV;
  VB1 = (((float) voltage_mV) / 1000.0)  * (1 + (float)R2/(float)R1);
}

/*
prints maximum volage
*/
void maxVoltage()
{
  float maxVoltage = ( 3.1)  * (1 + (float)R2/(float)R1);
    Serial.print("****Maximum Voltage: ");
    Serial.print(maxVoltage);
    Serial.println("V");

}//maxVoltage() end

void calculatResistor(int r1, float voltage)
{
    Serial.print("Calculating R2 when R1 is :");
    Serial.print(r1);
    Serial.print(" Ohms and Maximum Input Voltage is ");  
    Serial.print(voltage);  
    Serial.println("V");  
    Serial.print("***** R2 Should Be ");  
    float r2 = (voltage - 3.1)/ (3.1/ (float)r1);
    Serial.print(r2 / 1000.0);  
    Serial.println(" kilo Ohms");  
    while(1);
}
