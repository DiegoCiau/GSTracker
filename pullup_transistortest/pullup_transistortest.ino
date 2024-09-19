#include "utilities.h"
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
XPowersPMU  PMU;

const byte in10=10;
const byte in11=11;
const byte BAT=12;
bool  level     = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(in10,INPUT_PULLUP);
  pinMode(in11,INPUT_PULLUP);
  pinMode(BAT,INPUT_PULLUP);

    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("Failed to initialize power.....");
        while (1) {
            delay(5000);
        }
    }

}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(in10)==LOW){
     Serial.println("Señal de ignición");
     PMU.setChargingLedMode(level ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
     level ^= 1;
     delay(1000); 
  }
  else {
     Serial.println("Sin ignición");
     delay(1000); 
  }

  if(digitalRead(in11)==LOW){
     Serial.println("desconexión de bateria"); 
     delay(1000); 
  }
    else {
     Serial.println("bateria conectada");
     delay(1000); 
  }

  if(digitalRead(BAT)==LOW){
     Serial.println("detección entrada pin 12");
     PMU.setChargingLedMode(level ? XPOWERS_CHG_LED_ON : XPOWERS_CHG_LED_OFF);
     level ^= 1;
     delay(1000); 
  }

}
