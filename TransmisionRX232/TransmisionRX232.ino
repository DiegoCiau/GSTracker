#include <HardwareSerial.h>

// Configuración de los pines UART
#define RX_PIN 18  // Pin de recepción
#define TX_PIN 17  // Pin de transmisión

// Configuración de la velocidad de transmisión
#define BAUD_RATE 9600

HardwareSerial MySerial(1);  // Utiliza UART1

void setup() {
  MySerial.begin(BAUD_RATE, SERIAL_8O1, RX_PIN, TX_PIN);

  Serial.begin(115200);
  Serial.println("ESP32 UART Receiver Initialized");
}

void loop() {
  // Verifica si hay datos disponibles en el puerto UART
  if (MySerial.available()) {
    // Lee los datos entrantes
    String receivedData = MySerial.readString();

    // Imprime los datos recibidos en el puerto serial de depuración
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
  
  // Pequeña demora para evitar saturar el procesador
  delay(10);
}
