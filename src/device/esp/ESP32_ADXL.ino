#include <stdint.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/*  DEFINES */
#define SERIAL_BAUDRATE 921600
#define BUFFER_SIZE     1000
#define INIT_DELAY_MS   500

/* TYPES */
typedef struct {
  float x;
  float y;
  float z;
} AccData;

/* GLOBAL VARS */
portMUX_TYPE mux  = portMUX_INITIALIZER_UNLOCKED;

Adafruit_ADXL345_Unified  accADXL = Adafruit_ADXL345_Unified(12345);

AccData ringBuffer[BUFFER_SIZE] = {0};

volatile uint16_t bufferHead = 0;  
volatile uint16_t bufferTail = 0;

unsigned int  readingsCount     = 0;
unsigned long lastTime          = 0;
unsigned int  readingsPerMinute = 0;

bool isValidData(AccData data) {
  return !(isnan(data.x) || isnan(data.y) || isnan(data.z) || 
           isinf(data.x) || isinf(data.y) || isinf(data.z));
}

void AccReader(void *pvParameters) {
  Serial.println("Iniciando leitura do acelerômetro...");

  sensors_event_t acc   = {0};
 
  while (1) {
    accADXL.getEvent(&acc); // lendo dados do sensor
    
    AccData newData = { acc.acceleration.x, acc.acceleration.y, acc.acceleration.z };
    
    if (!isValidData(newData)) {
      Serial.println("Dados inválidos!");
      
      vTaskDelay(pdMS_TO_TICKS(1));
      
      continue;
    }

    portENTER_CRITICAL(&mux);
      int nextIndex = (bufferHead + 1) % BUFFER_SIZE;
      
      // descarta mais antigo caso buffer cheio
      if (nextIndex == bufferTail) bufferTail = (bufferTail + 1) % BUFFER_SIZE;

      ringBuffer[bufferHead] = newData;
      bufferHead = nextIndex;
      readingsCount++;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(1)); // evitar sobrecarga do processador
  }
}
 
void AccSender(void *pvParameters) {
  Serial.println("Iniciando envio de dados do acelerômetro...");

  uint8_t bufferStr[32] = {0};
  AccData currentData   = {0};
  uint8_t hasData       = 0;
 
  while (1) {
    portENTER_CRITICAL(&mux);
      if (bufferHead != bufferTail) {  // verifica se o ring buffer não está vazio
        currentData = ringBuffer[bufferTail];
        
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
        hasData = 1;
      }
    portEXIT_CRITICAL(&mux);

    if (hasData) {
      snprintf((char *)bufferStr, sizeof(bufferStr), "%0.2f;%0.2f;%0.2f", currentData.x, currentData.y, currentData.z);
      Serial.println((char *)bufferStr);
    }
    
    hasData = 0;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool initADXL345() {
  if (!accADXL.begin()) return false;

  accADXL.setRange(ADXL345_RANGE_2_G);
  accADXL.setDataRate(ADXL345_DATARATE_1600_HZ);

  return true;
}
 
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(INIT_DELAY_MS);

  Serial.println("Iniciando acelerômetro...");

  if (!initADXL345()) {
 
    Serial.println("Falha ao iniciar sensor");
    
    while (1);
  }

  Serial.println("ADXL345 iniciado com sucesso!");

  delay(INIT_DELAY_MS);

  xTaskCreatePinnedToCore(AccReader, "AccReader", 10000, NULL, 1, NULL, 0); // Task 1 no núcleo 0
  xTaskCreatePinnedToCore(AccSender, "AccSender", 10000, NULL, 1, NULL, 1); // Task 2 no núcleo 1
}
 
void loop() {
  /* verificação de leituras por minuto */
  if (millis() - lastTime >= 60000) {
    lastTime = millis();
    readingsPerMinute = readingsCount;
    readingsCount = 0; // reseta contador

    Serial.print("Leituras por minuto: ");
    Serial.println(readingsPerMinute);
  }
}