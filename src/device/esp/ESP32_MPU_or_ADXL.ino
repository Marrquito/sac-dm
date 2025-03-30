#include <stdint.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>

/*  DEFINES */
#define SERIAL_BAUDRATE 921600
#define BUFFER_SIZE     1000
#define INIT_DELAY_MS   500

/* TYPES */
typedef enum {
  ACCEL_UNKNOWN,
  ACCEL_MPU6050,
  ACCEL_ADXL345
} AccType;

typedef struct {
  float x;
  float y;
  float z;
} AccData;

/* GLOBAL VARS */
portMUX_TYPE mux  = portMUX_INITIALIZER_UNLOCKED;

Adafruit_MPU6050          accelMPU;
Adafruit_ADXL345_Unified  accelADXL = Adafruit_ADXL345_Unified(12345);

AccData ringBuffer[BUFFER_SIZE] = {0};
AccType accType                 = ACCEL_UNKNOWN;

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
  sensors_event_t acc   = {0};
  sensors_event_t gyro  = {0};
  sensors_event_t temp  = {0};
 
  while (1) {
    // lendo dados do sensor
    if      (accType == ACCEL_MPU6050) accelMPU.getEvent(&acc, &gyro, &temp);
    else if (accType == ACCEL_ADXL345) accelADXL.getEvent(&acc);
    
    AccData newData = { acc.acceleration.x, acc.acceleration.y, acc.acceleration.z };
    
    if (!isValidData(newData)) {
      Serial.println("Dados inválidos!");
      
      vTaskDelay(pdMS_TO_TICKS(5));
      
      continue;
    }

    portENTER_CRITICAL(&mux);
      int nextIndex = (bufferHead + 1) % BUFFER_SIZE;
      
      if (nextIndex == bufferTail) {  // verifica se o ring buffer está cheio
        bufferTail = (bufferTail + 1) % BUFFER_SIZE; // descarta mais antigo
      } 

      ringBuffer[bufferHead] = newData;
      bufferHead = nextIndex;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(pdMS_TO_TICKS(5)); // evitar sobrecarga do processador
  }
}
 
void AccSender(void *pvParameters) {
  uint8_t bufferStr[32] = {0};
  AccData currentData   = {0};
 
  while (1) {
    uint8_t hasData = 0;

    portENTER_CRITICAL(&mux);
      if (bufferHead != bufferTail) {  // verifica se o ring buffer não está vazio
        currentData = ringBuffer[bufferTail];
        
        bufferTail = (bufferTail + 1) % BUFFER_SIZE;
        hasData = 1;
      }
      readingsCount++;
    portEXIT_CRITICAL(&mux);

    if (hasData) {
      snprintf((char *)bufferStr, sizeof(bufferStr), "%0.2f;%0.2f;%0.2f", currentData.x, currentData.y, currentData.z);
      Serial.println((char *)bufferStr);
    }
 
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

bool initMPU6050() {
  if (!accelMPU.begin()) return false;

  accelMPU.setAccelerometerRange(MPU6050_RANGE_2_G);
  accelMPU.setGyroRange(MPU6050_RANGE_500_DEG);
  accelMPU.setFilterBandwidth(MPU6050_BAND_5_HZ);

  return true;
}

bool initADXL345() {
  if (!accelADXL.begin()) return false;

  accelADXL.setRange(ADXL345_RANGE_2_G);
  accelADXL.setDataRate(ADXL345_DATARATE_1600_HZ);

  return true;
}
 
 
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(INIT_DELAY_MS);

  // Inicialização do acelerômetro
  Serial.println("Iniciando acelerômetro...");

  if (initMPU6050()) {
    accType = ACCEL_MPU6050;
    Serial.println("MPU6050 iniciado com sucesso!");
  } else if (initADXL345()) {
    accType = ACCEL_ADXL345;
    Serial.println("ADXL345 iniciado com sucesso!");
  } else {
    Serial.println("Falha ao iniciar os dois!");
    
    while (1) {
      Serial.println("Falha ao iniciar os dois!");
      delay(1000);
    }
  }

  delay(INIT_DELAY_MS);
 
  // Criação das tasks
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