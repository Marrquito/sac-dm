#include <stdint.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>

/*  DEFINES */
#define ACC_MPU         "MPU"
#define ACC_ADXL        "ADXL"
#define SERIAL_BAUDRATE 921600
#define INIT_DELAY_MS   500
#define MPU_DELAY_US    320
#define ADXL_DELAY_US   160

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
AccData accData   = {0};
AccType accType   = ACCEL_UNKNOWN;

Adafruit_MPU6050          accelMPU;
Adafruit_ADXL345_Unified  accelADXL = Adafruit_ADXL345_Unified(12345);

unsigned int readingsCount      = 0;
unsigned long lastTime          = 0;
unsigned int readingsPerMinute  = 0;

void AccReader(void *pvParameters) {
  sensors_event_t event = {0};
  sensors_event_t g     = {0};
  sensors_event_t temp  = {0};
 
  while (1) {
    // Leitura dos dados do acelerômetro
    if      (accType == ACCEL_MPU6050) accelMPU.getEvent(&event, &g, &temp);
    else if (accType == ACCEL_ADXL345) accelADXL.getEvent(&event);
    
    // Aquisição do mutex para garantir acesso exclusivo às variáveis compartilhadas
    portENTER_CRITICAL(&mux);
      accData.x = event.acceleration.x;
      accData.y = event.acceleration.y;
      accData.z = event.acceleration.z;
    portEXIT_CRITICAL(&mux);
  }
}
 
void AccSender(void *pvParameters) {
  uint8_t   buffer[20] = {0};
  uint16_t  delay      = (accType == ACCEL_MPU6050) ? MPU_DELAY_US : ADXL_DELAY_US;
 
  while (1) {
    // Aquisição do mutex para garantir acesso exclusivo às variáveis compartilhadas
    portENTER_CRITICAL(&mux);
    AccData currentData = accData;
    readingsCount++;
    portEXIT_CRITICAL(&mux);
 
    snprintf((char *)buffer, sizeof(buffer), "%0.2f;%0.2f;%0.2f", currentData.x, currentData.y, currentData.z);
    Serial.println((char *)buffer);

    delayMicroseconds(delay);
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
  // O loop principal é deixado vazio, já que as tasks estão sendo executadas nos núcleos separados

  /* verificação de leituras por minuto */
  if (millis() - lastTime >= 60000) {
    lastTime = millis();
    readingsPerMinute = readingsCount;
    readingsCount = 0; // reseta contador

    Serial.print("Leituras por minuto: ");
    Serial.println(readingsPerMinute);
  }
}