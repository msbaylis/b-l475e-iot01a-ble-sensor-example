//
// STMicro STM32L4 Discovery Kit IoT Node BLE HTS221 temperature/humidity sensor example
// This sketch serves as a demo for use on the ST Microelectronics B-L475E-IOT01A board.
// Corresponding libraries are required :
//   - Low energy Bluetooth (SPBTLE_RF) : https://github.com/stm32duino/SPBTLE-RF
//   - Temperature and humidity sensor (HTS221) : https://github.com/stm32duino/HTS221
// 
// This sketch launches 2 BLE services : Environnemental and Time.
// 
// Temperature and humidity readings from the HTS221 sensor are sent to the environment service
// and updated every second.
//
// On-board green LED blinks when in BLE Advertising mode, and remains on when BLE connection
// is established.
//
// UART output (9600 baud) displays BLE connection state and HTS221 temperature/humidity values
// when BLE connection is established.
//

#include <SPI.h>
#include <HTS221Sensor.h>
#include <SPBTLE_RF.h>
#include <sensor_service.h>

SPIClass SPI_3(PC12, PC11, PC10); // PC12 = MOSI, PC11 = MISO, PC10 = SCLK 
SPBTLERFClass BTLE(&SPI_3, PD13, PE6, PA8, LED4); // PD13 = csPin, PE6 = spiIRQ, PA8 = reset, LED4 = led

const char *name = "B-L475E"; // BLE device name (limit 7 characters)
uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03}; // BLE device address

HTS221Sensor  *HumTemp;
TwoWire *dev_i2c;

uint32_t previousSecond = 0;  // used to trigger environmental data update
bool previousState = FALSE;

void update_temp_humidity_data(){  // retrieve data from HTS221 and xmit via BLE
  float humidity, temperature;

  HumTemp->GetHumidity(&humidity);
  HumTemp->GetTemperature(&temperature);
  Serial.print("temperature: ");
  Serial.print(temperature);
  Serial.print("° C, ");
  Serial.print("humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  SensorService.Temp_Update(temperature*10);
  SensorService.Humidity_Update(humidity*10);
}

void toggle_LED() {
  digitalWrite(LED_BUILTIN, HIGH);  // LED on
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);  // LED off
  delay(500);
}

void setup() {
  int ret;

  Serial.begin(9600);

  Serial.println("");
  Serial.println("------------------------------------------------");
  Serial.println("BLE Sensor Demo for STMicro B-L475E-IOT01A board");
  Serial.println("------------------------------------------------");
  Serial.println("");

  // Initialize LED_BUILTIN pin as output
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Initialize I2C bus.
  dev_i2c = new TwoWire(PB11, PB10);  // PB11 = I2C2 SDA, PB10 = I2C2 SCLK
  dev_i2c->begin();

  // Initialize components.
  HumTemp = new HTS221Sensor (dev_i2c);
  HumTemp->Enable();

  // Initialize BTLE part
  if(BTLE.begin() == SPBTLERF_ERROR){
    Serial.println("Error configuring bluetooth module");
    while(1);
  }

  if(SensorService.begin(name, SERVER_BDADDR)){
    Serial.println("Error configuring sensor service");
    while(1);
  }

  ret = SensorService.Add_Environmental_Sensor_Service();
  Serial.print("Adding Environmental Sensor service... ");
  if(ret == BLE_STATUS_SUCCESS)
    Serial.println("[OK]");
  else
    Serial.println("[Error]");

  ret = SensorService.Add_Time_Service();
  Serial.print("Adding Time service....................");
  if(ret == BLE_STATUS_SUCCESS)
    Serial.println("[OK]");
  else
    Serial.println("[Error]");
}

void loop() {
  BTLE.update();
  if(SensorService.isConnected() == TRUE) {
    if (previousState == TRUE){
      Serial.println("BLE connection established");
      digitalWrite(LED_BUILTIN, HIGH);  // LED on
      previousState = !previousState;
    }
    SensorService.Update_Time_Characteristics();  // update time
    if((millis() - previousSecond) >= 1000) {
      update_temp_humidity_data(); // update temperature and humidity
      previousSecond = millis();
    }
  }
  else {
    if (previousState == FALSE){
      Serial.println("");
      Serial.println("Awaiting connection request...");
      previousState = !previousState;
    }
    SensorService.setConnectable();  // stay in advertising mode
    toggle_LED();
  }
}
