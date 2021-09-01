#include <Wire.h>
#define MPU_ADDR 0x68
#define GYRO_Z_ADDR 0x43
#define ACCEL_Z_ADDR 0x3F

int16_t offset;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  writeByte(MPU_ADDR,0x1B , 0x18); // Set to 2000 DPS
  writeByte(MPU_ADDR, 0x1C, 0x00); // Set to 2g
  writeByte(MPU_ADDR, 0x1D, 0x06); // 5Hz low pass filter
  pinMode(6, OUTPUT); // Green light
  pinMode(5, OUTPUT); // Yellow light
  pinMode(3, OUTPUT); // Red light
  offset = getOffset(GYRO_Z_ADDR);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_ADDR);
  int16_t gyroZ = readSensorData(GYRO_Z_ADDR) - offset;
  int16_t accelZ = readSensorData(ACCEL_Z_ADDR);
  bool safeToFire = false;

//  Serial.write("Gyro Z rate of change: ");
//  Serial.print(z);
//  Serial.write("\n");

  // Yellow if moving too much
  if(abs(gyroZ) > 200) {
    digitalWrite(5, HIGH);
    safeToFire = false;
  } else {
    digitalWrite(5, LOW);
    safeToFire = true;
  }

  if(safeToFire) {
  // Green if not moving too much
    if(accelZ < 9191) {
      digitalWrite(6, HIGH);
    } else {
      digitalWrite(6, LOW);
      safeToFire = false;
    }
  } else {
      digitalWrite(6, LOW);
      safeToFire = false;
   }

  // Red if it's not safe to fire
  if(safeToFire) {
    digitalWrite(3, LOW);
  } else {
    digitalWrite(3, HIGH);
  }
  
}

int16_t getOffset(int16_t reg) {
  int16_t accumulator = 0;
  for(int i=0; i<15; i++) {
    accumulator += readSensorData(reg);
  }
  return -(accumulator/15);
}

int16_t readSensorData(uint8_t addr) {
  int16_t ret;
  uint8_t rawData[2]; // This is bad. Allocate outside of loops
  getBytes(rawData, MPU_ADDR, addr, 2);
  // Cast to 16 bit signed int
  // Bit shift left 8
  // bitwise or
  ret = ((int16_t)rawData[0] << 8) | rawData[1];
  return ret;
}

// Copy from here
// https://github.com/kriswiner/MPU9250/blob/72038b040bef3cf072612cd8a8ee8f26e3c87158/MPU9250BasicAHRS.ino#L1024
void getBytes(uint8_t* tmp, uint8_t addr, uint8_t subAddr, uint8_t bytes) {
  Wire.beginTransmission(addr);
  Wire.write(subAddr);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(addr, bytes);
  while(Wire.available()) {
    tmp[i++] = Wire.read();
  }
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
