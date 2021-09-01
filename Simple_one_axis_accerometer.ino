#include <Wire.h>
#define MPU_ADDR 0x68
//#define ACCEL_Z_ADDR 0x06
#define ACCEL_Z_ADDR 0x3F

// SDA pin should be 20
// SCL pin should be 21
int16_t previousAccelZ = 0;
int previousTime;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  writeByte(MPU_ADDR,0x1C , 0x00); // Set to 2g
  writeByte(MPU_ADDR, 0x1D, 0x06); // 5Hz low pass filter
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  previousTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_ADDR);
  int16_t z = readAccelData(ACCEL_Z_ADDR);
//  Serial.write("Z accelerometer reading: ");
//  Serial.print(z);
//  Serial.write("\n");
  bool safeToFire = false;

  // Yellow if moving too much
  float accelZDelta = z - previousAccelZ;
  int currTime = millis();
  accelZDelta /= (float)currTime;

  Serial.print(accelZDelta);
  Serial.write("\n");
  
  if(abs(accelZDelta) > 0.01) {
    digitalWrite(5, HIGH);
    safeToFire = false;
  } else {
    digitalWrite(5, LOW);
    safeToFire = true;
  }

  if(safeToFire) {
  // Green if think over 45 degrees
    if(z < 9191) {
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
  
  previousAccelZ = z;
  previousTime = currTime;
  delay(10);
}

int16_t readAccelData(uint8_t addr) {
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
//    Serial.write((int)Wire.read());
//    Serial.write("\n");
    tmp[i++] = Wire.read();
  }
//  Serial.write("\n");
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}
