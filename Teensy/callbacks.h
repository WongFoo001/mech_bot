#include "ICM_20948.h"
#include <Adafruit_VL6180X.h>
#include "uart_sender.h"
#include <ArduinoEigen.h>

using namespace Eigen;

ICM_20948_I2C ICM;

#define INT_PIN 2
#define LED_PIN LED_BUILTIN

#define bump_zone 0.5


#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                       // the ADR jumper is closed the value becomes 0

// Some vars to control or respond to interrupts
volatile bool isrFired = false;
volatile bool sensorSleep = false;
volatile bool canToggle = false;

void icmISR(void)
{
  isrFired = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
}

void setup_imu(){
  bool initialized = false;
  pinMode(INT_PIN, INPUT_PULLUP);                                   // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
  attachInterrupt(digitalPinToInterrupt(INT_PIN), icmISR, FALLING); // Set up a falling interrupt

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, !sensorSleep);

  Serial.begin(115200);

  Serial.println("starting");
  WIRE_PORT.begin();
  WIRE_PORT.setClock(4000000);
  ICM.begin(WIRE_PORT, AD0_VAL);
  Serial.println("Initialization of the sensor");
  if(ICM.status != ICM_20948_Stat_Ok){
      Serial.println("Trying again...");
      delay(500);
  }else{
      initialized = true;
  }
  
  Serial.println("Device connected!");

  
  // Here we are doing a SW reset to make sure the device starts in a known state
  ICM.swReset();
  if (ICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("Software Reset returned: "));
    Serial.println(ICM.statusString());
  }
  delay(250);


  // Now wake the sensor up
  ICM.sleep(sensorSleep);
  ICM.lowPower(false);


  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  ICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
  Serial.print(F("setSampleMode returned: "));
  Serial.println(ICM.statusString());

  ICM_20948_smplrt_t Smplrt;
  Smplrt.g = 54;
  ICM.setSampleRate(ICM_20948_Internal_Gyr, Smplrt);
  Serial.print(F("setSampleRate returned: "));
  Serial.println(ICM.statusString());

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t FSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  FSS.a = gpm8; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  ICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
  if (ICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(ICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t DLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  DLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  DLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  ICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), DLPcfg);
  if(ICM.status != ICM_20948_Stat_Ok){
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(ICM.statusString());
  }


  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = ICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = ICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(ICM.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(ICM.statusString(gyrDLPEnableStat));



  // Choose whether or not to start the magnetometer
  ICM.startupMagnetometer();
  if(ICM.status != ICM_20948_Stat_Ok){
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(ICM.statusString());
  }



  // Now we're going to set up interrupts. There are a lot of options, but for this test we're just configuring the interrupt pin and enabling interrupts to tell us when new data is ready
  /*
    ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
    ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
    ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse

    ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first

    ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
    ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

    ICM_20948_Status_e  intEnableI2C            ( bool enable );
    ICM_20948_Status_e  intEnableDMP            ( bool enable );
    ICM_20948_Status_e  intEnablePLL            ( bool enable );
    ICM_20948_Status_e  intEnableWOM            ( bool enable );
    ICM_20948_Status_e  intEnableWOF            ( bool enable );
    ICM_20948_Status_e  intEnableRawDataReady   ( bool enable );
    ICM_20948_Status_e  intEnableOverflowFIFO   ( uint8_t bm_enable );
    ICM_20948_Status_e  intEnableWatermarkFIFO  ( uint8_t bm_enable );
 */
  ICM.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
  ICM.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
  ICM.cfgIntLatch(true);      // Latch the interrupt until cleared
  Serial.print(F("cfgIntLatch returned: "));
  Serial.println(ICM.statusString());

  ICM.intEnableRawDataReady(true); // enable interrupts on raw data ready
  Serial.print(F("intEnableRawDataReady returned: "));
  Serial.println(ICM.statusString());

  //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
  //  uint8_t zero_0 = 0xFF;
  //  ICM_20948_execute_r( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
  //  SERIAL_PORT.print("INT_EN was: 0x"); SERIAL_PORT.println(zero_0, HEX);
  //  zero_0 = 0x00;
  //  ICM_20948_execute_w( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );

  Serial.println();
  Serial.println(F("Configuration complete!"));

}


// return true if bump and azimuth of bump
void bump_sens(ICM_20948_I2C *sensor){
  double x_accl = sensor->accX();
  double y_accl = sensor->accY();
  double yaw_vel = sensor->gyrZ();

//  Serial.print("raw x_accl: ");
//  Serial.println(x_accl);
//  
//  Serial.print("raw y_accl: ");
//  Serial.println(y_accl);
//  
//  Serial.print("raw yaw: ");
//  Serial.println(yaw_vel);

//  static Vector3f x_t1(0,0,0); // previous measurements 
//  static Matrix3f K; // kalman gain matrix
//  static Matrix3f P; // error covariance matrix
//  static Matrix3f R; // measurement noise covariance matrix
//  static Matrix3f C; // sensor model matrix -> init as identity to be easy
//  C << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
//  static Matrix3f Q; // process noise covariance matrix
//  static Matrix3f A = []{
//    Matrix3f tmp;
//    tmp << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
//    return tmp;
//  }();
//  const Matrix3f I = []{
//    Matrix3f tmp;
//    tmp << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
//    return tmp;
//  }(); // identity matrix  
  // measurement at current time
//  Vector3f z = {x_accl, y_accl, yaw_vel};

  // predict
//  Vector3f x = A * x_t1; // B * u_t1
//  P = A * P * A.transpose() + Q;
  // save for later
//  x_t1 = x;

  // update
//  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
//  Vector3f x_t = x + K * (z - C * x);
//  Vector3f p_x_t = (I - K * C) * P;

//  eigen_print(est, "estimate");
//  eigen_print(est_err, "estimate error");
//
//  if(abs(x_accl) > abs(est[0] * (1 + bump_zone)) || abs(y_accl) > abs(est[1] * (1 + bump_zone)) ){
//      Serial.print(est[0]);
//      Serial.print(est[1]);
//      Serial.println("eigen detected a bump");
//  }
//  

  static float mag;
  static unsigned long Time;
  //         ascii B, az  
  uint8_t buf[2] = {0x42, 0x00};
  // check for a force above some bounds 
  if(abs(x_accl) >= 150.0 || abs(y_accl) >= 150.0){
    // calculate bump direction

    // try to damp the bounceback
    float new_mag = sqrt(sq(sensor->accY()) + sq(sensor->accX()));
    unsigned long t = millis() - Time;
    Serial.println(mag /t);
    if(mag* 100 / (t) < new_mag){
      mag = new_mag;
      Time = millis();
      uint8_t bump_az = 255*(255/RAD_TO_DEG * atan2(-sensor->accY(), -sensor->accX()));
      if(bump_az)
      buf[1] = bump_az;
      Serial.print("BUMP! ");
      Serial.println(bump_az, DEC);
      send_er(buf);
    }
  } 
}



// also use address
Adafruit_VL6180X distF, distA;

void setup_lidar(){
  Serial.println("Got here");
  if (distF.begin() == 1){
    Serial.println("Distance sensor Initialized");
  }
  else{
    Serial.println("Distance Sensor Initialization failed");
  }
}

void dist_sens(Adafruit_VL6180X *vl, uint8_t adr){
  uint8_t buf[2] = {0x00, 0x00};
  
  //float lux = vl->readLux(VL6180X_ALS_GAIN_5);

  //Serial.print("Lux: "); Serial.println(lux);
  
  uint8_t range = vl->readRange();
  uint8_t status = vl->readRangeStatus();

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }

  if (status == VL6180X_ERROR_NONE) {
    //Serial.print("Distance: "); Serial.println(range);  
//    if(adr == 0x06){
//      // fore
//      buf[0] = 0x46; // ascii F
//    }else if(adr == 0x07){
//      //aft
//      buf[0] = 0x41; // ascii A
//    }

    buf[0] = 0x46; // ascii F
    buf[1] = range;
    
    
    send_er(buf);
  }
}
