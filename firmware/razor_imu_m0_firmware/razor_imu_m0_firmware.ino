#define USE_RAZOR_USB
#include <ros.h>
#include <razor_imu_m0/RazorIMURaw.h>

#include <SparkFunMPU9250-DMP.h>
#include "config.h"
MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

ros::NodeHandle_<ArduinoHardware, 2, 2, 1024, 1024> nh;
razor_imu_m0::RazorIMURaw imu_msg;
ros::Publisher imu_pub("razor_imu_raw", &imu_msg);

uint32_t lastBlink = 0;

void setup()
{
  // Initialize LED, interrupt input, and serial port.
  // LED defaults to off:
  initHardware();

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() )
  {
    while (1) ; // Loop forever if we fail to connect
    // LED will remain off in this state.
  }
  imu_msg.header.frame_id = "imu_link";
  nh.getHardware()->setBaud(2000000);
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop()
{
  // Then check IMU for new data, and log it
  if ( !imu.fifoAvailable() ) { // If no new data is available
    nh.spinOnce();
    return;                   // return to the top of the loop
  }

  // Read from the digital motion processor's FIFO
  if ( imu.dmpUpdateFifo() != INV_SUCCESS ) {
    nh.spinOnce();
    return; // If that fails (uh, oh), return to top
  }

  imu_msg.header.stamp = nh.now();
  imu_msg.quat[0] = imu.calcQuat(imu.qx);
  imu_msg.quat[1] = imu.calcQuat(imu.qy);
  imu_msg.quat[2] = imu.calcQuat(imu.qz);
  imu_msg.quat[3] = imu.calcQuat(imu.qw);

  imu_msg.acc[0] = imu.calcAccel(imu.ax) * GTOMS2;
  imu_msg.acc[1] = imu.calcAccel(imu.ay) * GTOMS2;
  imu_msg.acc[2] = imu.calcAccel(imu.az) * GTOMS2;

  imu_msg.gyr[0] = imu.calcGyro(imu.gx) * DEG2RAD;
  imu_msg.gyr[1] = imu.calcGyro(imu.gy) * DEG2RAD;
  imu_msg.gyr[2] = imu.calcGyro(imu.gz) * DEG2RAD;

  imu_pub.publish( &imu_msg );

  if ( millis() > lastBlink + UART_BLINK_RATE )
  {
    blinkLED();
    lastBlink = millis();
  }
}



///////////////////////
// LED Blink Control //
///////////////////////
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  // Configure sensors:
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(IMU_GYRO_FSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g
  imu.setAccelFSR(IMU_ACCEL_FSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(IMU_AG_LPF);
  // Set gyro/accel sample rate: must be between 4-1000Hz
  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE);
  // Set compass sample rate: between 4-100Hz
  imu.setCompassSampleRate(4);

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  if (ENABLE_GYRO_CALIBRATION)
  {
    // Gyro calibration re-calibrates the gyro after a set amount
    // of no motion detected
    dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  }
  else
  {
    // Otherwise add raw gyro readings to the DMP
    dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  }
  // Add accel and quaternion's to the DMP
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, DMP_SAMPLE_RATE);

  return true; // Return success
}


