
// ヘッダファイルの読み込み
#include "mrd_wire0.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用

//==================================================================================================
//  I2C wire0 関連の処理
//==================================================================================================
// 6軸or9軸センサーの値
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス

  MPU6050 mpu6050;        // MPU6050のインスタンス
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  float yaw_origin = 0;   // ヨー軸の補正センター値
  float yaw_source = 0;   // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;                      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                     // [x, y, z]            磁力センサの測定値
  long temperature;                    // センサの温度測定値
};

// 6軸or9軸センサーの値
AhrsValue m_ahrs;

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA, int a_pinSCL) {
  Serial.print("Initializing wire0 I2C... ");
  if (a_pinSDA == -1 && a_pinSCL == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pinSDA, a_pinSCL);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

/// @brief MPU6050センサーのDMP(デジタルモーションプロセッサ)を初期化し,
///        ジャイロスコープと加速度センサーのオフセットを設定する.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu6050_dmp() {
  m_ahrs.mpu6050.initialize();
  m_ahrs.devStatus = m_ahrs.mpu6050.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  m_ahrs.mpu6050.setXAccelOffset(-1745);
  m_ahrs.mpu6050.setYAccelOffset(-1034);
  m_ahrs.mpu6050.setZAccelOffset(966);
  m_ahrs.mpu6050.setXGyroOffset(176);
  m_ahrs.mpu6050.setYGyroOffset(-6);
  m_ahrs.mpu6050.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (m_ahrs.devStatus == 0) {
    m_ahrs.mpu6050.CalibrateAccel(6);
    m_ahrs.mpu6050.CalibrateGyro(6);
    m_ahrs.mpu6050.setDMPEnabled(true);
    m_ahrs.packetSize = m_ahrs.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

/// @brief BNO055センサーの初期化を試みます.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
///         現在, この関数は常にfalseを返すように設定されています.
bool mrd_wire0_init_bno055() {
  if (!m_ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  } else {
    Serial.println("BNO055 mounted.");
    delay(50);
    m_ahrs.bno.setExtCrystalUse(false);
    delay(10);
    return true;
  }
  // データの取得はセンサー用スレッドで実行
}

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です(MPU6050, MPU9250, BNO055).
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号.下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号.上記と合わせて省略可.
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, int a_pinSDA, int a_pinSCL) {
  if (a_imuahrs_type > 0) { // 何らかのセンサを搭載
    if (a_pinSDA == -1 && a_pinSCL == -1) {
      mrd_wire0_init_i2c(a_i2c0_speed);
    } else {
      mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
    }
  }

  if (a_imuahrs_type == MPU6050_IMU) { // MPU6050
    return mrd_wire0_init_mpu6050_dmp();
  } else if (a_imuahrs_type == MPU9250_IMU) { // MPU9250の場合
    // mrd_wire_init_mpu9250_dmp(m_ahrs)
    return false;
  } else if (a_imuahrs_type == BNO055_AHRS) { // BNO055の場合
    return mrd_wire0_init_bno055();
  }

  Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args) {
  while (1) {
    // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
    imu::Vector<3> accelerometer = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    m_ahrs.read[0] = (float)accelerometer.x();
    m_ahrs.read[1] = (float)accelerometer.y();
    m_ahrs.read[2] = (float)accelerometer.z();

    // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
    imu::Vector<3> gyroscope = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    m_ahrs.read[3] = gyroscope.x();
    m_ahrs.read[4] = gyroscope.y();
    m_ahrs.read[5] = gyroscope.z();

    // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
    imu::Vector<3> magnetometer = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    m_ahrs.read[6] = magnetometer.x();
    m_ahrs.read[7] = magnetometer.y();
    m_ahrs.read[8] = magnetometer.z();

    // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
    imu::Vector<3> euler = m_ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    m_ahrs.read[12] = euler.y();                   // DMP_ROLL推定値
    m_ahrs.read[13] = euler.z();                   // DMP_PITCH推定値
    m_ahrs.yaw_source = euler.x();                 // ヨー軸のソースデータ保持
    float yaw_tmp = euler.x() - m_ahrs.yaw_origin; // DMP_YAW推定値
    if (yaw_tmp >= 180) {
      yaw_tmp = yaw_tmp - 360;
    } else if (yaw_tmp < -180) {
      yaw_tmp = yaw_tmp + 360;
    }
    m_ahrs.read[14] = yaw_tmp; // DMP_YAW推定値
    m_ahrs.ypr[0] = m_ahrs.read[14];
    m_ahrs.ypr[1] = m_ahrs.read[13];
    m_ahrs.ypr[2] = m_ahrs.read[12];

    // センサフュージョンの方向推定値のクオータニオン
    // imu::Quaternion quat = bno.getQuat();

    // Serial.print("qW: ");
    // Serial.print(quat.w(), 4);
    // Serial.print(" qX: ");
    // Serial.print(quat.x(), 4);
    // Serial.print(" qY: ");
    // Serial.print(quat.y(), 4);
    // Serial.print(" qZ: ");
    // Serial.println(quat.z(), 4);

    // キャリブレーションのステータスの取得と表示
    // uint8_t system, gyro, accel, mag = 0;
    // bno.getCalibration(&system, &gyro, &accel, &mag);
    // Serial.print("CALIB Sys:");
    // Serial.print(system, DEC);
    // Serial.print(", Gy");
    // Serial.print(gyro, DEC);
    // Serial.print(", Ac");
    // Serial.print(accel, DEC);
    // Serial.print(", Mg");
    // Serial.println(mag, DEC);

    delay(IMUAHRS_INTERVAL);
  }
}

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
/// MPU6050, MPU9250を想定していますが, MPU9250は未実装.
/// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
bool mrd_wire0_read_ahrs_i2c(MrdFlags &a_flg) { // ※wireTimer0.beginの引数のためvoid必須

  if (MOUNT_IMUAHRS == MPU6050_IMU) {                                // MPU6050
    if (m_ahrs.mpu6050.dmpGetCurrentFIFOPacket(m_ahrs.fifoBuffer)) { // Get new data
      m_ahrs.mpu6050.dmpGetQuaternion(&m_ahrs.q, m_ahrs.fifoBuffer);
      m_ahrs.mpu6050.dmpGetGravity(&m_ahrs.gravity, &m_ahrs.q);
      m_ahrs.mpu6050.dmpGetYawPitchRoll(m_ahrs.ypr, &m_ahrs.q, &m_ahrs.gravity);

      // acceleration values
      m_ahrs.mpu6050.dmpGetAccel(&m_ahrs.aa, m_ahrs.fifoBuffer);
      m_ahrs.read[0] = (float)m_ahrs.aa.x;
      m_ahrs.read[1] = (float)m_ahrs.aa.y;
      m_ahrs.read[2] = (float)m_ahrs.aa.z;

      // gyro values
      m_ahrs.mpu6050.dmpGetGyro(&m_ahrs.gyro, m_ahrs.fifoBuffer);
      m_ahrs.read[3] = (float)m_ahrs.gyro.x;
      m_ahrs.read[4] = (float)m_ahrs.gyro.y;
      m_ahrs.read[5] = (float)m_ahrs.gyro.z;

      // magnetic field values
      m_ahrs.read[6] = (float)m_ahrs.mag.x;
      m_ahrs.read[7] = (float)m_ahrs.mag.y;
      m_ahrs.read[8] = (float)m_ahrs.mag.z;

      // Estimated gravity DMP value.
      m_ahrs.read[9] = m_ahrs.gravity.x;
      m_ahrs.read[10] = m_ahrs.gravity.y;
      m_ahrs.read[11] = m_ahrs.gravity.z;

      // Estimated heading value using DMP.
      m_ahrs.read[12] = m_ahrs.ypr[2] * 180 / M_PI;                       // Estimated DMP_ROLL
      m_ahrs.read[13] = m_ahrs.ypr[1] * 180 / M_PI;                       // Estimated DMP_PITCH
      m_ahrs.read[14] = (m_ahrs.ypr[0] * 180 / M_PI) - m_ahrs.yaw_origin; // Estimated DMP_YAW

      // Temperature
      m_ahrs.read[15] = 0; // Not implemented.

      if (a_flg.imuahrs_available) {
        memcpy(m_ahrs.result, m_ahrs.read, sizeof(float) * 16);
      }
      return true;
    } else {
      return false;
    }
  } else if (MOUNT_IMUAHRS == MPU9250_IMU) { // MPU9250
    return false;
  } else {
    return false;
  }
}

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む.
/// @param a_type 使用するセンサのタイプを示す列挙(MPU6050, MPU9250, BNO055).
/// @param m_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool meriput90_ahrs(Meridim90Union &a_meridim, int a_type, MERIDIANFLOW::Meridian &mrd, MrdFlags &a_flg) {
  if (a_type == BNO055_AHRS) {
    a_flg.imuahrs_available = false;
    a_meridim.sval[2] = mrd.float2HfShort(m_ahrs.read[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = mrd.float2HfShort(m_ahrs.read[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = mrd.float2HfShort(m_ahrs.read[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = mrd.float2HfShort(m_ahrs.read[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = mrd.float2HfShort(m_ahrs.read[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = mrd.float2HfShort(m_ahrs.read[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = mrd.float2HfShort(m_ahrs.read[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = mrd.float2HfShort(m_ahrs.read[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = mrd.float2HfShort(m_ahrs.read[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = mrd.float2HfShort(m_ahrs.read[15]); // temperature
    a_meridim.sval[12] = mrd.float2HfShort(m_ahrs.read[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = mrd.float2HfShort(m_ahrs.read[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = mrd.float2HfShort(m_ahrs.read[14]); // DMP_YAW推定値
    a_flg.imuahrs_available = true;
    return true;
  }
  return false;
}

void mrd_wire0_calibrate_yaw_origin() {
  m_ahrs.yaw_origin = m_ahrs.yaw_source;
}
