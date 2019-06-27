#define OUTPUT_BAUD_RATE 57600                                 //Baud Rate
#define OUTPUT_DATA_INTERVAL 5  // in milliseconds

//Untuk Kalibrasi
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)

// Magnetometer
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

// Gyroscope
#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)

// Altymeter
#define ALT_SEA_LEVEL_PRESSURE 102133


// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)


// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#include <Wire.h>
// RAW sensor data
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;
short i = 386, j = 903, k = 410, l = 122;
float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3] = {0, 0, 0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw, pitch, roll;
short int Y;
short int P;
short int R;
short int gx;
short int gy;
short int gz;
//char srRead;
// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void ReadSensors()
{
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();
}

void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  ReadSensors();

  timestamp = millis();

  //Get Pitch
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
  // Magnetometer axis mapping
  Magn_Vector[1] = -magnetom[0];
  Magn_Vector[0] = -magnetom[1];
  Magn_Vector[2] = -magnetom[2];

  // Magnetometer values mapping
  Magn_Vector[0] -= MAGN_X_OFFSET;
  Magn_Vector[0] *= MAGN_X_SCALE;
  Magn_Vector[1] -= MAGN_Y_OFFSET;
  Magn_Vector[1] *= MAGN_Y_SCALE;
  Magn_Vector[2] -= MAGN_Z_OFFSET;
  Magn_Vector[2] *= MAGN_Z_SCALE;

  // Accelerometer axis mapping
  Accel_Vector[1] = accel[0];
  Accel_Vector[0] = accel[1];
  Accel_Vector[2] = accel[2];

  // Accelerometer values mapping
  Accel_Vector[0] -= ACCEL_X_OFFSET;
  Accel_Vector[0] *= ACCEL_X_SCALE;
  Accel_Vector[1] -= ACCEL_Y_OFFSET;
  Accel_Vector[1] *= ACCEL_Y_SCALE;
  Accel_Vector[2] -= ACCEL_Z_OFFSET;
  Accel_Vector[2] *= ACCEL_Z_SCALE;

  // Gyroscope axis mapping
  Gyro_Vector[1] = -gyro[0];
  Gyro_Vector[0] = -gyro[1];
  Gyro_Vector[2] = -gyro[2];

  // Gyroscope values mapping
  Gyro_Vector[0] -= GYRO_X_OFFSET;
  Gyro_Vector[0] *= GYRO_X_SCALE;
  Gyro_Vector[1] -= GYRO_Y_OFFSET;
  Gyro_Vector[1] *= GYRO_Y_SCALE;
  Gyro_Vector[2] -= GYRO_Z_OFFSET;
  Gyro_Vector[2] *= GYRO_Z_SCALE;
}


void setup()
{
  // Init serial output
  Serial.begin(OUTPUT_BAUD_RATE);

  // Initlize sensors
  //delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();
  reset_sensor_fusion();
}

void loop()
{

  if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f;
    else
      G_Dt = 0;
    //srRead = Serial.read();
    ReadSensors();
    Compass_Heading(); // Calculate magnetic heading

    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

    output_full_sensors();

  }
}
