#include "wallaby_imu.h"

#include <math.h>

#include "wallaby.h"

#include "MPU9250.h"

#define sample_num_mdate  5000
#define PI 3.14159265359

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;


float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

#define WALLABY2

void setupIMU()
{
    initializeMpu();

    bool isConnected = testConnectionMpu();
    if (!isConnected)
    {
        debug_printf("MPU9250 connection failed\n");
        return;
    }

    delay_us(1000 * 1000);
    //  Mxyz_init_calibrated ();
}

uint32_t last_read_time = 0;
uint32_t last_mag_read_time = 0;

// Write to register:
// magn_x = (((uint16_t)buff[1]) << 8) | buff[0];
// magn_y = (((uint16_t)buff[3]) << 8) | buff[2];
// magn_z = (((uint16_t)buff[5]) << 8) | buff[4];
//
// // TODO: already have these in buff
// aTxBuffer[REG_RW_MAG_X_H] = (magn_x & 0xFF00) >> 8;
// aTxBuffer[REG_RW_MAG_X_L] = (magn_x & 0x00FF);
// aTxBuffer[REG_RW_MAG_Y_H] = (magn_y & 0xFF00) >> 8;
// aTxBuffer[REG_RW_MAG_Y_L] = (magn_y & 0x00FF);
// aTxBuffer[REG_RW_MAG_Z_H] = 0;
// aTxBuffer[REG_RW_MAG_Z_L] = v;
// //aTxBuffer[REG_RW_MAG_Z_L] = whoAmI;
// // aTxBuffer[REG_RW_MAG_Z_H] = (magn_z & 0xFF00) >> 8;
// // aTxBuffer[REG_RW_MAG_Z_L] = (magn_z & 0x00FF);

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0) tiltheading += 360;
}

void getCompass_Data(void)
{
    // ToDo: read magneto actually
    // I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    // delay(10);
    // I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
    //
    // mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0];
    // my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2];
    // mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4];
    //
    // Mxyz[0] = (double)mx * 1200 / 4096;
    // Mxyz[1] = (double)my * 1200 / 4096;
    // Mxyz[2] = (double)mz * 1200 / 4096;
}

void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void get_calibration_Data()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();
        /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */


        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];


    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}


void Mxyz_init_calibrated()
{
    debug_printf("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes.\n");
    debug_printf("Please put the 9DOF in a place where there is no magnetic field interference.\n");
    debug_printf("You will now have 2 minutes \n");

    debug_printf("Sampling starting...");
    get_calibration_Data();
    debug_printf("Compass calibration: { %f, %f, %f }\n", mx_centre, my_centre, mz_centre);
}

void getAccel_Data(void)
{
    getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double)ax / 16384;
    Axyz[1] = (double)ay / 16384;
    Axyz[2] = (double)az / 16384;
}

void getGyro_Data(void)
{
    getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double)gx * 250 / 32768;
    Gxyz[1] = (double)gy * 250 / 32768;
    Gxyz[2] = (double)gz * 250 / 32768;
}

void getCompassDate_calibrated()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}

void readIMU()
{
    // Read at 200Hz
    if (usCount - last_read_time < 5000) return;
    last_read_time = usCount;

    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();
    //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    // Serial.println("calibration parameter: ");
    // Serial.print(mx_centre);
    // Serial.print("         ");
    // Serial.print(my_centre);
    // Serial.print("         ");
    // Serial.println(mz_centre);
    // Serial.println("     ");
    //
    //
    // Serial.println("Acceleration(g) of X,Y,Z:");
    // Serial.print(Axyz[0]);
    // Serial.print(",");
    // Serial.print(Axyz[1]);
    // Serial.print(",");
    // Serial.println(Axyz[2]);
    // Serial.println("Gyro(degress/s) of X,Y,Z:");
    // Serial.print(Gxyz[0]);
    // Serial.print(",");
    // Serial.print(Gxyz[1]);
    // Serial.print(",");
    // Serial.println(Gxyz[2]);
    // Serial.println("Compass Value of X,Y,Z:");
    // Serial.print(Mxyz[0]);
    // Serial.print(",");
    // Serial.print(Mxyz[1]);
    // Serial.print(",");
    // Serial.println(Mxyz[2]);
    // Serial.println("The clockwise angle between the magnetic north and X-Axis:");
    // Serial.print(heading);
    // Serial.println(" ");
    // Serial.println(
    //     "The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
    // Serial.println(tiltheading);
    // Serial.println("   ");

    delay_us(1000 * 1000);
}
