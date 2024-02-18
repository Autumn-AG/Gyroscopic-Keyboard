#include <Wire.h>
#include "I2Cdev.h"  // IC2 Lib
#include "MPU9250.h" // AccelGyro Lib
#include "rgb_lcd.h" // RGB LCD 

I2Cdev I2C_M;
MPU9250 accelgyro;
rgb_lcd lcd;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

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

const int Touch = 2;
const int colorR = 200;
const int colorG = 200;
const int colorB = 200;
int memory = 0;
long int timepast, timenow = 0;

char current_string[20];
int current_length = 0;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire1.begin();

    // initialize serial communication
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    pinMode(Touch, INPUT);
    lcd.begin(16, 2);  
    lcd.setRGB(colorR, colorG, colorB);
    lcd.print("HGK Initialized.");
    delay(1000);
    lcd.clear();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
    delay(800);
    Serial.println("     ");
}

void loop()
{
    getAccel_Data();
    getGyro_Data();
    getCompassData_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    // Cycling
    

    int TouchVal = digitalRead(Touch);

    if (TouchVal == 1) {
      //Serial.println("If check");
      if (memory == 0) {
        timepast = millis();
      }
      memory = 1;
      //Serial.println(" --------------> mem update 1");
    } else if (TouchVal == 0) {
      //Serial.println("else check");
      if (memory == 1) {
        timenow = millis();
        //Serial.println("else -> if check");
        if (timenow - timepast < 800) {
          key_enter(current_string, current_length, getLetter(tiltheading));
          current_length++;
          Serial.println(current_length);
          Serial.println("CL+");
        } else {
          if (current_length != 0) {
            key_delete(current_string, current_length);
            current_length--;
            Serial.println(current_length);
            Serial.println("CL-");
          }
        }
      }
      memory = 0;
      //Serial.println(" --------------> mem update 0");
    }

    lcd.clear();
    lcd.print(current_string);
    lcd.print(getLetter(tiltheading));
  
    // Serial.print("Touch Sensor reading: ");
    // Serial.println(TouchVal);

    // Serial.println("Current letter: ");
    Serial.print(current_string);
    Serial.print("    ");
    Serial.println(getLetter(tiltheading));
    delay(300);
    
}

char key_enter(char* current_string, int current_length, char letter) {
  current_string[current_length] = letter;
}

char key_delete(char* current_string, int current_length) {
  current_string[current_length - 1] = '\0';
}

char getLetter(float tilt) {
  float end1, end2, end3, end4, end5, end6, end7, end8, end9, end10, end11, end12, end13, end14, end15, end16, end17, end18, end19, end20, end21, end22, end23, end24, end25, end26, end27, end28;

  end1 = 0.0;
  end2 = 13.0;
  end3 = 26.0;
  end4 = 39.0;
  end5 = 52.0;
  end6 = 65.0;
  end7 = 78.0;
  end8 = 91.0;
  end9 = 104.0;
  end10 = 117.0;
  end11 = 130.0;
  end12 = 143.0;
  end13 = 156.0;
  end14 = 169.0;
  end15 = 182.0;
  end16 = 195.0;
  end17 = 208.0;
  end18 = 221.0;
  end19 = 234.0;
  end20 = 247.0;
  end21 = 260.0;
  end22 = 273.0;
  end23 = 286.0;
  end24 = 299.0;
  end25 = 312.0;
  end26 = 325.0;
  end27 = 338.0;
  end28 = 351.0;


  if (tilt < end1) {
    return 'a';
  }
  else if (tilt >= end1 && tilt < end2) {
    return 'b';
  }
  else if (tilt >= end2 && tilt < end3) {
    return 'c';
  }
  else if (tilt >= end3 && tilt < end4) {
    return 'd';
  }
  else if (tilt >= end4 && tilt < end5) {
    return 'e';
  }
  else if (tilt >= end5 && tilt < end6) {
    return 'f';
  }
  else if (tilt >= end6 && tilt < end7) {
    return 'g';
  }
  else if (tilt >= end7 && tilt < end8) {
    return 'h';
  }
  else if (tilt >= end8 && tilt < end9) {
    return 'i';
  }
  else if (tilt >= end9 && tilt < end10) {
    return 'j';
  }
  else if (tilt >= end10 && tilt < end11) {
    return 'k';
  }
  else if (tilt >= end11 && tilt < end12) {
    return 'l';
  }
  else if (tilt >= end12 && tilt < end13) {
    return 'm';
  }
  else if (tilt >= end13 && tilt < end14) {
    return 'n';
  }
  else if (tilt >= end14 && tilt < end15) {
    return 'o';
  }
  else if (tilt >= end15 && tilt < end16) {
    return 'p';
  }
  else if (tilt >= end16 && tilt < end17) {
    return 'q';
  }
  else if (tilt >= end17 && tilt < end18) {
    return 'r';
  }
  else if (tilt >= end18 && tilt < end19) {
    return 's';
  }
  else if (tilt >= end19 && tilt < end20) {
    return 't';
  }
  else if (tilt >= end20 && tilt < end21) {
    return 'u';
  }
  else if (tilt >= end21 && tilt < end22) {
    return 'v';
  }
  else if (tilt >= end22 && tilt < end23) {
    return 'w';
  }
  else if (tilt >= end23 && tilt < end24) {
    return 'x';
  }
  else if (tilt >= end24 && tilt < end25) {
    return 'y';
  }
  else if (tilt >= end25 && tilt < end26) {
    return 'z';
  }
  else if (tilt >= end26 && tilt < end27) {
    return ' ';
  }
  else if (tilt >= end27 && tilt < end28) {
    return '.';
  }
  else {
    return ' ';
  }





}
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
    if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}


void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();

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

void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassData_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}