#include <WiFiClient.h>
#include <ESP8266WiFiMulti.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


ESP8266WiFiMulti WiFiMulti;

#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);



/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/


void setup() {

  Serial.begin(115200);
  Wire.begin(D1, D2);
  delay(10);
  
  //WiFiMulti.addAP("Jahangir's iPhone", "123454321");
  WiFiMulti.addAP("khoowifi", "khooumiot");

  Serial.println();
  Serial.println();


    /* Initialise the sensors */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);


  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Display some basic information on this sensor */
  displaySensorDetails();
  Serial.println("-------------------------------");
  bno.setExtCrystalUse(true);

  Serial.print("Wait for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);


  //const char * host = "192.168.0.133"; // ip or dns
  //const char * host = "172.20.10.13";
  const char * host = "192.168.0.168";
  const uint16_t port = 5555;

  

  // Use WiFiClient class to create TCP connections

  WiFiClient abc;
  abc.setNoDelay(1);

  int imuID = 1; // Change this for each imu sensor. 1 for shoulder, 2 for elbow and 3 for wrist.

  char anglexbuff[10];
  char angleybuff[10];
  char anglezbuff[10];
  
  char timebuff[10];
  
  if (!abc.connect(host, port)) {
    Serial.println("connection to server failed");
    //Serial.println("wait 5 sec...");
    delay(5000);
    //return;
  }
  else
  {
    long startTime = 0;
    long timeNow = 0;
    float timeNowSec = 0;
    bool startTransmission = false;
    while(1)
    {
      while(!startTransmission)
      {
        String reply;
        while (abc.available()) {
          char ch = static_cast<char>(abc.read());
          reply.concat(ch);
          reply.trim();
          if(reply.equals("Start") == 1)
          {
            Serial.println(reply);
            startTransmission = true;
            startTime = millis();
            break;
          }
        }
      }
        
      /* Get a new sensor event */
      //imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Vector<3> angle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      dtostrf(angle.x(), 4, 2, anglexbuff);
      dtostrf(angle.y(), 4, 2, angleybuff);
      dtostrf(angle.z(), 4, 2, anglezbuff);



      timeNow = millis() - startTime;
      timeNowSec = ((float)timeNow)/((float)1000);
      
      dtostrf(timeNowSec, 4, 3, timebuff);

      String data = "id=" + String(imuID) + "&x1=" + String(anglexbuff) + "&y1=" + String(angleybuff) + "&z1=" + String(anglezbuff) + "&t=" + String(timebuff);

      String lengthData = String(data.length());

      String Message = lengthData + data;

      abc.print(Message);
      //Serial.println(Message);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
  
}

void loop() {

  Serial.println("wait 5 sec...");
  delay(5000);

}
