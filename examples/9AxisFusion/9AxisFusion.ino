#include <ICM20948.h>

#define PIN_SDA 4
#define PIN_SCL 5

ICM20948 icm;

void fail()
{
    delay(10000);
    ESP.restart();
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    int code = icm.begin();

    switch (code)
    {
    case ICM_SUCCESS:
        Serial.println("Init ok");
        break;
    case ICM_BAD_WHOAMI:
        Serial.println("Bad WHOAMI");
        fail();
        break;
    case ICM_DMP_ERROR:
        Serial.println("Failed to initialize DMP");
        fail();
        break;
    case ICM_MAG_ERROR:
        Serial.println("Magnetometer not found");
        fail();
        break;
    case ICM_SERIAL_ERROR:
        Serial.println("Serial connection failure");
        fail();
        break;
    case ICM_SETUP_ERROR:
        Serial.println("Device setup failure");
        fail();
        break;
    }

    icm.startSensor(INV_SENSOR_TYPE_MAGNETOMETER, 16000);
    icm.startSensor(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 16000);
    icm.startSensor(INV_SENSOR_TYPE_ROTATION_VECTOR, 16000);
}

void loop()
{
    icm.update();

    if (icm.available())
    {
        Serial.printf("%f %f %f %f\n", icm.x(), icm.y()), icm.z(), icm.w());
        icm.clearAvailable();
    }

    delay(16);
}
