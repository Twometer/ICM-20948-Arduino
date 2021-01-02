#include <ICM20948.h>

#define PIN_SDA	 	4
#define PIN_SCL 	5

ICM20948 icm;

void fail() {
	delay(10000);
	ESP.restart();
}

void setup() {
	Serial.begin(115200);
	Wire.begin(PIN_SDA, PIN_SCL);
	int code = icm.begin();

	switch (code) {
		case ERR_OK:
			Serial.println("Init ok");
			break;
		case ERR_BAD_WHOAMI:
			Serial.println("Bad WHOAMI");
			fail();
			break;
		case ERR_DMP_FAILED:
			Serial.println("Failed to initialize DMP");
			fail();
			break;
		case ERR_AUX_FAILED:
			Serial.println("Auxiliary devices not found");
			fail();
			break;
	}

	icm.enableSensor(INV_SENSOR_TYPE_GYROSCOPE);
	icm.enableSensor(INV_SENSOR_TYPE_ACCELEROMETER);
	icm.enableSensor(INV_SENSOR_TYPE_MAGNETOMETER);
	icm.enableSensor(INV_SENSOR_TYPE_ROTATION_VECTOR);
	icm.setSampleRate(INV_SENSOR_TYPE_ROTATION_VECTOR, 60);
}

void loop() {
	icm.update();

	if (icm.available()) {
		vec4 &quat = icm.getRotation();
		Serial.print(quat.x);
		Serial.print(", ");
		Serial.print(quat.y);
		Serial.print(", ");
		Serial.print(quat.z);
		Serial.print(", ");
		Serial.println(quat.w);
		icm.clearAvailable();
	}
}
