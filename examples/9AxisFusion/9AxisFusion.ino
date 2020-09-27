#include <ICM-20948.h>

ICM20948 icm;

void setup() {
	icm.begin();
	icm.enableSensor(INV_SENSOR_TYPE_GYROSCOPE);
	icm.enableSensor(INV_SENSOR_TYPE_ACCELEROMETER);
	icm.enableSensor(INV_SENSOR_TYPE_MAGNETOMETER);
	icm.enableSensor(INV_SENSOR_TYPE_ROTATION_VECTOR);
	icm.setSampleRate(INV_SENSOR_TYPE_ROTATION_VECTOR, 60);
}

void loop() {
	icm.update();
}
