#ifndef ICM_20948_ARDUINO_
#define ICM_20948_ARDUINO_

#include "Icm20948.h"
#include "SensorTypes.h"
#include "Icm20948MPUFifoControl.h"

#include "IcmVectors.h"

#include <Wire.h>

#define EXPECTED_WHOAMI 0xEA

#define ERR_OK                 0        // Initialized successfully, no errors
#define ERR_BAD_WHOAMI        -1        // WHOAMI from the IMU does not match
#define ERR_DMP_FAILED        -2        // Failed to initialize the DMP
#define ERR_AUX_FAILED        -3        // Failed to initialize auxiliary devices (compass etc.)

/* Static definitions
------------------------------------ */
static uint8_t icm_convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_BAC,
	INV_SENSOR_TYPE_STEP_DETECTOR,
	INV_SENSOR_TYPE_STEP_COUNTER,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_ROTATION_VECTOR,
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
	INV_SENSOR_TYPE_MAGNETOMETER,
	INV_SENSOR_TYPE_SMD,
	INV_SENSOR_TYPE_PICK_UP_GESTURE,
	INV_SENSOR_TYPE_TILT_DETECTOR,
	INV_SENSOR_TYPE_GRAVITY,
	INV_SENSOR_TYPE_LINEAR_ACCELERATION,
	INV_SENSOR_TYPE_ORIENTATION,
	INV_SENSOR_TYPE_B2S
};

static const uint8_t icm_dmp3_image[] =
{
#include "icm20948_img.dmp3a.h"
};

static int icm_unscaled_bias[3 * 2];

static const float icm_default_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};

/* Main lib API class
------------------------------------ */
class ICM20948 {
private:
    uint8_t i2c_address = 0x69;
    uint8_t mag_address = 0x0C;
	inv_icm20948_compass_id mag_model = INV_ICM20948_COMPASS_ID_AK09916;

    inv_icm20948_t icm_device;

	vec3 accel{};
	vec3 gyro{};
	vec3 mag{};
	vec4 rotation{};

	bool isAvailable = false;

public:
	int getAddress() {
		return i2c_address;
	}

	vec3 &getAccelerometer() {
		return accel;
	}

	vec3 &getGyroscope() {
		return gyro;
	}

	vec3 &getMagnetometer() {
		return mag;
	}

	vec4 &getRotation() {
		return rotation;
	}

    void setI2cAddress(int addr) {
        i2c_address = addr;
    }

    void setMagAddress(int addr) {
        mag_address = addr;
    }

	void setMagModel(inv_icm20948_compass_id model) {
		mag_model = model;
	}

	bool available() {
		return isAvailable;
	}

	void setAvailable() {
		isAvailable = true;
	}

	void clearAvailable() {
		isAvailable = false;
	}

    // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
    void setAccelerometerRange(int range) {
        inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&range);
    	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&range);
    }


    // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
    void setGyroscopeRange(int range) {
		inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&range);
		inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&range);
    	inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&range);
    }


    void setMountingMatrix(const float* matrix3x3) {
	      for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
			  inv_icm20948_set_matrix(&icm_device, matrix3x3, (inv_icm20948_sensor)i);
		  }
    }


    int setSampleRate(int sensor, int hz) {
        int period = 1000 / hz;
        return inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(sensor), period);
    }


    int enableSensor(int sensor) {
        return inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(sensor), 1);
    }

	int disableSensor(int sensor) {
		return inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(sensor), 0);
	}


    int begin() {
        Wire.setClock(400000);

        /* Configuring lib
        ------------------------------------ */
        struct inv_icm20948_serif icm20948_serif;
        icm20948_serif.context   = this; /* no need */
        icm20948_serif.read_reg  = idd_io_hal_read_reg;
        icm20948_serif.write_reg = idd_io_hal_write_reg;
        icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
        icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */
        icm20948_serif.is_spi = false;

        icm_device.base_state.serial_interface = SERIAL_INTERFACE_I2C;

        inv_icm20948_reset_states(&icm_device, &icm20948_serif);
		inv_icm20948_register_aux_compass(&icm_device, mag_model, mag_address);

        /* Check WHOAMI
        ------------------------------------ */
        uint8_t whoami = 0xff;
        inv_icm20948_get_whoami(&icm_device, &whoami);

        if (whoami != EXPECTED_WHOAMI)
            return ERR_BAD_WHOAMI;

		delay(1000);

        inv_icm20948_init_matrix(&icm_device);

        /* Initialize
        ------------------------------------ */
        int rc = inv_icm20948_initialize(&icm_device, icm_dmp3_image, sizeof(icm_dmp3_image));
        if (rc != 0)
            return ERR_DMP_FAILED;

        /* Register compass
        ------------------------------------ */
        inv_icm20948_register_aux_compass(&icm_device, mag_model, mag_address);
        rc = inv_icm20948_initialize_auxiliary(&icm_device);
        if (rc != 0)
            return ERR_AUX_FAILED;

        /* Default config for device
        ------------------------------------ */
        setMountingMatrix(icm_default_mounting_matrix);
        setAccelerometerRange(4);
        setGyroscopeRange(2000);

        /* Finishing
        ------------------------------------ */
        inv_icm20948_init_structure(&icm_device);

        if (icm_device.selftest_done && !icm_device.offset_done) {
            // If we've run selftest and not already set the offset.
            inv_icm20948_set_offset(&icm_device, icm_unscaled_bias);
            icm_device.offset_done = 1;
        }
        return ERR_OK;
    }


    void update() {
		inv_icm20948_poll_sensor(&icm_device, this, handle_sensor);
    }


private:
	static void handle_sensor(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg) {
		ICM20948* parent_class = (ICM20948*) context;

		float raw_bias_data[6];
 		inv_sensor_event_t event{};
		uint8_t sensor_id = icm_convert_to_generic_ids[sensortype];

		event.sensor = sensor_id;
		event.timestamp = timestamp;

		Serial.print((int) context); Serial.print(" -> "); Serial.println(sensor_id);

		switch (sensor_id) {
			case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
			case INV_SENSOR_TYPE_ACCELEROMETER:
			{
				memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
				memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
				vec3 &dst = parent_class->getAccelerometer();
				dst.x = event.data.acc.vect[0];
				dst.y = event.data.acc.vect[1];
				dst.z = event.data.acc.vect[2];
				parent_class->setAvailable();
				break;
			}
			case INV_SENSOR_TYPE_MAGNETOMETER:
			{
				memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
  				memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
				vec3 &dst = parent_class->getMagnetometer();
				dst.x = event.data.mag.vect[0];
				dst.y = event.data.mag.vect[1];
				dst.z = event.data.mag.vect[2];
				parent_class->setAvailable();
				break;
			}
			case INV_SENSOR_TYPE_ROTATION_VECTOR:
			{
				memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
  				memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
				vec4 &dst = parent_class->getRotation();
				dst.x = event.data.quaternion.quat[0];
				dst.y = event.data.quaternion.quat[1];
				dst.z = event.data.quaternion.quat[2];
				dst.w = event.data.quaternion.quat[3];
				parent_class->setAvailable();
				break;
			}
			case INV_SENSOR_TYPE_GYROSCOPE:
			{
				memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
      			memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
				vec3 &dst = parent_class->getGyroscope();
				dst.x = event.data.gyr.vect[0];
				dst.y = event.data.gyr.vect[1];
				dst.z = event.data.gyr.vect[2];
				parent_class->setAvailable();
				break;
			}
		}
	}

	static int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen) {
		ICM20948* parent_class = (ICM20948*) context;
		int i2c_address = parent_class->getAddress();

		Wire.beginTransmission(i2c_address);
		Wire.write(reg);
		Wire.endTransmission(false);

		uint32_t offset = 0;
		uint32_t num_received = Wire.requestFrom(i2c_address, rlen);
		if (num_received == rlen) {
			for (uint8_t i = 0; i < rlen; i++)
				rbuffer[i] = Wire.read();
		} else return -1;
	}

	static int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen) {
		ICM20948* parent_class = (ICM20948*) context;

		Wire.beginTransmission(parent_class->getAddress());
		Wire.write(reg);
		Wire.write(wbuffer, wlen);
		Wire.endTransmission();
	}

	static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor) {
    	switch (sensor) {
    		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
        		return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
            case INV_SENSOR_TYPE_RAW_GYROSCOPE:
                return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
            case INV_SENSOR_TYPE_ACCELEROMETER:
                return INV_ICM20948_SENSOR_ACCELEROMETER;
            case INV_SENSOR_TYPE_GYROSCOPE:
                return INV_ICM20948_SENSOR_GYROSCOPE;
            case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
                return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
            case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
                return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
            case INV_SENSOR_TYPE_BAC:
                return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
            case INV_SENSOR_TYPE_STEP_DETECTOR:
                return INV_ICM20948_SENSOR_STEP_DETECTOR;
            case INV_SENSOR_TYPE_STEP_COUNTER:
                return INV_ICM20948_SENSOR_STEP_COUNTER;
            case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
                return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
            case INV_SENSOR_TYPE_ROTATION_VECTOR:
                return INV_ICM20948_SENSOR_ROTATION_VECTOR;
            case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
                return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
            case INV_SENSOR_TYPE_MAGNETOMETER:
                return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
            case INV_SENSOR_TYPE_SMD:
                return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
            case INV_SENSOR_TYPE_PICK_UP_GESTURE:
                return INV_ICM20948_SENSOR_FLIP_PICKUP;
            case INV_SENSOR_TYPE_TILT_DETECTOR:
                return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
            case INV_SENSOR_TYPE_GRAVITY:
                return INV_ICM20948_SENSOR_GRAVITY;
            case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
                return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
            case INV_SENSOR_TYPE_ORIENTATION:
                return INV_ICM20948_SENSOR_ORIENTATION;
            case INV_SENSOR_TYPE_B2S:
                return INV_ICM20948_SENSOR_B2S;
			default:
                return INV_ICM20948_SENSOR_MAX;
		}
	}
};

/* Declarations for Invensense lib
------------------------------------ */
void inv_icm20948_sleep(int ms) {
	delay(ms);
}

void inv_icm20948_sleep_us(int us) {
	uint32_t start = micros();
	while (micros() - start < us) {
		yield();
	}
}

uint64_t inv_icm20948_get_time_us(void) {
	return micros();
}

#endif
