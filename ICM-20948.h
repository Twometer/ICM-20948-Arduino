#include "Invensense/Icm20948.h"
#include "Invensense/Icm20948MPUFifoControl.h"
#include "Invensense/SensorTypes.h"

#include <Wire.h>

#define EXPECTED_WHOAMI 0xEA

#define MAG_ADDRESS	0x0C
#define IMU_ADDRESS	0x69

#define ERR_OK 				0		// Initialized successfully, no errors
#define ERR_BAD_WHOAMI		-1		// WHOAMI from the IMU does not match
#define ERR_DMP_FAILED		-2		// Failed to initialize the DMP
#define ERR_AUX_FAILED		-3		// Failed to initialize auxiliary devices (compass etc.)

static const uint8_t icm_dmp3_image[] =
{
#include "Invensense/icm20948_img.dmp3a.h"
};

class ICM20948 {
private:
	int i2c_address = IMU_ADDRESS;
	int mag_address = MAG_ADDRESS;
	
	inv_icm20948_t icm_device;
	
public:
	void setI2cAddress(int addr) {
		i2c_address = addr;
	}
	
	void setMagAddress(int addr) {
		mag_address = addr;
	}
	
	int begin() {
		Wire.setClock(400000);
		
		struct inv_icm20948_serif icm20948_serif;
		icm20948_serif.context   = 0; /* no need */
		icm20948_serif.read_reg  = idd_io_hal_read_reg;
		icm20948_serif.write_reg = idd_io_hal_write_reg;
		icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
		icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */
		icm20948_serif.is_spi = false;
		
		icm_device.base_state.serial_interface = SERIAL_INTERFACE_I2C;

		inv_icm20948_reset_states(&icm_device, &icm20948_serif);
		inv_icm20948_register_aux_compass(&icm_device, COMPASS_TYPE, COMPASS_ADDR);
		
		uint8_t whoami = 0xff;
		inv_icm20948_get_whoami(&icm_device, &whoami);
		
		if (whoami != EXPECTED_WHOAMI) 
			return ERR_BAD_WHOAMI;
		
		inv_icm20948_init_matrix(&icm_device);
		
		int rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
		if (rc != 0)
			return ERR_DMP_FAILED;
		
		inv_icm20948_register_aux_compass(&icm_device, COMPASS_TYPE, COMPASS_ADDR);
		rc = inv_icm20948_initialize_auxiliary(&icm_device);
		if (rc != 0)
			return ERR_AUX_FAILED;
		
		icm20948_apply_mounting_matrix();
		icm20948_set_fsr();
		inv_icm20948_init_structure(&icm_device);
		
		if (icm_device.selftest_done && !icm_device.offset_done) {
			// If we've run selftest and not already set the offset.
			inv_icm20948_set_offset(&icm_device, unscaled_bias);
			icm_device.offset_done = 1;
		}
		return ERR_OK;
	}
	
	int enableSensor(int sensor) {
		return inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(sensor), 1);
	}
	
	int setSampleRate(int sensor, int hz) {
		int period = 1000 / hz;
		return inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(sensor), period);
	}
	
	void update() {
	}
	
};
