#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

static const uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
};