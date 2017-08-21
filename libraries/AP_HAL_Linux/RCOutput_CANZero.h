#pragma once

#include "AP_HAL_Linux.h"
//#include "PWM_Sysfs.h"
#include "RCOutput_Sysfs.h"

#include "linux/can.h"
#include "map"

namespace Linux {

class RCOutput_CANZero : public AP_HAL::RCOutput {
public:
	RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count, uint8_t can_ch_count);
	~RCOutput_CANZero();

	static RCOutput_CANZero *from(AP_HAL::RCOutput *rcoutput)
	{
		return static_cast<RCOutput_CANZero *>(rcoutput);
	}

	void init() override;
	void set_freq(uint32_t chmask, uint16_t freq_hz) override;
	uint16_t get_freq(uint8_t ch) override;
	void enable_ch(uint8_t ch) override;
	void disable_ch(uint8_t ch) override;
	void write(uint8_t ch, uint16_t period_us) override;
	uint16_t read(uint8_t ch) override;
	void read(uint16_t *period_us, uint8_t len) override;
	void cork(void) override;
	void push(void) override;

private:
	// Instance of RCOutput_Sysfs to control the PWM.
	RCOutput_Sysfs *sysfs_out;
	uint8_t pwm_channel_count_max;
	uint8_t can_channel_count_max;
	uint8_t channel_count_max;
	uint8_t pwm_channel_count;
	uint8_t can_channel_count;
	uint8_t channel_count;
	int asc2nibble(char c);
	int parse_canframe(char *cs, struct can_frame *cf);
	int scan_devices(std::map<uint8_t, uint8_t> *ids); // Scans for available CAN devices.

	// Holds information about the assignment of PWM/CAN channels.
	typedef struct ChannelInfo {
		uint8_t can:1; // Indicates if the signals to this channel are meant to be sent via CAN.
		uint8_t hw_chan:7; // Indicates the corresponding PWM channel in the hardware or CAN node id.
		uint16_t freq_hz;
	} ChannelInfo;

	ChannelInfo *ch_inf;
	int can_socket = 0;
	char can_ifa_name[8]; // First CAN interface found in init().
	struct sockaddr_can can_addr_output;
	struct sockaddr_can can_addr_input;

	//struct can_frame frame_output;
	//struct can_frame frame_input;

	char pwmchip[10];
};

}
