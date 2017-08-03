#include "RCOutput_CANZero.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "linux/can.h"
#include "ifaddrs.h"
#include "unistd.h"

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count, uint8_t can_ch_count)
	{
		// Initialize sysfs to use PWM.
		//sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_ch_count);
		this->pwm_channel_count = pwm_ch_count;
		this->can_channel_count = can_ch_count;
		this->ch_inf = (ChannelInfo*)calloc(pwm_ch_count+can_ch_count, sizeof(ChannelInfo));
	}

	RCOutput_CANZero::~RCOutput_CANZero()
	{
		if(can_socket != 0) close(can_socket);
		if(ch_inf != NULL) free(ch_inf);
		//sysfs_out.~RCOutput_Sysfs();
	}

	void RCOutput_CANZero::init()
	{
		struct ifaddrs if_list;
		struct ifaddrs* if_list_ptr = &if_list;
		getifaddrs(&if_list_ptr); // Get list of available devices.
		can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

		//sysfs_out.init();
		freeifaddrs(&if_list);
	}

	void RCOutput_CANZero::set_freq(uint32_t chmask, uint16_t freq_hz)
	{
		//;
	}

	uint16_t RCOutput_CANZero::get_freq(uint8_t ch)
	{
		return 0;
	}

	void RCOutput_CANZero::enable_ch(uint8_t ch)
	{
		//;
	}

	void RCOutput_CANZero::disable_ch(uint8_t ch)
	{
		//;
	}

	void RCOutput_CANZero::write(uint8_t ch, uint16_t period_us)
	{
		//;
	}

	uint16_t RCOutput_CANZero::read(uint8_t ch)
	{
		return 0;
	}

	void RCOutput_CANZero::read(uint16_t *period_us, uint8_t len)
	{
		//;
	}

	void RCOutput_CANZero::cork(void)
	{

	}

	void RCOutput_CANZero::push(void)
	{

	}

}
