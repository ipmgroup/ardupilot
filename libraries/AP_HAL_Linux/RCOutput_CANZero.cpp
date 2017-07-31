#include "RCOutput_CANZero.h"

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_channel_count, uint8_t can_channel_count){
		// Initialize sysfs to use PWM.
		sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_channel_count);
		this->pwm_channel_count = pwm_channel_count;
		this->can_channel_count = can_channel_count;
	}

	RCOutput_CANZero::~RCOutput_CANZero(){
		sysfs_out.~RCOutput_Sysfs();
	}

	void init();
	void set_freq(uint32_t chmask, uint16_t freq_hz);
	uint16_t get_freq(uint8_t ch);
	void enable_ch(uint8_t ch);
	void disable_ch(uint8_t ch);
	void write(uint8_t ch, uint16_t period_us);
	uint16_t read(uint8_t ch);
	void read(uint16_t *period_us, uint8_t len);
	void cork(void) override;
	void push(void) override;

}
