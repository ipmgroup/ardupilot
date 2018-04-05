#include "RCOutput_CANZero.h"

#include "ifaddrs.h"
#include "stdint.h"
#include "unistd.h"
#include "net/if.h"
#include "stdio.h"
#include "time.h"
#include "sys/poll.h"
//#include "dirent.h"
#include "string"
#include "linux/can/raw.h"
#include "AP_Math/AP_Math.h"
#include "AP_Param/AP_Param.h"
#include "AP_BoardConfig/AP_BoardConfig.h"
#include "AP_Vehicle/AP_Vehicle_Type.h"

extern const AP_HAL::HAL& hal;

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count)
	{
		// Initialize sysfs to use PWM.
		sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_ch_count);
		this->pwm_channel_count_max = pwm_ch_count;
	}

	RCOutput_CANZero::~RCOutput_CANZero()
	{
		if(ch_inf != NULL) free(ch_inf);
		if(_pending != NULL) free(_pending);
		if(sysfs_out != NULL) (*sysfs_out).~RCOutput_Sysfs();
	}

	void RCOutput_CANZero::init()
	{
		sysfs_out->init();

		this->ap_co = AP_CANopen::get_CANopen(hal.can_mgr[0]);

		can_channel_count = ap_co->get_node_count();

		this->ch_inf = (ChannelInfo*)calloc(can_channel_count+pwm_channel_count_max, sizeof(ChannelInfo));
		this->_pending = (uint16_t*)calloc(can_channel_count+pwm_channel_count_max, sizeof(uint16_t));

		for(int i = 0; i < can_channel_count; i++){
			ch_inf[i].hw_chan = i;
			ch_inf[i].can = 1;
		}

		for(int i = 0; i < pwm_channel_count_max; i++){
			ch_inf[i+can_channel_count].hw_chan = i;
			ch_inf[i+can_channel_count].can = 0;
		}
		pwm_channel_count = pwm_channel_count_max;
		channel_count = can_channel_count + pwm_channel_count;
	}

	void RCOutput_CANZero::set_freq(uint32_t chmask, uint16_t freq_hz)
	{
	    for (uint8_t i = 0; i < channel_count; i++) {
	        if (chmask & 1 << i) {
	        	ch_inf[i].freq_hz = freq_hz;
	        	if(ch_inf[i].can){
	        		// Do nothing.
	        	}else{
	        		sysfs_out->set_freq(chmask >> can_channel_count, freq_hz);
	        	}
	        }
	    }
	}

	uint16_t RCOutput_CANZero::get_freq(uint8_t ch)
	{
		if(ch >= channel_count){
			return 0;
		}
		if(ch_inf[ch].can){
			// Do nothing.
		}else{
			ch_inf[ch].freq_hz = sysfs_out->get_freq(ch_inf[ch].hw_chan);
		}
		return ch_inf[ch].freq_hz;
	}

	void RCOutput_CANZero::enable_ch(uint8_t ch)
	{
		if(ch >= channel_count){
			return;
		}
		if(ch_inf[ch].can){
			//printf("Enabling channel %d.\n", ch_inf[ch].hw_chan);
			can_frame frame_output;
			generate_frame<CAN_SET_CTL_DATA_TYPE>(&frame_output, CAN_SET_CTL_ID, ch_inf[ch].hw_chan, CAN_SET_CTL_META, ctl_on);
			::write(can_socket, &frame_output, CAN_MTU);
		}else{
			sysfs_out->enable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::disable_ch(uint8_t ch)
	{
		if(ch >= channel_count){
			return;
		}
		if(ch_inf[ch].can){
			can_frame frame_output;
			generate_frame<CAN_SET_CTL_DATA_TYPE>(&frame_output, CAN_SET_CTL_ID, ch_inf[ch].hw_chan, CAN_SET_CTL_META, CAN_SET_CTL_OFF_DATA);
			::write(can_socket, &frame_output, CAN_MTU);
		}else{
			sysfs_out->disable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::write(uint8_t ch, uint16_t period_us)
	{
		if(ch >= channel_count){
			return;
		}
		if(_corked){
	        _pending[ch] = period_us;
	        _pending_mask |= (1U<<ch);
		}else{
			_write(ch, period_us);
		}
	}

	void RCOutput_CANZero::_write(uint8_t ch, uint16_t period_us)
	{
		ch_inf[ch].ppm = period_us;
		if(ch_inf[ch].can){
			ap_co->rco_write(period_us, ch_inf[ch].hw_chan);
		}else{
			sysfs_out->write(ch_inf[ch].hw_chan, period_us);
		}
	}

	uint16_t RCOutput_CANZero::read(uint8_t ch)
	{
		if(ch >= channel_count){
			return 1000;
		}
		uint16_t ppm;
		if(ch_inf[ch].can){
			ppm = ch_inf[ch].ppm;
		}else{
			ppm = sysfs_out->read(ch_inf[ch].hw_chan);
		}
		return ppm;
	}

	void RCOutput_CANZero::read(uint16_t *period_us, uint8_t len)
	{
	    for (int i = 0; i < MIN(len, channel_count); i++) {
	        period_us[i] = read(i);
	    }
	    for (int i = channel_count; i < len; i++) {
	        period_us[i] = 1000;
	    }
	}

	void RCOutput_CANZero::cork(void)
	{
		//printf("cork()\n");
		_corked = true;
		sysfs_out->cork();
	}

	void RCOutput_CANZero::push(void)
	{
	    if (!_corked) {
	        return;
	    }
	    //printf("push()\n");
	    for (uint8_t i=0; i<channel_count; i++) {
	        if (((1U<<i) & _pending_mask)/* && ch_inf[i].can*/) {
				_write(i, _pending[i]);
	        }
	    }
	    sysfs_out->push();
	    _pending_mask = 0;
	    _corked = false;
	}

	template<typename T> void RCOutput_CANZero::set_acceleration(uint8_t ch, T acc)
	{
		if(ch_inf[ch].can){
			can_frame frame_output;
			generate_frame<CAN_SET_RPMPS_DATA_TYPE>(&frame_output, CAN_SET_RPMPS_ID, ch_inf[ch].hw_chan, CAN_SET_RPMPS_META, acc);
			::write(can_socket, &frame_output, CAN_MTU);
		}
	}

	template<typename T> void RCOutput_CANZero::set_acceleration_all(T acc)
	{
		for(uint8_t ch = 0; ch < channel_count; ch++){
			if(ch_inf[ch].can){
				can_frame frame_output;
				generate_frame<CAN_SET_RPMPS_DATA_TYPE>(&frame_output, CAN_SET_RPMPS_ID, ch_inf[ch].hw_chan, CAN_SET_RPMPS_META, acc);
				::write(can_socket, &frame_output, CAN_MTU);
			}
		}
	}

	template<typename T> T RCOutput_CANZero::ppm_to_rpm(uint16_t pulse_width)
	{
		T rpm = ((float(max_rpm))/500)*(float(pulse_width)-1500);
		//printf("ppm: %d\nrpm: %d\nmax rpm: %d\n", pulse_width, rpm, max_rpm);
		return rpm;//((float(max_rpm))/500)*(float(pulse_width)-1500);
	}

	template<typename T> uint16_t RCOutput_CANZero::rpm_to_ppm(T rpm)
	{
		return (float(rpm)/float(max_rpm))*500+1500;
	}

	template<typename T> void RCOutput_CANZero::data_array_to_var(uint8_t *data, T *var){
		*var = 0;
		for(int i = 0; i < sizeof(T); i++){
			//printf("%02x", data[i]);
			*var |= (T)((((uint64_t)data[i])&0xFF)<<i*8);
		}
		//printf("\n");
	}
}
