#include "RCOutput_CANZero.h"

#include "ifaddrs.h"
#include "unistd.h"
#include "net/if.h"
#include "stdio.h"
#include "time.h"
#include "sys/poll.h"
#include "dirent.h"
#include "string"

//
#define CAN_SYNC_MSG "080#00" // Message to send during initialization to get the node IDs of all available CAN devices.

#define CAN_SYNC_ID (0x080)
#define CAN_SYNC_DATA_TYPE (uint8_t)
#define CAN_SYNC_DATA (0x00)
#define CAN_SET_RPM_ID (0x580)
#define CAN_SET_RPM_DATA_TYPE (int32_t)
#define CAN_SET_RPM_META (0x23FF6000)//(0x60FF) 001 0 00 1 1 1111111101100000 00000000
#define CAN_SET_RPMPS_ID (0x580)
#define CAN_SET_RPMPS_DATA_TYPE (uint32_t)
#define CAN_SET_RPMPS_META (0x23836000)//(0x6083)
#define CAN_SET_CTL_ID (0x580)
#define CAN_SET_CTL_DATA_TYPE (uint16_t)
#define CAN_SET_CTL_META (0x23406000)//(0x6040) 00100011010000000110000000000000
#define CAN_SET_CTL_ON_DATA (0x03)
#define CAN_SET_CTL_OFF_DATA (0x00)

#define CAN_GET_RPM_ID (0x600)
#define CAN_GET_RPM_DATA_TYPE (int32_t)
#define CAN_GET_RPM_META (0x406C6000)//(0x606C)
#define CAN_GET_MAX_RPMPS_ID (0x600)
#define CAN_GET_MAX_RPMPS_DATA_TYPE (float)
#define CAN_GET_MAX_RPMPS_META (4001200F)//(0x2001.F)

#define CAN_SET_MSG_LEN(L) (((uint32_t)(4-L))<<26) // | with META to get correct metadata.
#define CAN_GET_MSG_LEN(M) (4-(((uint32_t)M)>>26))
//

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8
#define CAN_MTU (sizeof(struct can_frame))
//#define CANFD_MAX_DLEN 64
//#define CANFD_MTU (sizeof(struct canfd_frame))

#define SCAN_TIMEOUT_TOTAL ((float)3.0) //sec
#define SCAN_TIMEOUT_MSG ((int)200) //ms

#define PWM_CHIP_PATH "/sys/class/pwm/"
#define PWM_CHIP_BASE_NAME "pwmchip"

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count, uint8_t can_ch_count)
	{
		// Initialize sysfs to use PWM.
		sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_ch_count);
		this->pwm_channel_count_max = pwm_ch_count;
		this->can_channel_count_max = can_ch_count;
		this->channel_count_max = pwm_ch_count+can_ch_count;
		this->ch_inf = (ChannelInfo*)calloc(channel_count_max, sizeof(ChannelInfo));
	}

	RCOutput_CANZero::~RCOutput_CANZero()
	{
		if(can_socket != 0) close(can_socket);
		if(ch_inf != NULL) free(ch_inf);
		if(sysfs_out != NULL) (*sysfs_out).~RCOutput_Sysfs();
	}

	void RCOutput_CANZero::init()
	{
		struct ifaddrs if_list;
		struct ifaddrs* if_list_ptr = &if_list;
		getifaddrs(&if_list_ptr); // Get list of available interfaces.

		// Searching for a CAN interface.
		struct ifaddrs* ifa_current = if_list_ptr;
		while(ifa_current != NULL){
			char* pos = strstr(ifa_current->ifa_name, "can");
			if(pos != NULL && strlen(ifa_current->ifa_name) < 7){
				strncpy(can_ifa_name, ifa_current->ifa_name, 8);
				break;
			}
			ifa_current = ifa_current->ifa_next;
		}

		// Creating a socket connection and binding it to the available interface.
		printf("Connecting to socket and binding to interface \"%s\".\n", ifa_current->ifa_name);
		can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		can_addr_output.can_family = AF_CAN;
		can_addr_output.can_ifindex = if_nametoindex(can_ifa_name);
		bind(can_socket, (struct sockaddr *)&can_addr_output, sizeof(can_addr_output));

		std::map<uint8_t, uint8_t> ids;
		scan_devices(&ids);

		//printf("Discovered devices:");
		std::pair<int,std::map<uint8_t,uint8_t>::iterator> it(0,ids.begin());
		for(; it.first < can_channel_count_max && it.second != ids.end();
				it.first++, it.second++){
			ch_inf[it.first].hw_chan = it.second->first; // Copying ID of the device to the channel info.
			ch_inf[it.first].can = 1; // Marking the channel as a CAN channel.
			//printf(" [%x|%x]", ch_inf[it.first].hw_chan, ch_inf[it.first].can);
		}
		can_channel_count = it.first;
		//printf("\n");

		// Doesn't seem to work for symlinks.
//		DIR *dir;
//		struct dirent *ent;
//		if((dir = opendir(PWM_CHIP_PATH)) != NULL){
//			while((ent = readdir(dir)) != NULL){
//				printf("filename: %s\n", ent->d_name);
//				if(std::string(ent->d_name).compare(0, strlen(PWM_CHIP_BASE_NAME), PWM_CHIP_BASE_NAME, 0, strlen(PWM_CHIP_BASE_NAME))){
//					strncpy(pwmchip, ent->d_name, 10);
//					break;
//				}
//			}
//		}

		//strncpy(pwmchip, PWM_CHIP_BASE_NAME, 9);
		//strcat(pwmchip, "0");
		//printf("pwmchip: %s\n", pwmchip);
		for(int i = 0; i < pwm_channel_count_max; i++){
			ch_inf[i+can_channel_count].hw_chan = i;
			ch_inf[i+can_channel_count].can = 0;
		}
		pwm_channel_count = pwm_channel_count_max;
		channel_count = can_channel_count + pwm_channel_count;

		sysfs_out->init();
		freeifaddrs(if_list_ptr);
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
		int freq = 0;
		if(ch_inf[ch].can){
			freq = ch_inf[ch].freq_hz;
		}else{
			freq = sysfs_out->get_freq(ch_inf[ch].hw_chan);
			ch_inf[ch].freq_hz = freq;
		}
		return freq;
	}

	void RCOutput_CANZero::enable_ch(uint8_t ch)
	{
		if(ch_inf[ch].can){
			//TODO: send controlword 3.
		}else{
			sysfs_out->enable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::disable_ch(uint8_t ch)
	{
		if(ch_inf[ch].can){
			//TODO: send controlword 0.
		}else{
			sysfs_out->disable_ch(ch_inf[ch].hw_chan);
		}
	}

	void RCOutput_CANZero::write(uint8_t ch, uint16_t period_us)
	{
		if(ch_inf[ch].can){

		}else{
			sysfs_out->write(ch_inf[ch].hw_chan, period_us);
		}
	}

	uint16_t RCOutput_CANZero::read(uint8_t ch)
	{
		if(ch_inf[ch].can){

		}else{
			sysfs_out->read(ch_inf[ch].hw_chan);
		}
		return 0;
	}

	void RCOutput_CANZero::read(uint16_t *period_us, uint8_t len)
	{
		sysfs_out->read(period_us, len);
	}

	void RCOutput_CANZero::cork(void)
	{
		sysfs_out->cork();
	}

	void RCOutput_CANZero::push(void)
	{
		sysfs_out->push();
	}

	// https://github.com/linux-can/can-utils/blob/master/lib.c
	int RCOutput_CANZero::parse_canframe(char *cs, struct can_frame *cf)
	{
		/* documentation see lib.h */

		int i, idx, dlen, len;
		int maxdlen = CAN_MAX_DLEN;
		int ret = CAN_MTU;
		unsigned char tmp;

		len = strlen(cs);

		memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

		if (len < 4)
			return 0;

		if (cs[3] == CANID_DELIM) { /* 3 digits */

			idx = 4;
			for (i=0; i<3; i++){
				if ((tmp = asc2nibble(cs[i])) > 0x0F)
					return 0;
				cf->can_id |= (tmp << (2-i)*4);
			}

		} else if (cs[8] == CANID_DELIM) { /* 8 digits */

			idx = 9;
			for (i=0; i<8; i++){
				if ((tmp = asc2nibble(cs[i])) > 0x0F)
					return 0;
				cf->can_id |= (tmp << (7-i)*4);
			}
			if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
				cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended oframe */

		} else
			return 0;

		if((cs[idx] == 'R') || (cs[idx] == 'r')){ /* RTR oframe */
			cf->can_id |= CAN_RTR_FLAG;

			/* check for optional DLC value for CAN 2.0B frames */
			if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
				cf->can_dlc = tmp;

			return ret;
		}

		// Not using CAN FD
//		if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */
//
//			maxdlen = CANFD_MAX_DLEN;
//			ret = CANFD_MTU;
//
//			/* CAN FD frame <canid>##<flags><data>* */
//			if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
//				return 0;
//
//			//cf->flags = tmp; // No field "flags" in struct can_frame.
//			idx += 2;
//		}

		for (i=0, dlen=0; i < maxdlen; i++){

			if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
				idx++;

			if(idx >= len) /* end of string => end of data */
				break;

			if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
				return 0;
			cf->data[i] = (tmp << 4);
			if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
				return 0;
			cf->data[i] |= tmp;
			dlen++;
		}
		cf->can_dlc = dlen;

		return ret;
	}

	// https://github.com/linux-can/can-utils/blob/master/slcanpty.c
	int RCOutput_CANZero::asc2nibble(char c)
	{

		if ((c >= '0') && (c <= '9'))
			return c - '0';

		if ((c >= 'A') && (c <= 'F'))
			return c - 'A' + 10;

		if ((c >= 'a') && (c <= 'f'))
			return c - 'a' + 10;

		return 16; /* error */
	}

	// Returns 0 if the single message timeout was reached (likely no more devices online in the network or they reply very slowly).
	// Returns 1 if the total timeout was reached (possibly more devices available and/or the network is being flooded by some nodes).
	int RCOutput_CANZero::scan_devices(std::map<uint8_t, uint8_t> *ids)
	{
		int ret = 1;
		unsigned int addr_len = sizeof(struct sockaddr_can);
		struct can_frame frame_output;
		struct can_frame frame_input;
		//int required_mtu = parse_canframe((char*)CAN_SYNC_MSG, &frame_output);
		frame_output.can_id = CAN_SYNC_ID;
		frame_output.can_dlc = 0;
		//TODO: implement proper frame generation.

		printf("frame_output.data at id %x:", frame_output.can_id);
		for(int i = 0; i < frame_output.can_dlc; i++){
			printf(" %x", frame_output.data[i]);
		}
		printf("\n");

		//::write(can_socket, &frame_output, required_mtu);
		::write(can_socket, &frame_output, CAN_MTU);

		struct pollfd fds = {can_socket, POLLIN, 0};
		struct timespec tps = {0, 0}; // Start time.
		float start_time = 0; // Start time in s.
		struct timespec tpe = {0, 0}; // Current time/end time.
		float dt = 0; // Time difference in s.
		const float nsps = 1000000000; // nanoseconds per second

		clock_gettime(CLOCK_MONOTONIC, &tps);
		start_time = (float(tps.tv_nsec)/nsps + tps.tv_sec);
		while(dt < SCAN_TIMEOUT_TOTAL){
			if(0 < poll(&fds, 1, SCAN_TIMEOUT_MSG)){
				::recvfrom(can_socket, &frame_input, sizeof(struct can_frame), 0, (struct sockaddr*)&can_addr_input, &addr_len);
				(*ids)[frame_input.can_id & 0x7F] = frame_input.can_id & 0x7F; // Creates an entry for the given ID if not already present.
			}else{
				ret = 0;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &tpe);
			dt = float((float(tpe.tv_nsec)/nsps + tpe.tv_sec) - start_time);
		}

		return ret;
	}
}
