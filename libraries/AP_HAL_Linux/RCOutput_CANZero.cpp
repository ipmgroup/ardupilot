#include "RCOutput_CANZero.h"

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8
#define CAN_MTU (sizeof(struct can_frame))
//#define CANFD_MAX_DLEN 64
//#define CANFD_MTU (sizeof(struct canfd_frame))

#define CAN_SYNC_MSG "080#00"
#define SCAN_TIMEOUT_TOTAL ((float)3.0) //sec
#define SCAN_TIMEOUT_MSG ((int)200) //ms

namespace Linux {

	RCOutput_CANZero::RCOutput_CANZero(uint8_t pwm_chip, uint8_t pwm_channel_base, uint8_t pwm_ch_count, uint8_t can_ch_count)
	{
		// Initialize sysfs to use PWM.
		//sysfs_out = new RCOutput_Sysfs(pwm_chip, pwm_channel_base, pwm_ch_count);
		this->pwm_channel_count = pwm_ch_count;
		this->can_channel_count = can_ch_count;
		this->ch_inf = (ChannelInfo*)calloc(pwm_ch_count+can_ch_count, sizeof(ChannelInfo));
		//this->can_addr = (struct sockaddr_can*)calloc(1, sizeof(struct sockaddr_can));
		//this->frame = (struct canfd_frame*)calloc(1, sizeof(struct canfd_frame));
	}

	RCOutput_CANZero::~RCOutput_CANZero()
	{
		if(can_socket != 0) close(can_socket);
		if(ch_inf != NULL) free(ch_inf);
		//if(can_addr != NULL) free(can_addr);
		//if(frame != NULL) free(frame);
		//sysfs_out.~RCOutput_Sysfs();
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
		printf("Connecting to socket \"%s\".\n", ifa_current->ifa_name);
		can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		can_addr_output.can_family = AF_CAN;
		can_addr_output.can_ifindex = if_nametoindex(can_ifa_name);
		bind(can_socket, (struct sockaddr *)&can_addr_output, sizeof(can_addr_output));

		std::map<uint8_t, uint8_t> ids;
		scan_devices(&ids);

		for(auto &elem : ids){
			printf("%x ", elem.first);
		}
		printf("\n");

		//sysfs_out.init();
		freeifaddrs(if_list_ptr);
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

	int RCOutput_CANZero::scan_devices(std::map<uint8_t, uint8_t> *ids)
	{
		int ret = 0;
		unsigned int addr_len = sizeof(struct sockaddr_can);
		int required_mtu = parse_canframe((char*)CAN_SYNC_MSG, &frame_output);
		::write(can_socket, &frame_output, required_mtu);

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
				printf("starting recvfrom()\n");
				::recvfrom(can_socket, &frame_input, sizeof(struct can_frame), 0, (struct sockaddr*)&can_addr_input, &addr_len);
				(*ids)[frame_input.can_id & 0x7F] = frame_input.can_id & 0x7F;
				printf("received id: %x\n", (*ids)[frame_input.can_id & 0x7F]);
			}else{
				printf("aborting scan: %f\n", dt+SCAN_TIMEOUT_MSG/100);
				ret = 1;
				break;
			}
			clock_gettime(CLOCK_MONOTONIC, &tpe);
			dt = float((float(tpe.tv_nsec)/nsps + tpe.tv_sec) - start_time);
		}

		printf("scan finished: %f\n", dt+SCAN_TIMEOUT_MSG/100);

		return ret;
	}
}
