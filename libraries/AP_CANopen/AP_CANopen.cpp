/*
 * AP_CANopen.cpp
 *
 *      Author: Dmitri Ranfft
 */

#include <unistd.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#ifdef HAL_WITH_UAVCAN

#include <typeinfo>

//#include "../AP_CANopen/AP_CANopen.h"
#include "AP_CANopen.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// Translation of all messages from CANopen structures into AP structures is done
// in AP_CANopen and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// TODO: temperature can come not only from baro. There should be separation on node ID
// to check where it belongs to. If it is not baro that is the source, separate layer
// of listeners/nodes should be added.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_CANopen::var_info[] = {
    // @Param: NODE
    // @DisplayName: CANopen node that is used for this network
    // @Description: CANopen node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_CANopen, _canopen_node, 0),

    // @Param: SRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over CANopen
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over CANopen
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_CANopen, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over CANopen
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over CANopen
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_CANopen, _esc_bm, 15),

    // @Param: CTL_W
    // @DisplayName: Controlword
    // @Description: Controlword that is sent to motor controllers to turn them on.
    // @Range: 1 63
    // @User: Advanced
	AP_GROUPINFO("CTL_W", 4, AP_CANopen, _ctl, 3),

    // @Param: MAXRPM
    // @DisplayName: Full scale RPM
    // @Description: Sets the RPM value that corresponds to the maximum valid PPM value.
    // @Range: 0 15000
    // @User: Advanced
	AP_GROUPINFO("MAXRPM", 5, AP_CANopen, _rpm_max, 12000),

    // @Param: MOT_AC
    // @DisplayName: Acceleration of the motors.
    // @Description: Sets the rate at which the controllers will attempt to change the RPM.
    // @Range: 1 82000
    // @User: Advanced
	AP_GROUPINFO("MOT_AC", 6, AP_CANopen, _rpmps, 20000),

    // @Param: W_POL
    // @DisplayName: CAN message interval
    // @Description: Sets the minimum amount of time between sending CAN messages in microseconds.
    // @Range: 0 1000000
    // @User: Advanced
	AP_GROUPINFO("W_POL", 7, AP_CANopen, _can_interval_us, 500),

	// @Param: R_POL
	// @DisplayName: CAN message interval
	// @Description: Sets the minimum amount of time between rpm reads in microseconds. 0 - return written value instead of attempting to read.
	// @Range: 0 1000000
	// @User: Advanced
	AP_GROUPINFO("R_POL", 8, AP_CANopen, _polling_interval_us, 20000),

    AP_GROUPEND
};

AP_CANopen::AP_CANopen() :
    _node_allocator(
        CANOPEN_NODE_POOL_SIZE, CANOPEN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < CANOPEN_SRV_NUMBER; i++) {
        _srv_conf[i].servo_pending = false;
        _srv_conf[i].esc_pending = false;
    }

    _last_sync = _last_sync.fromUSec(0);

    SRV_sem = hal.util->new_semaphore();
    _telem_sem = hal.util->new_semaphore();

    //debug_can(2, "AP_CANopen constructed\n\r");
}

AP_CANopen::~AP_CANopen()
{
}

void AP_CANopen::init(uint8_t driver_index)
{
	_driver_index = driver_index;
	
    if (hal.can_mgr[_driver_index] != nullptr && hal.can_mgr[_driver_index]->is_initialized() && hal.can_mgr[_driver_index]->get_driver() != nullptr) {
		node_discovery();
		snprintf(_thread_name, sizeof(_thread_name), "canopen_%u", driver_index);
		if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANopen::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
	        _initialized = true;
	    } else {
	    	_initialized = false;
	    }
	} else {
    	_initialized = false;
    }
}

int AP_CANopen::node_discovery(){

	// NMT reset.
	uint8_t data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	send_raw_packet(0x00, data, 2);

	// Wait for nodes to respond.
    hal.scheduler->delay(300);

    uavcan::MonotonicTime te = SystemClock::instance().getMonotonic() + uavcan::MonotonicDuration::fromMSec(100); // Time when the scan should end.

    uavcan::CanFrame frm;
    while(recv_raw_packet(frm) && te > SystemClock::instance().getMonotonic()){
    	_discovered_nodes[frm.id & 0x7F] = frm.id & 0x7F;
    }

    int i = 0;
    for(std::map<uint8_t, uint8_t>::iterator it = _discovered_nodes.begin(); it != _discovered_nodes.end() && i < CANOPEN_SRV_NUMBER; it++){
    	_srv_conf[i].id = it->first;
    	it->second = i;
    	i++;
    }

    _srv_node_cnt = i;

    // Set mode to operational. Required if nodes do not start up in operational mode.
    data[0] = 0x01;
    send_raw_packet(0x00, data, 2);

    printf("Nodes discovered: %d\n", _srv_node_cnt);

	return i;
}

uint8_t AP_CANopen::get_node_count(){
	return _srv_node_cnt;
}

bool AP_CANopen::srv_sem_take()
{
    bool sem_ret = SRV_sem->take(10);
//     if (!sem_ret) {
//         debug_canopen(1, "AP_CANopen SRV semaphore fail\n\r");
//     }
    return sem_ret;
}

void AP_CANopen::srv_sem_give()
{
    SRV_sem->give();
}

void AP_CANopen::srv_send_actuator(void)
{
	//printf("srv_send_actuator()\n");
	for(int i = 0; i < _srv_node_cnt; i++){
		if(_srv_conf[i].servo_pending && (((uint32_t)1) << i) & _servo_bm){
			CAN_Data data;

			int32_t rpm = ppm_to_rpm(_srv_conf[i].pulse);
			data.i32[0] = _rpmps;
			data.i32[1] = rpm;
			_srv_conf[i].servo_pending = false;
			uint32_t id = 0x300 | _srv_conf[i].id;
			hal.scheduler->delay_microseconds(1);
			send_raw_packet(id, data.ui8, 8);
			hal.scheduler->delay_microseconds(500);
		}
	}
}

void AP_CANopen::srv_send_esc(void)
{
    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = 0; i < CANOPEN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
            max_esc_num = i + 1;
            if (_srv_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
    	k = 0;

    	for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
    		if ((((uint32_t) 1) << i) & _esc_bm && i < _srv_node_cnt) {
    			CAN_Data data;

    			int32_t rpm = ppm_to_rpm(_srv_conf[i].pulse);
    			data.i32[0] = _rpmps;
    			data.i32[1] = rpm;
    			_srv_conf[i].esc_pending = false;
    			uint32_t id = 0x300 | _srv_conf[i].id;
    			send_raw_packet(id, data.ui8, 8);
    		}
    	}
    }
}

void AP_CANopen::loop(void)
{
	while (true) {
	    if (!_initialized) {
	        hal.scheduler->delay_microseconds(1000);
	        return;
	    }
	
		// If actuators aren't armed yet and some motor is expected to move, enable them.
		// If actuators are armed, but we don't want to move, disable them.
		if(hal.util->get_soft_armed() && !_srv_armed){
			srv_arm_actuators(true);
		}else if (!hal.util->get_soft_armed() && _srv_armed){
			srv_arm_actuators(false);
		}

		if (_srv_armed) {
			bool sent_servos = false;
			
			// if we have any Servos in bitmask
			if (_servo_bm > 0) {
				uint32_t now = AP_HAL::micros();
				if (now - _srv_last_send_us >= (uint32_t)_can_interval_us) {
					_srv_last_send_us = now;
					srv_send_actuator();
					sent_servos = true;
				}
			}

			// if we have any ESC's in bitmask
			if (_esc_bm > 0 && !sent_servos) {
				srv_send_esc();
			}

			recv_telem();
		}
	}
}

void AP_CANopen::send_raw_packet(uint32_t id, uint8_t* data, uint8_t len)
{
    if (len > 8) {
        len = 8;
    }

    uavcan::CanFrame frm((id), data, len); //& uavcan::CanFrame::MaskStdID
    uavcan::MonotonicDuration dur;
    dur = dur.fromMSec(100);
    uavcan::MonotonicTime ddt = SystemClock::instance().getMonotonic() + dur;
    //printf("%ul %ul\n", dur.toMSec(), ddt.toMSec());
    get_can_driver()->getIface(_driver_index)->send(frm, ddt, 0);
}

int AP_CANopen::recv_raw_packet(uavcan::CanFrame& recv_frame)
{
	int recv = 0;

    uavcan::MonotonicTime ddt = SystemClock::instance().getMonotonic();
    uavcan::UtcTime utc;
    uavcan::CanIOFlags flags = 0;
    recv = get_can_driver()->getIface(_driver_index)->receive(recv_frame, ddt, utc, flags);
//    if (recv) {
//    	printf("received %03X: ", frm.id);
//    	for(int i = 0; i < frm.dlc; i++){
//    		printf("%02X ", frm.data[i]);
//    	}
//    	printf("\n");
//    }

    return recv;
}

void AP_CANopen::recv_telem(){
	uint8_t data[8] = {0x00};

	uavcan::MonotonicTime now = SystemClock::instance().getMonotonic();
	uavcan::MonotonicDuration dt_min;
	dt_min = dt_min.fromUSec(_polling_interval_us);
	uavcan::MonotonicTime lockout_end = _last_sync + dt_min;

	if(_polling_interval_us > 0){
		if(now > lockout_end){
			send_raw_packet(0x80, data, 1);
			_last_sync = now;
		}

		uavcan::CanFrame recv_frame;
		CAN_Data pdo;
		int recv = recv_raw_packet(recv_frame);
		while(recv > 0){
			memcpy(pdo.ui8, recv_frame.data, sizeof(pdo.ui8));
			uint16_t pdoid = recv_frame.id & 0xFF80;
			uint8_t nodeid = recv_frame.id & 0x7F;
			
			if (!_telem_sem->take(1)) {
		        return;
		    }
			
			switch (pdoid) {
				case 0x180:
					// statusword
					break;
				case 0x280:
					// rpm
					_srv_conf[_discovered_nodes[nodeid]].pulse_read = rpm_to_ppm(pdo.i32[0]);
					_telemetry[_discovered_nodes[nodeid]].rpm = pdo.i32[0];
					break;
				case 0x380:
					// volatage, current, temperature
					_telemetry[_discovered_nodes[nodeid]].voltage = pdo.ui16[0];
					_telemetry[_discovered_nodes[nodeid]].current = pdo.ui16[1];
					_telemetry[_discovered_nodes[nodeid]].temp = pdo.ui16[2];
					break;
				case 0x480:
					// status registers
					break;
				default:
					break;
			}
			
			_telem_sem->give();

			recv = recv_raw_packet(recv_frame);
		}
		//printf("motor ppm: %d %d %d %d\n", _srv_conf[0].pulse_read, _srv_conf[1].pulse_read, _srv_conf[2].pulse_read, _srv_conf[3].pulse_read);
		return;
	}else{
		for(int i = 0; i < _srv_node_cnt; i++){
			_srv_conf[i].pulse_read = _srv_conf[i].pulse;
		}
		return;
	}
}

uint16_t AP_CANopen::get_ppm(uint8_t ch){
	uint16_t ppm = 0;

	srv_sem_take();
	ppm = _srv_conf[ch].pulse_read;
	srv_sem_give();

	return ppm;
}

bool AP_CANopen::channel_enabled(uint8_t ch){
	bool ch_en = (_servo_bm & (1<<ch)) || (_esc_bm & (1<<ch));
	return ch_en;
}

int32_t AP_CANopen::ppm_to_rpm(int ppm){
	int32_t rpm = 0;

	int ppm_d = ppm - 1500; // ppm of 1500 is 0 rpm
	float rpm_per_ppm = ((float)_rpm_max) / 500.0;

	rpm = rpm_per_ppm * ppm_d;

	return rpm;
}

uint16_t AP_CANopen::rpm_to_ppm(int32_t rpm){
	uint16_t ppm = 1500;

	float rpm_per_ppm = ((float)_rpm_max) / 500.0;
	float ppm_d = rpm/rpm_per_ppm;
	ppm += ppm_d;

	return ppm;
}

uavcan::ISystemClock & AP_CANopen::get_system_clock()
{
    return SystemClock::instance();
}

uavcan::ICanDriver * AP_CANopen::get_can_driver()
{
    if (hal.can_mgr[_driver_index] != nullptr) {
        if (hal.can_mgr[_driver_index]->is_initialized() == false) {
            return nullptr;
        } else {
            return hal.can_mgr[_driver_index]->get_driver();
        }
    }
    return nullptr;
}

uavcan::Node<0> *AP_CANopen::get_node()
{
    if (_node == nullptr && get_can_driver() != nullptr) {
        _node = new uavcan::Node<0>(*get_can_driver(), get_system_clock(), _node_allocator);
    }

    return _node;
}

void AP_CANopen::srv_arm_actuators(bool arm)
{
	uint16_t pdo1 = arm?_ctl:0;
	int32_t pdo2[2] = {_rpmps, 0};
	//printf("%s %d actuators\n", arm?"arming":"disarming", _srv_node_cnt);
	for(int i = 0; i < _srv_node_cnt; i++){
		//printf("%s node %d\n", arm?"Activating":"Deactivating", _srv_conf[i].id);
		send_raw_packet(0x200 | _srv_conf[i].id, (uint8_t*)&pdo1, 2);
		send_raw_packet(0x300 | _srv_conf[i].id, (uint8_t*)&pdo2, 8);
		//hal.scheduler->delay(1);
		usleep(1000);
	}
	if(_srv_node_cnt > 0){
		_srv_armed = arm;
	}
}

void AP_CANopen::srv_write(uint16_t pulse_len, uint8_t ch)
{
    _srv_conf[ch].pulse = pulse_len;
    _srv_conf[ch].servo_pending = true;
    _srv_conf[ch].esc_pending = true;
}

AP_CANopen* AP_CANopen::get_canopen(uint8_t driver_index)
{
	if (driver_index >= AP::can().get_num_drivers() || 
		AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_CANOPEN) {
		return nullptr;
	}
	return static_cast<AP_CANopen*>(AP::can().get_driver(driver_index));
}

void AP_CANopen::SRV_push_servos()
{
    SRV_sem->take_blocking();

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i)) {
            srv_write(SRV_Channels::srv_channel(i)->get_output_pwm(), i);
        }
    }

    SRV_sem->give();

    _srv_armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

void AP_CANopen::send_mavlink(uint8_t ch) {
	telemetry_info_t telem_buffer[CANOPEN_SRV_NUMBER];
	
	if (!_telem_sem->take(1)) {
        return;
    }
    
    memcpy(telem_buffer, _telemetry, sizeof(telemetry_info_t) * CANOPEN_SRV_NUMBER);
    
    _telem_sem->give();
    
    uint16_t voltage[4] {};
    uint16_t current[4] {};
    uint16_t rpm[4] {};
    uint8_t temperature[4] {};
    uint16_t totalcurrent[4] {};
    uint16_t count[4] {};
    
    for (uint8_t i = 0; i < _srv_node_cnt && i < 8; i++) {
        uint8_t idx = i % 4;
        
        voltage[idx]      = telem_buffer[i].voltage;
        current[idx]      = telem_buffer[i].current;
        rpm[idx]          = telem_buffer[i].rpm;
        temperature[idx]  = telem_buffer[i].temp;

        if (idx == 3 || i == _srv_node_cnt - 1) {
            if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)ch, ESC_TELEMETRY_1_TO_4)) {
                return;
            }

            if (i < 4) {
                mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)ch, temperature, voltage, current, totalcurrent, rpm, count);
            } else {
                mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)ch, temperature, voltage, current, totalcurrent, rpm, count);
            }
        }
    }
}

#endif // HAL_WITH_CANOPEN
