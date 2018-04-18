/*
 * AP_CANopen.cpp
 *
 *      Author: Eugene Shamaev
 */

#include <unistd.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

//#if HAL_WITH_CANOPEN
#if HAL_WITH_UAVCAN

#include <typeinfo>

//#include "../AP_CANopen/AP_CANopen.h"
#include "AP_CANopen.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

// Zubax GPS and other GPS, baro, magnetic sensors
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>

extern const AP_HAL::HAL& hal;

#define debug_canopen(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

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
    AP_GROUPINFO("NOD", 1, AP_CANopen, _canopen_node, 0),

    // @Param: SRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over CANopen
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over CANopen
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Advanced
    AP_GROUPINFO("SRV_B", 2, AP_CANopen, _servo_bm, 255),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over CANopen
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over CANopen
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_B", 3, AP_CANopen, _esc_bm, 255),

    // @Param: CTL
    // @DisplayName: Controlword
    // @Description: Controlword that is sent to motor controllers to turn them on.
    // @Range: 1 63
    // @User: Advanced
	AP_GROUPINFO("CTL", 4, AP_CANopen, _ctl, 3),

    // @Param: RPM
    // @DisplayName: Full scale RPM
    // @Description: Sets the RPM value that corresponds to the maximum valid PPM value.
    // @Range: 0 15000
    // @User: Advanced
	AP_GROUPINFO("RPM_M", 5, AP_CANopen, _rpm_max, 12000),

    // @Param: RPMPS
    // @DisplayName: Acceleration of the motors.
    // @Description: Sets the rate at which the controllers will attempt to change the RPM.
    // @Range: 1 82000
    // @User: Advanced
	AP_GROUPINFO("RPMPS", 6, AP_CANopen, _rpmps, 20000),

    // @Param: CI
    // @DisplayName: CAN message interval
    // @Description: Sets the minimum amount of time between sending CAN messages in us.
    // @Range: 0 1000000
    // @User: Advanced
	AP_GROUPINFO("CI", 7, AP_CANopen, _can_interval_us, 500),

	// @Param: POL
	// @DisplayName: CAN message interval
	// @Description: Sets the minimum amount of time between rpm reads in us. 0 - return written value instead of attempting to read.
	// @Range: 0 1000000
	// @User: Advanced
	AP_GROUPINFO("POL", 8, AP_CANopen, _polling_interval_us, 20000),

    AP_GROUPEND
};

AP_CANopen::AP_CANopen() :
    _node_allocator(
        CANOPEN_NODE_POOL_SIZE, CANOPEN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < CANOPEN_RCO_NUMBER; i++) {
        _rco_conf[i].active = false;
    }

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
        _gps_nodes[i] = UINT8_MAX;
        _gps_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
        _baro_nodes[i] = UINT8_MAX;
        _baro_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
        _mag_nodes[i] = UINT8_MAX;
        _mag_node_taken[i] = 0;
        _mag_node_max_sensorid_count[i] = 1;
    }

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        _gps_listener_to_node[i] = UINT8_MAX;
        _gps_listeners[i] = nullptr;

        _baro_listener_to_node[i] = UINT8_MAX;
        _baro_listeners[i] = nullptr;

        _mag_listener_to_node[i] = UINT8_MAX;
        _mag_listeners[i] = nullptr;
        _mag_listener_sensor_ids[i] = 0;
    }

    _last_sync = _last_sync.fromUSec(0);

    _rc_out_sem = hal.util->new_semaphore();

    debug_canopen(2, "AP_CANopen constructed\n\r");
}

AP_CANopen::~AP_CANopen()
{
}

bool AP_CANopen::try_init(void)
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() && !_initialized) {

            _canopen_i = UINT8_MAX;
            for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
                if (_parent_can_mgr == hal.can_mgr[i]) {
                    _canopen_i = i;
                    break;
                }
            }

            if(_canopen_i == UINT8_MAX) {
            	_initialized = false;
            }
        }

        _initialized = true;
    }

    return _initialized;
}

int AP_CANopen::node_discovery(){
	uint8_t data[8] = {0x00};
	send_raw_packet(0x80, data, 1);
    hal.scheduler->delay(100);

    uavcan::MonotonicTime te = SystemClock::instance().getMonotonic() + uavcan::MonotonicDuration::fromMSec(100); // Time when the scan should end.

    uavcan::CanFrame frm;
    while(recv_raw_packet(frm) && te > SystemClock::instance().getMonotonic()){
    	_discovered_nodes[frm.id & 0x7F] = frm.id & 0x7F;
    }

    int i = 0;
    for(std::map<uint8_t, uint8_t>::iterator it = _discovered_nodes.begin(); it != _discovered_nodes.end() && i < CANOPEN_RCO_NUMBER; it++){
    	_rco_conf[i].id = it->first;
    	it->second = i;
    	i++;
    }

    _rco_node_cnt = i;

    printf("Nodes discovered: %d\n", _rco_node_cnt);

	return i;
}

uint8_t AP_CANopen::get_node_count(){
	return _rco_node_cnt;
}

bool AP_CANopen::rc_out_sem_take()
{
    bool sem_ret = _rc_out_sem->take(10);
    if (!sem_ret) {
        debug_canopen(1, "AP_CANopen RCOut semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_CANopen::rc_out_sem_give()
{
    _rc_out_sem->give();
}

void AP_CANopen::rc_out_send_servos(void)
{
	//printf("rc_out_send_servos()\n");
	for(int i = 0; i < _rco_node_cnt; i++){
		if(_rco_conf[i].active && (((uint32_t)1) << i) & _servo_bm){
			int32_t rpm = ppm_to_rpm(_rco_conf[i].pulse);
			int32_t data[2] = {_rpmps, rpm};
			_rco_conf[i].active = false;
			uint32_t id = 0x300 | _rco_conf[i].id;
			//printf("id: %X\n", id);
			send_raw_packet(id, (uint8_t*)data, 8);
			hal.scheduler->delay_microseconds(500);
		}
	}
}

void AP_CANopen::rc_out_send_esc(void)
{
    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = 0; i < CANOPEN_RCO_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
            max_esc_num = i + 1;
            if (_rco_conf[i].active) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
    	k = 0;

    	for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
    		if ((((uint32_t) 1) << i) & _esc_bm && i < _rco_node_cnt) {
    			int32_t rpm = ppm_to_rpm(_rco_conf[i].pulse);
    			int32_t data[2] = {_rpmps, rpm};
    			_rco_conf[i].active = false;
    			uint32_t id = 0x300 | _rco_conf[i].id;
    			//printf("id: %X\n", _rco_conf[i].id);
    			send_raw_packet(id, (uint8_t*)data, 8);
    		}
    	}
    }
}

void AP_CANopen::do_cyclic(void)
{
    if (!_initialized) {
        hal.scheduler->delay_microseconds(1000);
        return;
    }

	//auto *node = get_node();

	//const int error = node->spin(uavcan::MonotonicDuration::fromMSec(1));

//	if (error < 0) {
//		printf("error: %d\n", error);
//		hal.scheduler->delay_microseconds(1000);
//		return;
//	}

	if (rc_out_sem_take()) {
		//printf("_rco_armed: %d\n", hal.util->get_soft_armed());

		// If actuators aren't armed yet and some motor is expected to move, enable them.
		// If actuators are armed, but we don't want to move, disable them.
		if(hal.util->get_soft_armed() && !_rco_armed){
			rco_arm_actuators(true);
		}else if (!hal.util->get_soft_armed() && _rco_armed){
			rco_arm_actuators(false);
		}

		if (_rco_armed) {
			// if we have any Servos in bitmask
			if (_servo_bm > 0) {
				rc_out_send_servos();
			}

			// if we have any ESC's in bitmask
			if (_esc_bm > 0) {
				rc_out_send_esc();
			}

			recv_ppm();
		}

		for (uint8_t i = 0; i < CANOPEN_RCO_NUMBER; i++) {
			// mark as transmitted
			_rco_conf[i].active = false;
		}

		rc_out_sem_give();
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
    get_can_driver()->getIface(0)->send(frm, ddt, 0);
}

int AP_CANopen::recv_raw_packet(uavcan::CanFrame& recv_frame)
{
	int recv = 0;

    uavcan::MonotonicTime ddt = SystemClock::instance().getMonotonic();
    uavcan::UtcTime utc;
    uavcan::CanIOFlags flags = 0;
    recv = get_can_driver()->getIface(0)->receive(recv_frame, ddt, utc, flags);
//    if (recv) {
//    	printf("received %03X: ", frm.id);
//    	for(int i = 0; i < frm.dlc; i++){
//    		printf("%02X ", frm.data[i]);
//    	}
//    	printf("\n");
//    }

    return recv;
}

int AP_CANopen::recv_ppm(){
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
		int recv = recv_raw_packet(recv_frame);
		while(recv > 0 && (recv_frame.id & 0xFF80) == 0x280){
			int32_t *pdo = (int32_t*)recv_frame.data;
			//printf("received: %d\n", rpm_to_ppm(pdo[0]));

			//printf("%03X:", recv_frame.id);
			//for(int i = 0; i < 8; i++) printf(" %02X", recv_frame.data[i]);
			//printf("\n");

			_rco_conf[_discovered_nodes[recv_frame.id&0x7F]].pulse_read = rpm_to_ppm(pdo[0]);

			recv = recv_raw_packet(recv_frame);
		}
		//printf("motor 1 ppm: %d\n", _rco_conf[0].pulse_read);
		return recv;
	}else{
		for(int i = 0; i < _rco_node_cnt; i++){
			_rco_conf[i].pulse_read = _rco_conf[i].pulse;
		}
		return 0;
	}
}

uint16_t AP_CANopen::get_ppm(uint8_t ch){
	uint16_t ppm = 0;

	rc_out_sem_take();
	ppm = _rco_conf[ch].pulse_read;
	rc_out_sem_give();

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

	// Ignore negative rpm when using copter.
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
	if(rpm < 0){
		rpm = 0;
	}
#endif

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
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return _parent_can_mgr->get_driver();
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

void AP_CANopen::rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < CANOPEN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].safety_pulse = pulse_len;
        }
    }
}

void AP_CANopen::rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < CANOPEN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].failsafe_pulse = pulse_len;
        }
    }
}

void AP_CANopen::rco_force_safety_on(void)
{
    _rco_safety = true;
}

void AP_CANopen::rco_force_safety_off(void)
{
    _rco_safety = false;
}

void AP_CANopen::rco_arm_actuators(bool arm)
{
	uint16_t pdo1 = arm?_ctl:0;
	int32_t pdo2[2] = {_rpmps, 0};
	//printf("%s %d actuators\n", arm?"arming":"disarming", _rco_node_cnt);
	for(int i = 0; i < _rco_node_cnt; i++){
		//printf("%s node %d\n", arm?"Activating":"Deactivating", _rco_conf[i].id);
		send_raw_packet(0x200 | _rco_conf[i].id, (uint8_t*)&pdo1, 2);
		send_raw_packet(0x300 | _rco_conf[i].id, (uint8_t*)&pdo2, 8);
		//hal.scheduler->delay(1);
		usleep(1000);
	}
	if(_rco_node_cnt > 0){
		_rco_armed = arm;
	}
}

void AP_CANopen::rco_write(uint16_t pulse_len, uint8_t ch)
{
    _rco_conf[ch].pulse = pulse_len;
    _rco_conf[ch].active = true;
}

uint8_t AP_CANopen::find_gps_without_listener(void)
{
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr && _gps_nodes[i] != UINT8_MAX) {
            return _gps_nodes[i];
        }
    }

    return UINT8_MAX;
}

uint8_t AP_CANopen::register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel <= AP_CANOPEN_MAX_GPS_NODES) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = preferred_channel - 1;
                _gps_node_taken[_gps_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_canopen(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
                if (_gps_node_taken[i] == 0) {
                    _gps_listeners[sel_place] = new_listener;
                    _gps_listener_to_node[sel_place] = i;
                    _gps_node_taken[i]++;
                    ret = i + 1;

                    debug_canopen(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_CANopen::register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
            if (_gps_nodes[i] == node) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = i;
                _gps_node_taken[i]++;
                ret = i + 1;

                debug_canopen(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

void AP_CANopen::remove_gps_listener(AP_GPS_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == rem_listener) {
            _gps_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_gps_node_taken[_gps_listener_to_node[i]] > 0) {
                _gps_node_taken[_gps_listener_to_node[i]]--;
            }
            _gps_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_GPS::GPS_State *AP_CANopen::find_gps_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            return &_gps_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == UINT8_MAX) {
            _gps_nodes[i] = node;
            return &_gps_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_CANopen::update_gps_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_CANOPEN_MAX_LISTENERS; j++) {
                if (_gps_listener_to_node[j] == i) {
                    _gps_listeners[j]->handle_gnss_msg(_gps_node_state[i]);
                }
            }
        }
    }
}

uint8_t AP_CANopen::register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_CANOPEN_MAX_BARO_NODES) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = preferred_channel - 1;
                _baro_node_taken[_baro_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_canopen(2, "reg_Baro place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
                if (_baro_node_taken[i] == 0) {
                    _baro_listeners[sel_place] = new_listener;
                    _baro_listener_to_node[sel_place] = i;
                    _baro_node_taken[i]++;
                    ret = i + 1;

                    debug_canopen(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_CANopen::register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
            if (_baro_nodes[i] == node) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = i;
                _baro_node_taken[i]++;
                ret = i + 1;

                debug_canopen(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}


void AP_CANopen::remove_baro_listener(AP_Baro_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == rem_listener) {
            _baro_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_baro_node_taken[_baro_listener_to_node[i]] > 0) {
                _baro_node_taken[_baro_listener_to_node[i]]--;
            }
            _baro_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_CANopen::Baro_Info *AP_CANopen::find_baro_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            return &_baro_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == UINT8_MAX) {

            _baro_nodes[i] = node;
            return &_baro_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_CANopen::update_baro_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_CANOPEN_MAX_LISTENERS; j++) {
                if (_baro_listener_to_node[j] == i) {
                    _baro_listeners[j]->handle_baro_msg(_baro_node_state[i].pressure, _baro_node_state[i].temperature);
                }
            }
        }
    }
}

/*
 * Find discovered not taken baro node with smallest node ID
 */
uint8_t AP_CANopen::find_smallest_free_baro_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_BARO_NODES; i++) {
        if (_baro_node_taken[i] == 0) {
            ret = MIN(ret, _baro_nodes[i]);
        }
    }

    return ret;
}

uint8_t AP_CANopen::register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_CANOPEN_MAX_MAG_NODES) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = preferred_channel - 1;
                _mag_node_taken[_mag_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_canopen(2, "reg_Compass place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
                if (_mag_node_taken[i] == 0) {
                    _mag_listeners[sel_place] = new_listener;
                    _mag_listener_to_node[sel_place] = i;
                    _mag_node_taken[i]++;
                    ret = i + 1;

                    debug_canopen(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_CANopen::register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
            if (_mag_nodes[i] == node) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = i;
                _mag_listener_sensor_ids[sel_place] = 0;
                _mag_node_taken[i]++;
                ret = i + 1;

                debug_canopen(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

void AP_CANopen::remove_mag_listener(AP_Compass_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == rem_listener) {
            _mag_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_mag_node_taken[_mag_listener_to_node[i]] > 0) {
                _mag_node_taken[_mag_listener_to_node[i]]--;
            }
            _mag_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_CANopen::Mag_Info *AP_CANopen::find_mag_node(uint8_t node, uint8_t sensor_id)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            if (_mag_node_max_sensorid_count[i] < sensor_id) {
                _mag_node_max_sensorid_count[i] = sensor_id;
                debug_canopen(2, "AP_CANopen: Compass: found sensor id %d on node %d\n\r", (int)(sensor_id), (int)(node));
            }
            return &_mag_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == UINT8_MAX) {
            _mag_nodes[i] = node;
            _mag_node_max_sensorid_count[i] = (sensor_id ? sensor_id : 1);
            debug_canopen(2, "AP_CANopen: Compass: register sensor id %d on node %d\n\r", (int)(sensor_id), (int)(node));
            return &_mag_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

/*
 * Find discovered mag node with smallest node ID and which is taken N times,
 * where N is less than its maximum sensor id.
 * This allows multiple AP_Compass_CANopen instanses listening multiple compasses
 * that are on one node.
 */
uint8_t AP_CANopen::find_smallest_free_mag_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
        if (_mag_node_taken[i] < _mag_node_max_sensorid_count[i]) {
            ret = MIN(ret, _mag_nodes[i]);
        }
    }

    return ret;
}

void AP_CANopen::update_mag_state(uint8_t node, uint8_t sensor_id)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_CANOPEN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_CANOPEN_MAX_LISTENERS; j++) {
                if (_mag_listener_to_node[j] == i) {

                    /*If the current listener has default sensor_id,
                      while our sensor_id is not default, we have
                      to assign our sensor_id to this listener*/
                    if ((_mag_listener_sensor_ids[j] == 0) && (sensor_id != 0)) {
                        bool already_taken = false;
                        for (uint8_t k = 0; k < AP_CANOPEN_MAX_LISTENERS; k++) {
                            if (_mag_listener_sensor_ids[k] == sensor_id) {
                                already_taken = true;
                            }
                        }
                        if (!already_taken) {
                            debug_canopen(2, "AP_CANopen: Compass: sensor_id updated to %d for listener %d\n", sensor_id, j);
                            _mag_listener_sensor_ids[j] = sensor_id;
                        }
                    }

                    /*If the current listener has the sensor_id that we have,
                      or our sensor_id is default, ask the listener to handle the measurements
                      (the default one is used for the nodes that have only one compass*/
                    if ((sensor_id == 0) || (_mag_listener_sensor_ids[j] == sensor_id)) {
                        _mag_listeners[j]->handle_mag_msg(_mag_node_state[i].mag_vector);
                    }
                }
            }
        }
    }
}

AP_CANopen* AP_CANopen::get_CANopen(AP_HAL::CANManager *mgr)
{
    CANProtocol *p = mgr->get_CANProtocol();
//    if (typeid(*p) == typeid(AP_CANopen*)){
//        return dynamic_cast<AP_CANopen*>(p);
//    }
//    return nullptr;
    return (AP_CANopen*)p;
}

#endif // HAL_WITH_CANOPEN
