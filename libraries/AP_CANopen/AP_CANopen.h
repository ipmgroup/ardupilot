/*
 *
 *      Author: Andrew, MAX
 */
#ifndef AP_CANOPEN_H_
#define AP_CANOPEN_H_

#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL_Linux/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>

#include <AP_GPS/GPS_Backend.h>
#include <AP_Baro/AP_Baro_Backend.h>
#include <AP_Compass/AP_Compass.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#ifndef CANOPEN_NODE_POOL_SIZE
#define CANOPEN_NODE_POOL_SIZE 8192
#endif

#ifndef CANOPEN_NODE_POOL_BLOCK_SIZE
#define CANOPEN_NODE_POOL_BLOCK_SIZE 256
#endif

#ifndef CANOPEN_RCO_NUMBER
#define CANOPEN_RCO_NUMBER 18
#endif

#define AP_CANOPEN_MAX_LISTENERS 4
#define AP_CANOPEN_MAX_GPS_NODES 4
#define AP_CANOPEN_MAX_MAG_NODES 4
#define AP_CANOPEN_MAX_BARO_NODES 4

#define AP_CANOPEN_SW_VERS_MAJOR 1
#define AP_CANOPEN_SW_VERS_MINOR 0

#define AP_CANOPEN_HW_VERS_MAJOR 1
#define AP_CANOPEN_HW_VERS_MINOR 0

class AP_CANopen: public AP_HAL::CANProtocol {
public:
    AP_CANopen();
    ~AP_CANopen();

    static const struct AP_Param::GroupInfo var_info[];

    // this function will register the listening class on a first free channel or on the specified channel
    // if preferred_channel = 0 then free channel will be searched for
    // if preferred_channel > 0 then listener will be added to specific channel
    // return value is the number of assigned channel or 0 if fault
    // channel numbering starts from 1
    uint8_t register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel);

    uint8_t register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node);

    uint8_t find_gps_without_listener(void);

    // Removes specified listener from all nodes
    void remove_gps_listener(AP_GPS_Backend* rem_listener);

    // Returns pointer to GPS state connected with specified node.
    // If node is not found and there are free space, locate a new one
    AP_GPS::GPS_State *find_gps_node(uint8_t node);

    // Updates all listeners of specified node
    void update_gps_state(uint8_t node);

    struct Baro_Info {
        float pressure;
        float pressure_variance;
        float temperature;
        float temperature_variance;
    };

    uint8_t register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel);
    uint8_t register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node);
    void remove_baro_listener(AP_Baro_Backend* rem_listener);
    Baro_Info *find_baro_node(uint8_t node);
    uint8_t find_smallest_free_baro_node();
    void update_baro_state(uint8_t node);

    struct Mag_Info {
        Vector3f mag_vector;
    };

    uint8_t register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel);
    void remove_mag_listener(AP_Compass_Backend* rem_listener);
    Mag_Info *find_mag_node(uint8_t node, uint8_t sensor_id);
    uint8_t find_smallest_free_mag_node();
    uint8_t register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node);
    void update_mag_state(uint8_t node, uint8_t sensor_id);

    // synchronization for RC output
    bool rc_out_sem_take();
    void rc_out_sem_give();

    // output from do_cyclic
    void rc_out_send_servos();
    void rc_out_send_esc();

    uint8_t get_node_count();

private:
    // ------------------------- GPS
    // 255 - means free node
    uint8_t _gps_nodes[AP_CANOPEN_MAX_GPS_NODES];
    // Counter of how many listeners are connected to this source
    uint8_t _gps_node_taken[AP_CANOPEN_MAX_GPS_NODES];
    // GPS data of the sources
    AP_GPS::GPS_State _gps_node_state[AP_CANOPEN_MAX_GPS_NODES];

    // 255 - means no connection
    uint8_t _gps_listener_to_node[AP_CANOPEN_MAX_LISTENERS];
    // Listeners with callbacks to be updated
    AP_GPS_Backend* _gps_listeners[AP_CANOPEN_MAX_LISTENERS];

    // ------------------------- BARO
    uint8_t _baro_nodes[AP_CANOPEN_MAX_BARO_NODES];
    uint8_t _baro_node_taken[AP_CANOPEN_MAX_BARO_NODES];
    Baro_Info _baro_node_state[AP_CANOPEN_MAX_BARO_NODES];
    uint8_t _baro_listener_to_node[AP_CANOPEN_MAX_LISTENERS];
    AP_Baro_Backend* _baro_listeners[AP_CANOPEN_MAX_LISTENERS];

    // ------------------------- MAG
    uint8_t _mag_nodes[AP_CANOPEN_MAX_MAG_NODES];
    uint8_t _mag_node_taken[AP_CANOPEN_MAX_MAG_NODES];
    Mag_Info _mag_node_state[AP_CANOPEN_MAX_MAG_NODES];
    uint8_t _mag_node_max_sensorid_count[AP_CANOPEN_MAX_MAG_NODES];
    uint8_t _mag_listener_to_node[AP_CANOPEN_MAX_LISTENERS];
    AP_Compass_Backend* _mag_listeners[AP_CANOPEN_MAX_LISTENERS];
    uint8_t _mag_listener_sensor_ids[AP_CANOPEN_MAX_LISTENERS];

    // ------------------------- RCO

    struct {
    	uint8_t id;
        uint16_t pulse;
        uint16_t safety_pulse;
        uint16_t failsafe_pulse;
        bool active;
        uint16_t pulse_read; // Contains the value received from the controllers.
    } _rco_conf[CANOPEN_RCO_NUMBER];

    std::map<uint8_t, uint8_t> _discovered_nodes; // Used for discovery and reverse search. Key is node ID, value is position in _rco_conf.

    uint8_t _rco_node_cnt;

    bool _initialized;
    uint8_t _rco_armed;
    uint8_t _rco_safety;

    AP_HAL::Semaphore *_rc_out_sem;

    uavcan::MonotonicTime _last_sync;

    void send_raw_packet(uint32_t id, uint8_t* data, uint8_t len);
    int recv_raw_packet(uavcan::CanFrame& recv_frame);
    int recv_ppm();
    int32_t ppm_to_rpm(int ppm);
    uint16_t rpm_to_ppm(int32_t);

    //template<typename T> void generate_frame(can_frame *frame, uint16_t base_id, uint16_t node_id, uint32_t meta, T value, uint8_t ignore_meta = 0);

    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
        SystemClock()
        {
        }

        uavcan::UtcDuration utc_adjustment;
        virtual void adjustUtc(uavcan::UtcDuration adjustment)
        {
            utc_adjustment = adjustment;
        }

    public:
        virtual uavcan::MonotonicTime getMonotonic() const
        {
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            return uavcan::MonotonicTime::fromUSec(usec);
        }
        virtual uavcan::UtcTime getUtc() const
        {
            uavcan::UtcTime utc;
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            utc.fromUSec(usec);
            utc += utc_adjustment;
            return utc;
        }

        static SystemClock& instance()
        {
            static SystemClock inst;
            return inst;
        }
    };

    uavcan::Node<0> *_node = nullptr;

    uavcan::ISystemClock& get_system_clock();
    uavcan::ICanDriver* get_can_driver();
    uavcan::Node<0>* get_node();

    // This will be needed to implement if CANOPEN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {
    public:
        RaiiSynchronizer()
        {
        }

        ~RaiiSynchronizer()
        {
        }
    };

    uavcan::HeapBasedPoolAllocator<CANOPEN_NODE_POOL_BLOCK_SIZE, AP_CANopen::RaiiSynchronizer> _node_allocator;

    AP_Int8 _canopen_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int8 _ctl;
    AP_Int32 _rpm_max;
    AP_Int32 _rpmps;
    AP_Int32 _can_interval_us; // Sets the minimum amount of time between sending CAN messages.
    AP_Int32 _polling_interval_us; // Sets the minimum amount of time between rpm reads. 0 - return written value instead of attempting to read.

    uint8_t _canopen_i;

public:
    void do_cyclic(void);
    bool try_init(void);
    int node_discovery(void);

    void rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len);
    void rco_force_safety_on(void);
    void rco_force_safety_off(void);
    void rco_arm_actuators(bool arm);
    void rco_write(uint16_t pulse_len, uint8_t ch);
    uint16_t get_ppm(uint8_t ch);
    bool channel_enabled(uint8_t ch);


    static AP_CANopen* get_CANopen(AP_HAL::CANManager *mgr);


};

#endif /* AP_CANOPEN_H_ */
