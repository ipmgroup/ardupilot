/*
 *
 *      Author: Andrew, MAX
 */
#ifndef AP_CANOPEN_H_
#define AP_CANOPEN_H_

#if HAL_WITH_UAVCAN

#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL_Linux/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>

//#include <AP_GPS/GPS_Backend.h>
//#include <AP_Baro/AP_Baro_Backend.h>
//#include <AP_Compass/AP_Compass.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#ifndef CANOPEN_NODE_POOL_SIZE
#define CANOPEN_NODE_POOL_SIZE 8192
#endif

#ifndef CANOPEN_NODE_POOL_BLOCK_SIZE
#define CANOPEN_NODE_POOL_BLOCK_SIZE 256
#endif

#ifndef CANOPEN_SRV_NUMBER
#define CANOPEN_SRV_NUMBER 16
#endif

class AP_CANopen : public AP_HAL::CANProtocol {
public:
    AP_CANopen();
    ~AP_CANopen();

    static const struct AP_Param::GroupInfo var_info[];

    // synchronization for RC output
    bool rc_out_sem_take();
    void rc_out_sem_give();

    // output from do_cyclic
    void srv_send_actuator();
    void srv_send_esc();

    uint8_t get_node_count();

private:

    // ------------------------- RCO

    typedef union CAN_Data {
    	int64_t i64[1];
    	uint64_t ui64[1];
    	int32_t i32[2];
    	uint32_t ui32[2];
    	int16_t i16[4];
    	uint16_t ui16[4];
    	int8_t i8[8];
    	uint8_t ui8[8];
    } CAN_Data;

    struct {
    	uint8_t id;
        uint16_t pulse;
        bool servo_pending;
        bool esc_pending;
        uint16_t pulse_read; // Contains the value received from the controllers.
    } _srv_conf[CANOPEN_SRV_NUMBER];

    std::map<uint8_t, uint8_t> _discovered_nodes; // Used for discovery and reverse search. Key is node ID, value is position in _srv_conf.

    uint8_t _srv_node_cnt;

	char _thread_name[12];
    bool _initialized;
    uint8_t _srv_armed;
    uint32_t _srv_last_send_us;
    uint8_t _srv_safety;

    AP_HAL::Semaphore *SRV_sem;

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
    
    uint8_t _driver_index;

    uavcan::ISystemClock& get_system_clock();
    uavcan::ICanDriver* get_can_driver();
    uavcan::Node<0>* get_node();

    // This will be needed to implement if CANOPEN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {};

    uavcan::HeapBasedPoolAllocator<CANOPEN_NODE_POOL_BLOCK_SIZE, AP_CANopen::RaiiSynchronizer> _node_allocator;

    AP_Int8 _canopen_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int8 _ctl;
    AP_Int32 _rpm_max;
    AP_Int32 _rpmps;
    AP_Int32 _can_interval_us; // Sets the minimum amount of time between sending CAN messages.
    AP_Int32 _polling_interval_us; // Sets the minimum amount of time between rpm reads. 0 - return written value instead of attempting to read.

public:
    void loop(void);
    void init(uint8_t driver_index);
    int node_discovery(void);

    void srv_arm_actuators(bool arm);
    void SRV_push_servos();
    void srv_write(uint16_t pulse_len, uint8_t ch);
    uint16_t get_ppm(uint8_t ch);
    bool channel_enabled(uint8_t ch);


    static AP_CANopen* get_canopen(uint8_t driver_index);


};

#endif /* HAL_WITH_UAVCAN */

#endif /* AP_CANOPEN_H_ */
