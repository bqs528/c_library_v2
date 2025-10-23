#pragma once
// MESSAGE MPPT_STATUS PACKING

#define MAVLINK_MSG_ID_MPPT_STATUS 12922


typedef struct __mavlink_mppt_status_t {
 uint64_t timestamp; /*<  */
 float input_voltage; /*<  */
 float input_current; /*<  */
 float output_voltage; /*<  */
 float output_current; /*<  */
} mavlink_mppt_status_t;

#define MAVLINK_MSG_ID_MPPT_STATUS_LEN 24
#define MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN 24
#define MAVLINK_MSG_ID_12922_LEN 24
#define MAVLINK_MSG_ID_12922_MIN_LEN 24

#define MAVLINK_MSG_ID_MPPT_STATUS_CRC 237
#define MAVLINK_MSG_ID_12922_CRC 237



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MPPT_STATUS { \
    12922, \
    "MPPT_STATUS", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mppt_status_t, timestamp) }, \
         { "input_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mppt_status_t, input_voltage) }, \
         { "input_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mppt_status_t, input_current) }, \
         { "output_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mppt_status_t, output_voltage) }, \
         { "output_current", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mppt_status_t, output_current) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MPPT_STATUS { \
    "MPPT_STATUS", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mppt_status_t, timestamp) }, \
         { "input_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mppt_status_t, input_voltage) }, \
         { "input_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mppt_status_t, input_current) }, \
         { "output_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mppt_status_t, output_voltage) }, \
         { "output_current", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mppt_status_t, output_current) }, \
         } \
}
#endif

/**
 * @brief Pack a mppt_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  
 * @param input_voltage  
 * @param input_current  
 * @param output_voltage  
 * @param output_current  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mppt_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float input_voltage, float input_current, float output_voltage, float output_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, input_voltage);
    _mav_put_float(buf, 12, input_current);
    _mav_put_float(buf, 16, output_voltage);
    _mav_put_float(buf, 20, output_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#else
    mavlink_mppt_status_t packet;
    packet.timestamp = timestamp;
    packet.input_voltage = input_voltage;
    packet.input_current = input_current;
    packet.output_voltage = output_voltage;
    packet.output_current = output_current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPPT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
}

/**
 * @brief Pack a mppt_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  
 * @param input_voltage  
 * @param input_current  
 * @param output_voltage  
 * @param output_current  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mppt_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, float input_voltage, float input_current, float output_voltage, float output_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, input_voltage);
    _mav_put_float(buf, 12, input_current);
    _mav_put_float(buf, 16, output_voltage);
    _mav_put_float(buf, 20, output_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#else
    mavlink_mppt_status_t packet;
    packet.timestamp = timestamp;
    packet.input_voltage = input_voltage;
    packet.input_current = input_current;
    packet.output_voltage = output_voltage;
    packet.output_current = output_current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPPT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a mppt_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  
 * @param input_voltage  
 * @param input_current  
 * @param output_voltage  
 * @param output_current  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mppt_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float input_voltage,float input_current,float output_voltage,float output_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, input_voltage);
    _mav_put_float(buf, 12, input_current);
    _mav_put_float(buf, 16, output_voltage);
    _mav_put_float(buf, 20, output_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#else
    mavlink_mppt_status_t packet;
    packet.timestamp = timestamp;
    packet.input_voltage = input_voltage;
    packet.input_current = input_current;
    packet.output_voltage = output_voltage;
    packet.output_current = output_current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MPPT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
}

/**
 * @brief Encode a mppt_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mppt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mppt_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mppt_status_t* mppt_status)
{
    return mavlink_msg_mppt_status_pack(system_id, component_id, msg, mppt_status->timestamp, mppt_status->input_voltage, mppt_status->input_current, mppt_status->output_voltage, mppt_status->output_current);
}

/**
 * @brief Encode a mppt_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mppt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mppt_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mppt_status_t* mppt_status)
{
    return mavlink_msg_mppt_status_pack_chan(system_id, component_id, chan, msg, mppt_status->timestamp, mppt_status->input_voltage, mppt_status->input_current, mppt_status->output_voltage, mppt_status->output_current);
}

/**
 * @brief Encode a mppt_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mppt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mppt_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mppt_status_t* mppt_status)
{
    return mavlink_msg_mppt_status_pack_status(system_id, component_id, _status, msg,  mppt_status->timestamp, mppt_status->input_voltage, mppt_status->input_current, mppt_status->output_voltage, mppt_status->output_current);
}

/**
 * @brief Send a mppt_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  
 * @param input_voltage  
 * @param input_current  
 * @param output_voltage  
 * @param output_current  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mppt_status_send(mavlink_channel_t chan, uint64_t timestamp, float input_voltage, float input_current, float output_voltage, float output_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MPPT_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, input_voltage);
    _mav_put_float(buf, 12, input_current);
    _mav_put_float(buf, 16, output_voltage);
    _mav_put_float(buf, 20, output_current);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    mavlink_mppt_status_t packet;
    packet.timestamp = timestamp;
    packet.input_voltage = input_voltage;
    packet.input_current = input_current;
    packet.output_voltage = output_voltage;
    packet.output_current = output_current;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#endif
}

/**
 * @brief Send a mppt_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mppt_status_send_struct(mavlink_channel_t chan, const mavlink_mppt_status_t* mppt_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mppt_status_send(chan, mppt_status->timestamp, mppt_status->input_voltage, mppt_status->input_current, mppt_status->output_voltage, mppt_status->output_current);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)mppt_status, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MPPT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mppt_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float input_voltage, float input_current, float output_voltage, float output_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, input_voltage);
    _mav_put_float(buf, 12, input_current);
    _mav_put_float(buf, 16, output_voltage);
    _mav_put_float(buf, 20, output_current);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, buf, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#else
    mavlink_mppt_status_t *packet = (mavlink_mppt_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->input_voltage = input_voltage;
    packet->input_current = input_current;
    packet->output_voltage = output_voltage;
    packet->output_current = output_current;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MPPT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MPPT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MPPT_STATUS_LEN, MAVLINK_MSG_ID_MPPT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MPPT_STATUS UNPACKING


/**
 * @brief Get field timestamp from mppt_status message
 *
 * @return  
 */
static inline uint64_t mavlink_msg_mppt_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field input_voltage from mppt_status message
 *
 * @return  
 */
static inline float mavlink_msg_mppt_status_get_input_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field input_current from mppt_status message
 *
 * @return  
 */
static inline float mavlink_msg_mppt_status_get_input_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field output_voltage from mppt_status message
 *
 * @return  
 */
static inline float mavlink_msg_mppt_status_get_output_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field output_current from mppt_status message
 *
 * @return  
 */
static inline float mavlink_msg_mppt_status_get_output_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a mppt_status message into a struct
 *
 * @param msg The message to decode
 * @param mppt_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mppt_status_decode(const mavlink_message_t* msg, mavlink_mppt_status_t* mppt_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mppt_status->timestamp = mavlink_msg_mppt_status_get_timestamp(msg);
    mppt_status->input_voltage = mavlink_msg_mppt_status_get_input_voltage(msg);
    mppt_status->input_current = mavlink_msg_mppt_status_get_input_current(msg);
    mppt_status->output_voltage = mavlink_msg_mppt_status_get_output_voltage(msg);
    mppt_status->output_current = mavlink_msg_mppt_status_get_output_current(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MPPT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MPPT_STATUS_LEN;
        memset(mppt_status, 0, MAVLINK_MSG_ID_MPPT_STATUS_LEN);
    memcpy(mppt_status, _MAV_PAYLOAD(msg), len);
#endif
}
