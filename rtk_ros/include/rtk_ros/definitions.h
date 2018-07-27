/**
 * @file definitions.h
 * common platform-specific definitions & abstractions based from QGroundControl
 * @author Alexis Paques <alexis.paques@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once
#include <stdint.h>
#include <ros/ros.h>
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f

#define GPS_READ_BUFFER_SIZE 1024
#define GPS_INFO(...) ROS_INFO(__VA_ARGS__)
#define GPS_WARN(...) ROS_WARN(__VA_ARGS__)
#define GPS_ERR(...) ROS_FATAL(__VA_ARGS__)

typedef uint64_t gps_abstime;
typedef uint64_t gps_absolute_time;

//timespec is UNIX-specific
#ifdef _WIN32
#if _MSC_VER < 1900
struct timespec
{
    time_t tv_sec;
    long tv_nsec;
};
#else
#include <time.h>
#endif
#else
#include <time.h>
#endif


/*
 * The following structs are auto-generated from https://github.com/PX4/Firmware/blob/master/msg/satellite_info.msg
 * and was manually copied here.
 */

struct satellite_info_s {
	uint64_t timestamp;
	uint8_t count;
	uint8_t svid[20];
	uint8_t used[20];
	uint8_t elevation[20];
	uint8_t azimuth[20];
	uint8_t snr[20];
#ifdef __cplusplus
	static const uint8_t SAT_INFO_MAX_SATELLITES = 20;
#endif
};

struct vehicle_gps_position_s {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	uint8_t fix_type;
	bool vel_ned_valid;
	uint8_t satellites_used;
};

