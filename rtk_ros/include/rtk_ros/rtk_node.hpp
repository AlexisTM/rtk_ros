/**
 * @file rtk_node.h
 * Node connecting to the RTK device and reading messages 
 * @author Alexis Paques <alexis.paques@gmail.com>
 */

#pragma once

#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <serial/serial.h>
#include <mavros_msgs/RTCM.h>
#include <rtk_ros/GpsDrivers/src/ubx.h>
#include <rtk_ros/GpsDrivers/src/ashtech.h>
#include <rtk_ros/GpsDrivers/src/gps_helper.h>
#include "definitions.h"

class RTKNode
{
public:
	RTKNode(unsigned _baud = 0, 
        std::string _port = std::string("/dev/ttyACM0"), 
        float _surveyAccuracy = 1.0,
        float _surveyDuration = 90.0): 
        connected(false), baud(_baud), port(_port),
        surveyAccuracy(_surveyAccuracy), surveyDuration(_surveyDuration) {
            surveyInStatus = new SurveyInStatus();
    };
	~RTKNode() {
        if (gpsDriver) {
            delete gpsDriver;
            gpsDriver = nullptr;
        }
        if (serial) {
            delete serial;
            serial = nullptr;
        }
        if (pReportSatInfo) {
            delete pReportSatInfo;
            pReportSatInfo = nullptr;
        }
        if (surveyInStatus) {
            delete surveyInStatus;
            surveyInStatus = nullptr;
        }
    };

    void connect() {
        if (!serial) serial = new serial::Serial();
        serial::Timeout to(serial::Timeout::simpleTimeout(500));
        serial->setTimeout(to);
        serial->setPort(port);
        serial->setBaudrate(baud);

        for (int tries = 0; tries < 5; tries++) {
            try {
                ROS_DEBUG("Trying to connect to the serial port");
                serial->open();
            } catch (serial::IOException) {
            } catch (...) {
                ROS_FATAL("Other serial port exception");
            }

            if (serial->isOpen()) {
                connected = true;
                return;
            } else {
                connected = false;
                ROS_INFO_STREAM("Bad Connection with serial port Error " << port);
            }
        }
    };

    void configure() {
        ROS_WARN("Configuration");
        if (gpsDriver->configure(baud, GPSDriverUBX::OutputMode::RTCM) == 0) {

            /* reset report */
            memset(&reportGPSPos, 0, sizeof(reportGPSPos));

            //In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
            //bus errors or buggy firmware. In this case we want to try multiple times before giving up.
            int numTries = 0;

            while (ros::ok() && numTries < 3) {
                int helperRet = gpsDriver->receive(500);
                ROS_WARN("Reading data");

                if (helperRet > 0) {
                    numTries = 0;

                    if (helperRet & 1) {
                        publishGPSPosition();
                        ROS_WARN("Got GPS position");
                        numTries = 0;
                    }

                    if (pReportSatInfo && (helperRet & 2)) {
                        publishGPSSatellite();
                        ROS_WARN("Got GPS Satellites");
                        numTries = 0;
                    }
                } else {
                    ++numTries;
                }
            }
        }
    };

    void publishGPSPosition() {
        
        ROS_WARN("Publish pose");
    };

    void publishGPSSatellite() {
        
        ROS_WARN("Publish sat");
    };

    void connect_gps() {
        // dynamic model
        uint8_t stationary_model = 2;
        ROS_WARN("Connect Driver");
        gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &reportGPSPos, pReportSatInfo, stationary_model);
        gpsDriver->setSurveyInSpecs(surveyAccuracy * 10000, surveyDuration);
        ROS_WARN("Configure survey");
        memset(&reportGPSPos, 0, sizeof(reportGPSPos)); // Reset report
    };


    static int callbackEntry(GPSCallbackType type, void *data1, int data2, void *user)
    {   
        ROS_WARN("Got data");
        RTKNode *node = (RTKNode *)user;
        return node->callback(type, data1, data2);
    };


    void gotRTCMData(uint8_t *data, size_t len) {
        
        ROS_WARN("Publish RTCM");
    }

    int callback(GPSCallbackType type, void *data1, int data2)
    {
        switch (type) {
            case GPSCallbackType::readDeviceData: {
                ROS_WARN("Read device data");
                if (serial->available()) {
                    uint32_t timeout = 1;
                    serial->waitByteTimes(timeout);
                    if (!serial->available())
                        return 0; //timeout
                }
                return (int)serial->read((uint8_t*) data1, data2);
            }
            case GPSCallbackType::writeDeviceData:
                ROS_WARN("Write device data");
                if (serial->write((uint8_t*) data1, data2) >= 0) {
                    if (serial->available())
                        return data2;
                }
                return -1;

            case GPSCallbackType::setBaudrate:
                ROS_WARN("Set baudrate");
                serial->setBaudrate((uint32_t) data2);
                return 0;

            case GPSCallbackType::gotRTCMMessage:
                ROS_WARN("RTCM");
                gotRTCMData((uint8_t*) data1, data2);
                break;

            case GPSCallbackType::surveyInStatus:
            {
                ROS_WARN("Survey");
                surveyInStatus = (SurveyInStatus*)data1;
                ROS_DEBUG_STREAM("Survey-in status: " << surveyInStatus->duration  << " cur accuracy: " << surveyInStatus->mean_accuracy 
                        << " valid:" << (int)(surveyInStatus->flags & 1) << " active: " << (int)((surveyInStatus->flags>>1) & 1));
                break;
            }

            case GPSCallbackType::setClock:
                /* do nothing */
                break;

            default:
                /* do nothing */
                break;
        }

        // return 0;
    };


    bool connected;
private:
    unsigned baud;
    std::string port;
    float surveyAccuracy;
    float surveyDuration;
    SurveyInStatus* surveyInStatus = nullptr;
    GPSHelper* gpsDriver = nullptr;
    serial::Serial* serial = nullptr;
	struct vehicle_gps_position_s	reportGPSPos;
	struct satellite_info_s		*pReportSatInfo = nullptr;
};
