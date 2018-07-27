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
        std::string _port = std::string("/dev/ttyUSB0"), 
        float _surveyAccuracy = 1.0,
        float _surveyDuration = 90.0): 
        connected(false), baud(_baud), port(_port),
        surveyAccuracy(_surveyAccuracy), surveyDuration(_surveyDuration) {

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
        if (gpsDriver->configure(baud, GPSDriverUBX::OutputMode::RTCM) == 0) {

            /* reset report */
            memset(&reportGPSPos, 0, sizeof(reportGPSPos));

            //In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
            //bus errors or buggy firmware. In this case we want to try multiple times before giving up.
            int numTries = 0;

            while (ros::ok() && numTries < 3) {
                int helperRet = gpsDriver->receive(500);

                if (helperRet > 0) {
                    numTries = 0;

                    if (helperRet & 1) {
                        publishGPSPosition();
                        numTries = 0;
                    }

                    if (pReportSatInfo && (helperRet & 2)) {
                        publishGPSSatellite();
                        numTries = 0;
                    }
                } else {
                    ++numTries;
                }
            }
        }
    };

    void publishGPSPosition() {
        
    };

    void publishGPSSatellite() {
        
    };

    void connect_gps() {
        // dynamic model
        uint8_t stationary_model = 2;
        gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &reportGPSPos, pReportSatInfo, stationary_model);
        gpsDriver->setSurveyInSpecs(surveyAccuracy * 10000, surveyDuration);
        memset(&reportGPSPos, 0, sizeof(reportGPSPos)); // Reset report
    };


    static int callbackEntry(GPSCallbackType type, void *data1, int data2, void *user)
    {
        // GPSProvider *gps = (GPSProvider *)user;
        // return gps->callback(type, data1, data2);
        callback(type, data1, data2);
    };

    static int callback(GPSCallbackType type, void *data1, int data2)
    {
        // switch (type) {
        //     case GPSCallbackType::readDeviceData: {
        //         if (serial->bytesAvailable() == 0) {
        //             int timeout = *((int *) data1);
        //             if (!serial->waitForReadyRead(timeout))
        //                 return 0; //timeout
        //         }
        //         return (int)serial->read((char*) data1, data2);
        //     }
        //     case GPSCallbackType::writeDeviceData:
        //         if (serial->write((char*) data1, data2) >= 0) {
        //             if (serial->waitForBytesWritten(-1))
        //                 return data2;
        //         }
        //         return -1;

        //     case GPSCallbackType::setBaudrate:
        //         return serial->setBaudrate(data2) ? 0 : -1;

        //     case GPSCallbackType::gotRTCMMessage:
        //         gotRTCMData((uint8_t*) data1, data2);
        //         break;

        //     case GPSCallbackType::surveyInStatus:
        //     {
        //         SurveyInStatus* status = (SurveyInStatus*)data1;
        //         qCDebug(RTKGPSLog) << QString("Survey-in status: %1s cur accuracy: %2mm valid: %3 active: %4").arg(status->duration).arg(status->mean_accuracy).arg((int)(status->flags & 1)).arg((int)((status->flags>>1) & 1));
        //         emit surveyInStatus(status->duration, status->mean_accuracy, (int)(status->flags & 1), (int)((status->flags>>1) & 1));
        //     }
        //         break;

        //     case GPSCallbackType::setClock:
        //         /* do nothing */
        //         break;
        // }

        // return 0;
    };


    bool connected;
private:
    unsigned baud;
    std::string port;
    float surveyAccuracy;
    float surveyDuration;
    GPSHelper* gpsDriver = nullptr;
    serial::Serial* serial = nullptr;
	struct vehicle_gps_position_s	reportGPSPos;
	struct satellite_info_s		*pReportSatInfo = nullptr;
};
