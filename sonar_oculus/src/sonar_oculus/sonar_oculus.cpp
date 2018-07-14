// VSieben@slb.com (original author, 2017)
// pvt@mit.edu     (extensions, 2018)
// jwang92@stevens.edu

#include <arpa/inet.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>
#include <string>

#include "sonar_oculus/Oculus.h"
#include "sonar_oculus/OculusClient.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

#include <dynamic_reconfigure/server.h>
#include <sonar_oculus/OculusParamsConfig.h>

#define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100
#define PI 3.14159265359

// Global sonar configuration
int mode = 1;              // 0 => dev/not used, 1 => ~750khz, 2 => ~1.2Mhz.
double range = 10;         // m, limited to 120m in mode 1, and 40m in mode 2
double gain = 20;          //%
double soundspeed = 1500;  // m/s
double salinity = 0;       // ppm, 0 = freshwater, 35=saltwater

// Callback for dynamic reconfigure server
void callback(sonar_oculus::OculusParamsConfig &config, uint32_t level) {
  mode = config.Mode;
  gain = config.Gain;
  soundspeed = config.Speed;
  range = config.Range;
  salinity = config.Salinity;

  ROS_INFO("Reconfigure Request: %i %i %i %i %i", config.Mode, config.Gain,
           config.Speed, config.Range, config.Salinity);
}

int main(int argc, char **argv) {
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar_oculus");
  ros::NodeHandle nh("~");

  ros::Publisher pub, pub2, ping_pub;
  ping_pub = nh.advertise<sonar_oculus::OculusPing>("ping", 1);

  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  serverParam.setCallback(f);

  struct sockaddr_in serverUDP, clientUDP;
  struct sockaddr_in serverTCP, clientTCP;
  int sockUDP, sockTCP, sockTCPfd, datagramSize, n;
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerUDP, lengthClientUDP;
  socklen_t lengthServerTCP, lengthClientTCP;
  char datagramMessage[BUFLEN], buffer[BUFLEN], sonardata[DATALEN];

  OsClientCtrl m750d;
  std::string frame_str;

  // Clear and intialize values of server and client network info
  lengthServerUDP = sizeof(serverUDP);
  bzero((char *)&serverUDP, lengthServerUDP);
  serverUDP.sin_family = AF_INET;
  serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  serverUDP.sin_port = htons(PORT_UDP);

  lengthClientUDP = sizeof(clientUDP);
  lengthServerTCP = sizeof(serverTCP);

  ROS_INFO("Connecting...");
  sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockUDP < 0) ROS_ERROR("Error opening UDP listening socket");

  if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
    ROS_ERROR("Error binding UDP listening socket");

  unsigned int latest_id = 0;
  listen(sockUDP, 5);
  while (true) {
    int64_t bytesAvailable;
    ioctl(sockUDP, FIONREAD, &bytesAvailable);

    OculusStatusMsg osm;
    if (bytesAvailable > 0) {
      unsigned bytesRead = read(sockUDP, (char *)&osm, bytesAvailable);
      struct in_addr ip_addr;
      ip_addr.s_addr = osm.ipAddr;
      ROS_INFO_STREAM("The IP address is " << inet_ntoa(ip_addr));

      bzero((char *)&serverTCP, lengthServerTCP);
      serverTCP.sin_family = AF_INET;
      serverTCP.sin_addr.s_addr = osm.ipAddr;
      serverTCP.sin_port = htons(PORT_TCP);

      sockTCP = socket(AF_INET, SOCK_STREAM, 0);
      if (sockTCP < 0) ROS_ERROR("Error opening TCP main socket");
      if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
        ROS_ERROR("Error connecting TCP socket");
      if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size,
                     sizeof(buf_size)) < 0)
        ROS_ERROR("Error increasing RCVBUF for TCP socket");
      if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive,
                     sizeof(keepalive)) < 0)
        ROS_ERROR("Error keeping alive option set for TCP socket");
      listen(sockTCP, 5);
      break;
    }
    ros::Duration(1.0).sleep();
  }

  m750d.m_readData.m_pSocket = &sockTCP;
  m750d.Connect();
  ROS_INFO("Connected!");

  m750d.Fire(mode, range, gain, soundspeed, salinity);
  if (nh.getParam("frame", frame_str)) {
    ROS_INFO("Got param: %s", frame_str.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frame'");
    frame_str = "/sonar";
  }

  ros::Rate rate(50);
  while (ros::ok()) {
    m750d.m_readData.run();

    unsigned int nbins = m750d.m_readData.m_osBuffer[0].m_rfm.nRanges;
    unsigned int nbeams = m750d.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = m750d.m_readData.m_osBuffer[0].m_rfm.pingId;

    if (nbeams > 0 && nbins > 0 && id > latest_id) {
      latest_id = id;

      if (m750d.m_readData.m_osBuffer[0].m_rawSize) {
        sensor_msgs::Image sonar_image;
        sonar_image.header.stamp = ros::Time::now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        sonar_image.step = nbins;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(m750d.m_readData.m_osBuffer[0].m_pImage,
                  m750d.m_readData.m_osBuffer[0].m_pImage + nbins * nbeams,
                  sonar_image.data.begin());

        sonar_oculus::OculusFire fire_msg;
        fire_msg.header.stamp = sonar_image.header.stamp;

        fire_msg.mode =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
        fire_msg.gamma =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
        fire_msg.flags = m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
        fire_msg.range = m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
        fire_msg.gain =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
        fire_msg.speed_of_sound =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
        fire_msg.salinity =
            m750d.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

        sonar_oculus::OculusPing ping_msg;
        ping_msg.header.frame_id = frame_str;
        ping_msg.header.stamp = fire_msg.header.stamp;
        ping_msg.ping = sonar_image;
        ping_msg.fire_msg = fire_msg;
        ping_msg.ping_id = id;
        ping_msg.status = m750d.m_readData.m_osBuffer[0].m_rfm.status;
        ping_msg.frequency = m750d.m_readData.m_osBuffer[0].m_rfm.frequency;
        ping_msg.temperature = m750d.m_readData.m_osBuffer[0].m_rfm.temperature;
        ping_msg.pressure = m750d.m_readData.m_osBuffer[0].m_rfm.pressure;
        ping_msg.speed_of_sound =
            m750d.m_readData.m_osBuffer[0].m_rfm.speedOfSoundUsed;

        ping_msg.start_time =
            m750d.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (int i = 0; i < nbeams; ++i)
          ping_msg.bearings[i] = m750d.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution =
            m750d.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;

        ping_pub.publish(ping_msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
    m750d.Fire(mode, range, gain, soundspeed, (double)salinity);
  }

  m750d.Disconnect();
  close(sockUDP);
  close(sockTCP);
  return 0;

}  // main
