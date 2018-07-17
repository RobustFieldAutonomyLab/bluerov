// VSieben@slb.com (original author, 2017)
// pvt@mit.edu     (extensions, 2018)

#include <algorithm>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "Oculus.h"
#include "OculusClient.h"

// ROS includes
#include <ros/ros.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sonar_oculus/OculusFire.h>
#include <sonar_oculus/OculusPing.h>

// Dynamic server
#include <dynamic_reconfigure/server.h>
#include <sonar_oculus/OculusParamsConfig.h>

#define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100
//#define SONAR_ADDR "169.254.37.89"
#define SONAR_ADDR "169.254.30.199"
//#define SONAR_ADDR "10.0.0.128"
#define PI 3.14159265359

// Global sonar configuration
int mode = 1;             // 0 => dev/not used, 1 => ~750khz, 2 => ~1.2Mhz.
double range = 10;        // m, limited to 120m in mode 1, and 40m in mode 2
double gain = 20;         //%
double soundspeed = 1500; // m/s
double salinity = 0;      // ppm, 0 = freshwater, 35=saltwater
int threshold = 90;       // intensity threshold

// Error handling function
void error(const char *msg) {
  perror(msg);
  exit(0);
}

// Callback for dynamic reconfigure server
void callback(sonar_oculus::OculusParamsConfig &config, uint32_t level) {
  mode = config.Mode;
  gain = config.Gain;
  soundspeed = config.Speed;
  range = config.Range;
  salinity = config.Salinity;
  threshold = config.Threshold;

  ROS_INFO("Reconfigure Request: %i %i %i %i %i", config.Mode, config.Gain,
           config.Speed, config.Range, config.Salinity);
}

// Main program for listening to sonar
int main(int argc, char **argv) {
  // Initialize ROS
  ROS_INFO("Initializing...");
  ros::init(argc, argv, "sonar_oculus");
  ros::NodeHandle nh("~");

  unsigned int latest_id = 0; // keep track of latest ping to avoid republishing

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub, pub2, ping_pub;
  // pub = nh.advertise<sensor_msgs::PointCloud>("sonar_oculus_output", 1);
  // pub2 = nh.advertise<sensor_msgs::LaserScan>("sonar_oculus_laserEQ", 1);
  ping_pub = nh.advertise<sonar_oculus::OculusPing>("ping", 1);

  // Setup dynamic server
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig> serverParam;
  dynamic_reconfigure::Server<sonar_oculus::OculusParamsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  serverParam.setCallback(f);

  // Variable declarations
  // Communications
  struct sockaddr_in serverUDP, clientUDP;
  struct sockaddr_in serverTCP, clientTCP;
  int sockUDP, sockTCP, sockTCPfd, datagramSize, n;
  int buf_size = DATALEN;
  int keepalive = 1;
  socklen_t lengthServerUDP, lengthClientUDP;
  socklen_t lengthServerTCP, lengthClientTCP;
  char datagramMessage[BUFLEN], buffer[BUFLEN], sonardata[DATALEN];

  // Create sonar oculus control class
  OsClientCtrl m750d;

  // Sonar info
  unsigned int nbins = 0;
  unsigned int nbins_prev = 0;
  unsigned int nbeams = 0;
  unsigned int nbeams_prev = 0;
  float r_step = 0.0;
  float windowed_avg = 0.0;
  unsigned int lbeams_EQ = 0;
  float lbeam_current = 0.0;

  // PointCloud and laserscan messages.
  // sensor_msgs::PointCloud sonar_cloud;
  // sensor_msgs::LaserScan sonar_laserEQ;
  std::string frame_str;

  // Clear and intialize values of server and client network info
  lengthServerUDP = sizeof(serverUDP);
  bzero((char *)&serverUDP, lengthServerUDP);
  serverUDP.sin_family = AF_INET;
  serverUDP.sin_addr.s_addr = htonl(INADDR_ANY);
  // serverUDP.sin_addr.s_addr = inet_addr(SONAR_ADDR);
  serverUDP.sin_port = htons(PORT_UDP);

  lengthClientUDP = sizeof(clientUDP);
  lengthServerTCP = sizeof(serverTCP);
  
  ROS_INFO("Connecting...");
  // Create the UDP listening socket or exit
  sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockUDP < 0)
    error("Error opening UDP listening socket");

  // Bind the UDP socket to address and port, or exit with error
  if (bind(sockUDP, (struct sockaddr *)&serverUDP, lengthServerUDP) < 0)
    error("Error binding UDP listening socket");
  listen(sockUDP, 5);

  while (true) {
    int64_t bytesAvailable;
    ioctl(sockUDP, FIONREAD, &bytesAvailable);

    OculusStatusMsg osm;
    if (bytesAvailable > 0) { 
      unsigned bytesRead = read(sockUDP, (char*)&osm, bytesAvailable);
      struct in_addr ip_addr;
      ip_addr.s_addr = osm.ipAddr;
      printf("The IP address is %s\n", inet_ntoa(ip_addr));

      bzero((char *)&serverTCP, lengthServerTCP);
      serverTCP.sin_family = AF_INET;
      serverTCP.sin_addr.s_addr = osm.ipAddr;
      serverTCP.sin_port = htons(PORT_TCP);

  // Create the TCP socket for main communication or exit
  sockTCP = socket(AF_INET, SOCK_STREAM, 0);
  if (sockTCP < 0)
    error("Error opening TCP main socket");
  // Connect to the sonar Server via TCP socket or exit with error
  if (connect(sockTCP, (struct sockaddr *)&serverTCP, lengthServerTCP) < 0)
    error("Error connecting TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) <
      0)
    error("Error increasing RCVBUF for TCP socket");
  if (setsockopt(sockTCP, SOL_SOCKET, SO_KEEPALIVE, &keepalive,
                 sizeof(keepalive)) < 0)
    error("Error keeping alive option set for TCP socket");
  listen(sockTCP, 5);
      break;
    }
    ros::Duration(1.0).sleep();
  }

  // Setup Sonar and messages
  // Pass the socket to the control
  m750d.m_readData.m_pSocket = &sockTCP;
  // Connect and instance a thread
  m750d.Connect();

  ROS_INFO("Connected!");

  // Send Ping and initiate data collection
  m750d.Fire(mode, range, gain, soundspeed, salinity);
  // Get frame ID
  if (nh.getParam("frame", frame_str)) {
    ROS_INFO("Got param: %s", frame_str.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frame'");
    frame_str = "/sonar";
  }
  // sonar_cloud.header.frame_id = frame_str.c_str();
  // sonar_laserEQ.header.frame_id = frame_str.c_str();

  // Run continously
  ros::Rate r(50); // pvt: sonar should be under 40Hz (reduced 100 to 50)
  while (ros::ok()) {

    // Run the readthread sonar
    m750d.m_readData.run();

    // Get bins and beams #.
    nbins = m750d.m_readData.m_osBuffer[0].m_rfm.nRanges;
    nbeams = m750d.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = m750d.m_readData.m_osBuffer[0].m_rfm.pingId;

    // Create pointcloud message from sonar data
    if (nbeams > 0 && nbins > 0 && id > latest_id) {
      latest_id = id;

      // Resize PointCloud and channel for intensities
      // sonar_cloud.header.stamp = ros::Time::now();
      // sonar_cloud.points.resize(nbeams * nbins);
      // sonar_cloud.channels.resize(1);
      // sonar_cloud.channels[0].name = "Intensity";
      // sonar_cloud.channels[0].values.resize(nbeams * nbins);

      // sonar image
      if ( m750d.m_readData.m_osBuffer[0].m_rawSize){
      //if (0){
        sensor_msgs::Image sonar_image;
        sonar_image.header.stamp = ros::Time::now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        // sonar_image.is_bigendian = 0; // default works
        sonar_image.step = nbins;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(m750d.m_readData.m_osBuffer[0].m_pImage,
                  m750d.m_readData.m_osBuffer[0].m_pImage +
                  nbins*nbeams,
                  sonar_image.data.begin());

        // fire msg
        sonar_oculus::OculusFire fire_msg;
        fire_msg.header.stamp = ros::Time::now();

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

        // sonar ping
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

        ping_msg.start_time = m750d.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (int i = 0; i < nbeams; ++i)
            ping_msg.bearings[i] = m750d.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution =
          m750d.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;

        ping_pub.publish(ping_msg);
      }

      // Acquire sonar range spatial data
      r_step = m750d.m_readData.m_osBuffer[0].m_rfm.rangeResolution;

      // Set LaserScan parameters, which may change if user modifies
      // sonar_laserEQ.header.stamp = ros::Time::now();
      // sonar_laserEQ.angle_min =
      //     (float)m750d.m_readData.m_osBuffer[0].m_pBrgs[0] * PI / 18000.0;
      // sonar_laserEQ.angle_max =
      //     (float)m750d.m_readData.m_osBuffer[0].m_pBrgs[nbeams - 1] * PI /
      //     18000.0;
      // sonar_laserEQ.angle_increment =
      //     PI / 18000; //(sonar_laserEQ.angle_max-sonar_laserEQ.angle_min) /
      //                 //(nbeams-1);//need to take out min and max end points
      // sonar_laserEQ.time_increment = 0; //(1/10.0); // Ping rate is 10 hz
      // sonar_laserEQ.scan_time = 1;
      // sonar_laserEQ.range_min = 0.1;   // meters, min per spec sheet
      // sonar_laserEQ.range_max = range; // meters, max set by user up to 120m
      // lbeams_EQ = ((int)m750d.m_readData.m_osBuffer[0].m_pBrgs[nbeams - 1] -
      //              (int)m750d.m_readData.m_osBuffer[0].m_pBrgs[0]);
      // sonar_laserEQ.ranges.assign(lbeams_EQ, 0.0);
      // sonar_laserEQ.intensities.assign(lbeams_EQ, 0.0);

      // Generate PoinCloud data
      // for (unsigned int i = 0; i < nbins; i++) {
      //   for (unsigned int j = 0; j < nbeams; j++) {
      //     // Cloud parameters
      //     // Generate X,Y,Z coordinates from swath data
      //     if (nbins != nbins_prev || nbeams != nbeams_prev) {
      //       sonar_cloud.points[i * nbeams + j].x =
      //           cos((float)m750d.m_readData.m_osBuffer[0].m_pBrgs[j] * PI /
      //               18000.0) *
      //           (i * r_step);
      //       sonar_cloud.points[i * nbeams + j].y =
      //           sin((float)m750d.m_readData.m_osBuffer[0].m_pBrgs[j] * PI /
      //               18000.0) *
      //           (i * r_step);
      //       sonar_cloud.points[i * nbeams + j].z = 0;
      //     }
      //     // Assigned intensities
      //     sonar_cloud.channels[0].values[i * nbeams + j] =
      //         m750d.m_readData.m_osBuffer[0].m_pImage[i * nbeams + j];
      //     // ROS_INFO("Bearings: %5.5f",
      //     // (float)m750d.m_readData.m_osBuffer[0].m_pBrgs[j]);
      //   } // for nbeams
      // }   // for nbins

      // Generate LaserScan Range data
      // for (unsigned int j = 0; j < nbeams; j++) {
      //   for (unsigned int i = 0; i < nbins - 10; i++) {
      //     windowed_avg = 0.0;
      //     for (unsigned int zz = 0; zz < 10; zz++) {
      //       windowed_avg +=
      //           m750d.m_readData.m_osBuffer[0].m_pImage[i * nbeams + j];
      //     }
      //     windowed_avg /= 10.0;
      //     // Determine first echo that exceeds predefined threshold for laser
      //     // equivalent
      //     if (windowed_avg > threshold) {
      //       lbeam_current = (float)m750d.m_readData.m_osBuffer[0].m_pBrgs[j] -
      //                       (float)m750d.m_readData.m_osBuffer[0].m_pBrgs[0];
      //       // Ensure bounded for array
      //       if (unsigned(lbeam_current) < lbeams_EQ) {
      //         sonar_laserEQ.ranges[(unsigned)lbeam_current] = r_step * i;
      //       }
      //       break;
      //     }
      //   }
      // }

      nbins_prev = nbins;
      nbeams_prev = nbeams;

      // ROS_INFO("data:%lu", sonar_laserEQ.ranges.size());
      //%5.5f, %5.5f, %5.5f",sonar_laserEQ.angle_min, sonar_laserEQ.angle_max,
      // sonar_laserEQ.angle_increment);
      //%hi, %hi ... m750d.m_readData.m_osBuffer[0].m_pBrgs[0],
      // m750d.m_readData.m_osBuffer[0].m_pBrgs[nbeams-1]);
      //%5.5f ... m750d.m_readData.m_osBuffer[0].m_rfm.frequency);

      // Publish sonar data and laserEQ
      // pub.publish(sonar_cloud);
      // pub2.publish(sonar_laserEQ);
      // ping_pub.publish(sonar_ping);
      // ROS_INFO("Pinging...");
    } // if (nbins>0 && nbeams>0 && id>latest_id)

    // Fire sonar (so we sleep while the ping travels)
    m750d.Fire(mode, range, gain, soundspeed, (double)salinity);
    r.sleep();
    // Process ROS events
    ros::spinOnce();
  }

  // Disconnect and close
  m750d.Disconnect();

  // Exit
  close(sockUDP);
  close(sockTCP);
  return 0;

} // main
