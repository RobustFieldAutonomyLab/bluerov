#include "utility.h"

int main(int argc, char **argv)
{
  // Initialize ROS node and topics
  ros::init(argc, argv, "fake_map");
  ros::NodeHandle nh("~");

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_map", 1);

  nav_msgs::OccupancyGrid occupancyMap2D;

  // Transform Listener (get robot position)
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // initialization of map message
  occupancyMap2D.header.frame_id = "map";
  occupancyMap2D.info.width = 500;
  occupancyMap2D.info.height = 500;
  occupancyMap2D.info.resolution = 0.1;

  occupancyMap2D.info.origin.orientation.x = 0.0;
  occupancyMap2D.info.origin.orientation.y = 0.0;
  occupancyMap2D.info.origin.orientation.z = 0.0;
  occupancyMap2D.info.origin.orientation.w = 1.0;

  occupancyMap2D.data.resize(occupancyMap2D.info.width * occupancyMap2D.info.height);
  std::fill(occupancyMap2D.data.begin(), occupancyMap2D.data.end(), 0);

  ros::Rate rate(1);
  while (ros::ok())
  {
    try
    {
      listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Transfrom Failure.");
      rate.sleep();
      continue;
    }

    float x = transform.getOrigin().x();
    float y = transform.getOrigin().y();
    float z = transform.getOrigin().z();

    occupancyMap2D.header.stamp = ros::Time::now();

    occupancyMap2D.info.origin.position.x = x - occupancyMap2D.info.width * occupancyMap2D.info.resolution / 2.0f;
    occupancyMap2D.info.origin.position.y = y - occupancyMap2D.info.height * occupancyMap2D.info.resolution / 2.0f;

    pub.publish(occupancyMap2D);

    rate.sleep();
  }

  return 0;
}
