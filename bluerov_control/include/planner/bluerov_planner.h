#include "utility.h"
// this is a plugin that receives path from PRM global planner and pass it to move_base
#ifndef bluerov_planner_CPP
#define bluerov_planner_CPP

namespace bluerov_planner {


    class BluerovPlanner : public nav_core::BaseGlobalPlanner {

    public:
        ros::NodeHandle nh;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        ros::Subscriber subPath;
        ros::Publisher pubGoal;

        // visualize twist command
        ros::Subscriber subTwistCommand; // twist command from move_base
        ros::Publisher pubTwistCommand; // adjust twist command height to show above the robot

        nav_msgs::Path globalPath;

        std::mutex mtx;

        BluerovPlanner(); 
        BluerovPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg);
        // visualize twist command
        void twistCommandHandler(const nav_msgs::Path::ConstPtr& pathMsg);

        bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);  

    };


};


#endif