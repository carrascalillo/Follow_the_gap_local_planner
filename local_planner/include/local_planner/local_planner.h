#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265

using namespace std;

namespace local_planner{

    struct position{
        double x, y, az;
    };
    
    struct edges{
    	int down;
    	double degree, depth;
    };
    
    struct regions{
    	double start, finish;
    	double startDepth, finishDepth, length;
    	bool checked;
    };

class LocalPlanner : public nav_core::BaseLocalPlanner{
public:

    LocalPlanner();
    LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~LocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    bool initialized_;
    
    position now;
    position next;
    position error;
    double distance_;
    int plan_count;
    int length;
    bool first_time;
    bool finish;
    
    edges edge[540]; //maximum number of gaps in 720 samples
    regions region[270]; // maximum of regions is 540/2
    
    int numberRegions;
    int numberEdges;
    double maxRegionAngle;
    bool thereIsRegion;
    double FTGYaw;
    const double alpha = 1;
    const double beta = 8;
    double dmin;
    double conto;
    bool thereAreObstacles;
    int step = 0;
    bool securityFlag;
    int cooldown;

    
    //laser has 720 samples
    const int laserMax = 720;
    double R;
    
    ros::Publisher vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber amcl_sub;
    geometry_msgs::Twist cmd;
    sensor_msgs::LaserScan laserData;
    std::vector<geometry_msgs::PoseStamped> plan;
    
    
    //laserCallback: function called whenever a laser msg is published

    void laserCallback(const sensor_msgs::LaserScan::Ptr& msg);

    //amclCallback: function called whenever an amcl message is published
    
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

    //setNowError: calculates the current distance to the goal and the angle error to the goal
    
    void setNowError();

    //getYaw: calculates yaw angle with the message the amcl gave
    
    double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

    //setGoal: sets the next subgoal
    
    void setGoal();

    //setVel: sets the velocity parameters of the robot
    
    void setVel();

    //setRot: sets the rotation of the robot if the angle to the next subgoal is too wide
    
    void setRot();

    //setRegions: calculates the regions the laser can provide with edges
    
    void setRegions();

    //checkRegions: verifies that the regions calculated are safe to drive in and chooses the max region
    
    void checkRegions();

    //setFTGYaw: computes the angle the robot needs to follow taking into account the angle error to the goal and the angle to the max region center
    
    void setFTGYaw();
};
};

#endif
