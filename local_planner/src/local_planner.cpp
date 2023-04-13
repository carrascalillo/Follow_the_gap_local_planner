#include "local_planner/local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

LocalPlanner::~LocalPlanner() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
    
    //initializing nodes
    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    laser_sub = n.subscribe("/front/scan", 10, &LocalPlanner::laserCallback, this);
    amcl_sub = n.subscribe("amcl_pose", 10, &LocalPlanner::amclCallback, this);
    
    //in order to get the global time just one time
    first_time = true;
    finish = false;
    
    //set velocity to 0
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    next.x = 0.0;
    next.y = 0.0;
    
    numberRegions = 0;
    numberEdges = 0;
    maxRegionAngle = 0;
    thereIsRegion = false;
    dmin = 30;
    conto = 0;
    R = 0.5;
    thereAreObstacles = false;
    step = 0;
    securityFlag = false;
    cooldown = 0;
}

bool LocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    
    if(first_time){
    	plan_count = 0;
    	plan = orig_global_plan;
    	length = plan.size();
    	setGoal();
    	//ROS_INFO("Longitud del plan: %d", length);
    	//ROS_INFO("%f", plan[3].pose.position.x);
	next.x = plan[plan_count].pose.position.x;
	next.y = plan[plan_count].pose.position.y;
    	first_time = false;
    }
    return true;
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    //ROS_INFO("------------------------------------------------------------------------------------------------");
    //ROS_INFO("Velocidad lineal: %f", cmd.linear.x);
    //ROS_INFO("Velocidad angular: %f", cmd.angular.z);
    //cmd.linear.x = 0.2;
    //cmd.angular.z = 0.2;
    //cmd_vel = cmd;
    //vel_pub.publish(cmd_vel);
    setNowError();

    //funcionamiento FTG
    setRegions();
    checkRegions();
    setFTGYaw();
    //region[0].start = edge[0].degree;
    
    if(distance_ < 0.3){
    	if((length - 1 - plan_count) <= 1){
    	    finish = true;
    	} else {
    	    if((length - 10 - plan_count) < 0){
    	    	step = length - plan_count - 1;
    	    	plan_count += step;
    	    } else {
    	        plan_count += 10;
    	        setGoal();
    	    }
    	}
    } else {
        if(fabs(FTGYaw) > (25*PI/180)){
            setRot();
        } else {
            setVel();
        }
    }
    cmd_vel = cmd;
    vel_pub.publish(cmd_vel);
    //ROS_INFO("plan count: %d ... linear.x: %f ... angular.z: %f", plan_count, cmd.linear.x, cmd.angular.z);
    //ROS_INFO("plan count: %d", plan_count);
    
    
    /*if(conto < 100){
    	ROS_INFO("LENTGH: %d, PLAN_COUNT: %d", length, plan_count);
    	conto = 0;
    }
    conto++;*/
    
    /*if(conto < 200){
    	ROS_INFO("DISTANCE TO SUBGOAL: %f", distance_);
    	conto = 0;
    }
    conto++;*/
    
    return true;
}

bool LocalPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}

void LocalPlanner::laserCallback(const sensor_msgs::LaserScan::Ptr& msg){
    laserData = *msg;
    //ROS_INFO("longitud: %ld", laserData.ranges.size());
    for(int i = 0; i < laserMax; i++){
    	if(laserData.ranges[i] > 4){
    	    laserData.ranges[i] = 4;
    	    
    	    //security condition
    	    /*if(laserData.ranges[i] < 0.5){
    	    	securityFlag = true;
    	    }*/
    	}
    }
    
    /*if(securityFlag && (length - 2 - plan_count) < 1 && cooldown == 0){
    	plan_count++;
    	cooldown = 29;
    }
    
    if(securityFlag){
    	cooldown--;
    }
    
    securityFlag = false;*/
    
    /*if(conto < 1){
    	for(int i = 0; i<720; i++){
    	    ROS_INFO("%f", laserData.ranges[i]);
    	}
    }
    conto++;*/
}

void LocalPlanner::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg){
    position before;
    //if(hasStarted){
    //	before.x = now.x;
    //	before.y = now.y;
    //}
    before.x = now.x;
    before.y = now.y;
    now.x = msg->pose.pose.position.x;
    now.y = msg->pose.pose.position.y;
    now.az = getYaw(*msg); 
    setNowError();

}

double LocalPlanner::getYaw(geometry_msgs::PoseWithCovarianceStamped msg){

    double quaternion[4];
    quaternion[0] = msg.pose.pose.orientation.x;
    quaternion[1] = msg.pose.pose.orientation.y;
    quaternion[2] = msg.pose.pose.orientation.z;
    quaternion[3] = msg.pose.pose.orientation.w;
    
    double t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
    double t4 = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    
    return std::atan2(t3, t4);
}

void LocalPlanner::setNowError(){

    double ang;

    error.x = (next.x - now.x);
    error.y = (next.y - now.y);

    if (error.y == 0 & error.x == 0){  
    	ang = now.az;
    }else{	
	ang = std::atan2(error.y, error.x);
    }

    distance_ = std::sqrt(error.x*error.x + error.y*error.y);
    error.az = ang - now.az;

    if ( error.az > PI ){
    error.az -= 2*PI;
    }
    if ( error.az < -PI ){
    error.az += 2*PI;
    }
    //ROS_INFO("Distancia: %f", distance_);
    //ROS_INFO("error.x: %f, error.y: %f, error.az: %f", error.x, error.y, error.az);
    //ROS_INFO("now.x: %f, now.y: %f, next.x: %f, next.y: %f", now.x, now.y, next.x, next.y);

}

void LocalPlanner::setGoal(){

    next.x = plan[plan_count].pose.position.x;
    next.y = plan[plan_count].pose.position.y;
    
    if(conto > 10){
    
    	ROS_INFO("NEXT.X: %f, NEXT.Y: %f", plan[plan_count].pose.position.x, plan[plan_count].pose.position.y);
    	conto = 0;
    }
    
    conto++;
}

void LocalPlanner::setVel(){
    //cmd.linear.x = 1.5 * distance_;
    //cmd.angular.z = 0.75 * error.az;
    cmd.linear.x = 1.25 * distance_;
    cmd.angular.z = 0.75 * FTGYaw;
}

void LocalPlanner::setRot(){
    //cmd.linear.x = 0.1;
    //cmd.angular.z = error.az;
    cmd.linear.x = 0.1;
    cmd.angular.z = FTGYaw;
}

void LocalPlanner::setRegions(){

    int j = 1;
    int k = 0;
    
    /*for(int i = 0; i < 720; i++){
    
    	if(fabs(laserData.ranges[i] - laserData.ranges[i+1]) > 2*R){
    	
    	//conto = fabs(laserData.ranges[i] - laserData.ranges[i+1]);
    	//ROS_INFO("diferencia: %f", conto);
    	
    	    if(laserData.ranges[i] < laserData.ranges[i+1]){
    	    
    	    	edge[j].down = -1;
    	    	edge[j].degree = (i-360)*0.375;
    	    	edge[j].depth = laserData.ranges[i];
    	    	
    	    } else {
    	    
    	    	edge[j].down = 1;
    	    	edge[j].degree = (i+1-360)*0.375;
    	    	edge[j].depth = laserData.ranges[i];
    	    }
    	    j++;
    	}
    }*/
    
    for(int i = 120; i < 600; i++){
    
    	if(fabs(laserData.ranges[i] - laserData.ranges[i+1]) > 2*R){
    	
    	//conto = fabs(laserData.ranges[i] - laserData.ranges[i+1]);
    	//ROS_INFO("diferencia: %f", conto);
    	
    	    if(laserData.ranges[i] < laserData.ranges[i+1]){
    	    
    	    	edge[j].down = -1;
    	    	edge[j].degree = (i-360)*0.375;
    	    	edge[j].depth = laserData.ranges[i];
    	    	
    	    } else {
    	    
    	    	edge[j].down = 1;
    	    	edge[j].degree = (i+1-360)*0.375;
    	    	edge[j].depth = laserData.ranges[i];
    	    }
    	    j++;
    	}
    }
    
    edge[0].down = -edge[1].down;
    edge[0].degree = -90;
    edge[0].depth = laserData.ranges[120];
    
    edge[j].down = -edge[j-1].down;
    edge[j].degree = 90;
    edge[j].depth = laserData.ranges[599];
    
    /*edge[0].down = -edge[1].down;
    edge[0].degree = -135;
    edge[0].depth = laserData.ranges[0];
    
    edge[j].down = -edge[j-1].down;
    edge[j].degree = 135;
    edge[j].depth = laserData.ranges[719];*/
    
    j++;
    
    /*for(int i = 0; i < j; i++){
    	ROS_INFO("edge[%d].degree: %f", i, edge[i].degree);
    }*/
    
    //ROS_INFO("j: %d", j);
    

    for(int i = 0; i < (j - 1); i++){
    
    	if(edge[i].down == -1){
    	
    	    region[k].start = edge[i].degree;
    	    region[k].startDepth = edge[i].depth;
    	    region[k].finish = edge[i+1].degree;
    	    region[k].finishDepth = edge[i+1].depth;
    	    region[k].checked = 0;
    	    k++;
    	    
    	} else if (edge[i].down == 1){
    	
    	    if(edge[i+1].down == 1){
    	    
    	    	region[k].start = edge[i].degree;
    	    	region[k].startDepth = edge[i].depth;
    	    	region[k].finish = edge[i+1].degree;
    	    	region[k].finishDepth = edge[i+1].depth;
    	    	region[k].checked = 0;
    	    	k++;
    	    }
    	}
    	
    }
    
    numberEdges = j;
    numberRegions = k;
    
    //ROS_INFO("NUMBER REGIONS: %d", numberRegions);
    /*conto++;
    if(conto > 5){
	    for(int i = 0; i < numberEdges; i++){
	    	ROS_INFO("DEGREE: %f, DOWN: %d", edge[i].degree, edge[i].down);
	    }
	    conto = 0;
	    ROS_INFO("cmd.linear.x: %f, cmd.angular.z; %f", cmd.linear.x, cmd.angular.z);
	    ROS_INFO("------------------------------------");
    }*/
}

void LocalPlanner::checkRegions(){

    int j;
    double b = 0;
    double c = 0;
    double angle = 0;
    double maxLength = 0;
    
    thereIsRegion = false;
    
    for(int i = 0; i < numberRegions; i++){
    
	if(!region[i].checked){
	
	    region[i].checked = true;
	    if((region[i].start > -90 && region[i].start < 90) && (region[i].finish > -90 && region[i].finish < 90)){
	    
	    	angle = (region[i].finish - region[i].start)*PI/180;
	    	b = region[i].startDepth;
	    	c = region[i].finishDepth;
	    	region[i].length = std::sqrt(b*b + c*c - 2 * b * c * std::cos(angle));
	    	//ROS_INFO("b: %f,  c: %f, LENGTH: %f, i: %d", b, c, region[i].length, i);
	    }
	    
	    if((region[i].length > 2*R) && (region[i].length > maxLength)){
	    	maxLength = region[i].length;
	    	j = i;
	    }
	}
    }
    
    if(maxLength){
    	maxRegionAngle = ((region[j].start + region[j].finish)/2)*PI/180;
    	thereIsRegion = true;
    	if(region[j].startDepth < region[j].finishDepth){
    	    dmin = region[j].startDepth;
    	} else {
    	    dmin = region[j].finishDepth;
    	}
    }
    
}

void LocalPlanner::setFTGYaw(){
    
    thereAreObstacles = false;
    
    for(int i = 0; i < 720; i++){
    	if(laserData.ranges[i] < 1.5){
    	    thereAreObstacles = true;
    	}
    }
    /*if(thereIsRegion && thereAreObstacles){
    	FTGYaw = ((alpha/dmin)*maxRegionAngle + beta*error.az) / (alpha/dmin + beta);
    } else {
    	FTGYaw = error.az;
    }*/
    if(thereIsRegion && thereAreObstacles){
    	FTGYaw = ((alpha/dmin)*maxRegionAngle + (beta/distance_)*error.az) / (alpha/dmin + beta/(distance_ + 0.001));
    } else {
    	FTGYaw = error.az;
    }
    
    /*if(conto > 6){
    	ROS_INFO("MAX_REGION_ANGLE: %f, FTG_YAW: %f", maxRegionAngle, FTGYaw);
    	conto = 0;
    } else {
    	conto++;
    }*/
}

}
