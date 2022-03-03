#include<vector>
#include<tf/tf.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <memory.h>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
using namespace Eigen;
void handling(sig_atomic_t)
{
    exit(1);
}
//===================ROS Variable==================================
geometry_msgs::Twist cmd_vel;
mavros_msgs::State current_state;
sensor_msgs::LaserScan ds_data;
geometry_msgs::PoseStamped LocalPosition;
sensor_msgs::Imu imu_data;
double roll, pitch, yaw;
//===================Obstacle avoidance Variable===================
const int Max_distance=6000;
const int smallestDistanceSaturation=1500;
const double desired_eta = 100;
const uint8_t num_readings = 120;
uint8_t laser_frequency = 20;
float ranges2[num_readings];
float Final_Angle=0;
//=================================================================
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    LocalPosition = *msg;
}
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data = *msg;
    tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);
}

void Lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
// Position estimation by using a Lidar
//  0 : no obstacle
//  1 :    obstacle
// 00 : no obstacle area
// 10 : Starting point
// 01 : Ending point
// 11 : obstacle area
    vector<int> Clearance_StartAngle;
    vector<int> Clearance_EndAngle;
    uint8_t Obstacle_flag = 0;
    uint8_t Obstacle_flag_pre = 0;
    uint8_t Num_of_Obstacle=0;
    int Obstacle_distance_num=0;
    int Obstacle_distance[num_readings]={0};

    uint8_t Clearance_flag=0;
    uint8_t Clearance_flag_pre=0;
    uint8_t Num_of_Clearance=0;

    int Obstacle_point_start[120]  = {0};	
    int Obstacle_point_end[120]    = {0};
    int Obstacle_point_middle[120] = {0};
    int Num_of_obstacle_point[120] = {0};
    int Distance_of_Obstacle[120]  = {0};

    int Distance_of_Obstacle_start[20] = {0};
    int Distance_of_Obstacle_end[20]   = {0};

    int Clearance_point_start[120]  	= {0};
    int Clearance_point_end[120]    	= {0};
    int Num_of_Clearance_point[120] 	= {0};
    int Distance_of_Clearance_start[120]= {0};
    int Distance_of_Clearance_end[120]	= {0};
    // flag를 이용해 Obstacles와 clearances를 구분하는 function.

    for(int i=0; i<num_readings; i++)
    {
        ranges2[i]=(msg->ranges[i]*1000)-(int)(msg->ranges[i]*1000)%10;
        if(ranges2[i] < Max_distance)
        {
            Obstacle_flag = 1;
            Clearance_flag = 0;

            Obstacle_distance[Obstacle_distance_num]=ranges2[i];
            Obstacle_distance_num++;// Obstacle 개수 
        }
        else
        {
            Obstacle_flag = 0;
            Clearance_flag = 1;
        }
        //======================Obstacle=============================
        if(Obstacle_flag==1 && Obstacle_flag_pre==0)
        {
            Num_of_Obstacle++;
        }
        Obstacle_flag_pre=Obstacle_flag;
        //======================Clearance=============================
        //첫 시작이 clearance라면 
        if(Clearance_flag == 1 && Clearance_flag_pre == 0)
            Num_of_Clearance++;
        Clearance_flag_pre=Clearance_flag;
    }
//========================Obstacle,Clearance info =============================
    for(int i=0; i<num_readings; i++)
    {
        uint8_t Obstacle_Points = 0;
        if(ranges2[i] < Max_distance)
        {
            Obstacle_flag = 1;
            Clearance_flag = 0;
        }
        else
        {
            Obstacle_flag = 0;
            Clearance_flag = 1;
        }
        if((Obstacle_flag == 1) && (Obstacle_flag_pre == 0))
        {
            
            Obstacle_point_start[Obstacle_Points] = i; // start point pos
            Distance_of_Obstacle_start[Obstacle_Points]=ranges2[i]; // distance to start point
            Num_of_obstacle_point[Obstacle_Points] = Num_of_obstacle_point[Obstacle_Points] + 1; //

        }
        else if((Obstacle_flag == 1) && (Obstacle_flag_pre == 1))
        {
            Num_of_obstacle_point[Obstacle_Points] = Num_of_obstacle_point[Obstacle_Points] + 1; // Obstacle을 구성하는 Points의 개수를 counting
            if(i > 118)
            {
                
                Obstacle_point_end[Obstacle_Points] = i;// * 0.25 - 60;	// Obstacle Ending Point
                Distance_of_Obstacle_end[Obstacle_Points] = ds_data.ranges[i];///////////////////////////////////////////////////////
                Obstacle_Points++;	// Obstacle_point : the number of obstacle points
            }
        }
        else if((Obstacle_flag == 0) && (Obstacle_flag_pre == 1))
        {
            
            Obstacle_point_end[Obstacle_Points] = i-1;// * 0.25 - 60;	// Obstacle Ending Point
            Distance_of_Obstacle_end[Obstacle_Points] = ranges2[i-1];///////////////////////////////////////////////////////
            Obstacle_Points++;
        }
        else if((Clearance_flag == 1) && (Clearance_flag_pre == 0))
        {
            Clearance_point_start[Obstacle_Points] = i;
            Clearance_StartAngle.push_back(i);
        }
        else if((Clearance_flag == 0) && (Clearance_flag_pre == 1))
        {
            Clearance_point_end[Obstacle_Points] = i;
            Clearance_EndAngle.push_back(i);
        }
        else
        {
            Obstacle_Points = Obstacle_Points;
        }
        Obstacle_flag_pre = Obstacle_flag;
        //Known Obstacle Num, Angle, distance, points
        
    }
    //start num --> Vector(angle num)
    //Angle of obstacle and cleanrance
    //Clearance num + Obstacle num 
    vector<int> GapAngle;
    vector<double> FinalAngle;
    double Now_FinalAngle=0.0;
    double Max_FinalAngle=0.0;
    int Now_GapAngle=0;
    int Max_GapAngle_Num=0;
    int Min_Distance=0;
    float Gain_GapAngle = 1;
    float Gain_GoalAngle=1;
    float GoalAngle=0;
    for(int i=0; i<Clearance_StartAngle.size(); i++)
    {
        if(Distance_of_Clearance_end[Clearance_EndAngle[i]] > Distance_of_Clearance_start[Clearance_StartAngle[i]])
            Min_Distance = Distance_of_Clearance_start[Clearance_StartAngle[i]];
        else
            Min_Distance = Distance_of_Clearance_end[Clearance_EndAngle[i]]; // how do robot when approach to zero 
        Now_GapAngle = Clearance_EndAngle[i] - Clearance_StartAngle[i];
        GapAngle.push_back(Now_GapAngle);
        Now_FinalAngle = (Gain_GapAngle/Min_Distance*Now_GapAngle+Gain_GoalAngle*GoalAngle)/(Gain_GapAngle/Min_Distance+Gain_GoalAngle);
        FinalAngle.push_back(Now_FinalAngle);
        if(Max_FinalAngle<Now_FinalAngle)
            Max_FinalAngle = Now_FinalAngle;
    }
    if(find(FinalAngle.begin(),FinalAngle.end(), Max_FinalAngle) != FinalAngle.end())
        Max_GapAngle_Num = find(FinalAngle.begin(), FinalAngle.end(), Max_FinalAngle) - FinalAngle.begin();
    //}or SORT -> 0th index 
    Final_Angle = FinalAngle[Max_GapAngle_Num];
    //Known GapAngle Num, Max_GapAngle, obstacle distance
    

    //ranges2[Clearance_StartAngle[Max_GapAngle_Num]]

}
void Control_Robot()//차이 알아보기
{
    
    Eigen::Vector3f Desired_Position;
    Eigen::Vector3f PositionError;
    Eigen::Vector3f AttractiveForce;
    Eigen::Vector3f RuepulsiveForce;
    float K_att = 0.5;
    float K_rep = 0.5;
    float Distance=0;
    float DesiredYaw=0;
    float YawError=0;
    Desired_Position << 10,0,3;
    //norm --> Local_Position.norm()
    PositionError(0) = (Desired_Position(0) - LocalPosition.pose.position.x);
    PositionError(1) = (Desired_Position(1) - LocalPosition.pose.position.y);
    PositionError(2) = (Desired_Position(2) - LocalPosition.pose.position.z);
    Distance = sqrt(pow(PositionError(0),2)+pow(PositionError(1),2));
    AttractiveForce(0) = PositionError(0)/Distance;
    AttractiveForce(1) = PositionError(1)/Distance;
    AttractiveForce(2) = PositionError(2);
    
    DesiredYaw=atan2(PositionError(1),PositionError(0));
    YawError = DesiredYaw - yaw;
    cmd_vel.linear.x = AttractiveForce(0);
    cmd_vel.linear.y = AttractiveForce(1);
    cmd_vel.linear.z = AttractiveForce(2);
    cmd_vel.angular.z=0.2*YawError;
    
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_gap",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    //ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("obs_data", 20);
    ros::Subscriber distance_sub = nh.subscribe<sensor_msgs::LaserScan>
            ("/laser/scan",20,Lidar_callback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20, pose_callback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 20, imu_callback);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    ros::Publisher Velocity_Publisher = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped",20);
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    

    int count=0;

    // float F_att_x, F_att_y,F_att_z;
    // float F_rep_x, F_rep_y;

    // float desired_pos_x=10;
    // float desired_pos_y=1;
    // float desired_pos_z=6;

    // float obs_pos_x, obs_pos_y;
    
    geometry_msgs::PoseStamped pose;
    signal(SIGINT,handling);
    while(ros::ok()){
        Control_Robot();
        Velocity_Publisher.publish(cmd_vel);
        

        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
