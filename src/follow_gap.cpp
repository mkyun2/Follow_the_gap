#include <iostream>
#include <vector>

#include <tf/tf.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "sensor_msgs/LaserScan.h"
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
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
double roll, pitch, yaw = 0;
//===================Obstacle avoidance Variable===================
float DesiredPosition_x =20;
float DesiredPosition_y =0;
float DesiredPosition_z =2;
const int Max_distance=6000;
const int smallestDistanceSaturation=1500;
const double desired_eta = 100;
const int num_readings = 120;
const int laser_frequency = 20;
float ranges2[num_readings]={0.0};
float Final_Angle=0;
float DesiredYaw=0;
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
    vector<int> Obstacle_StartAngle;
    vector<int> Obstacle_EndAngle;
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

    // int Distance_of_Obstacle_start[20] = {0};
    // int Distance_of_Obstacle_end[20]   = {0};

    int Distance_of_Clearance[120]  = {6000};
    // int Clearance_point_start[120]  	= {0};
    // int Clearance_point_end[120]    	= {0};
    int Num_of_Clearance_point[120] 	= {0};
    int Distance_of_Clearance_start[120]= {0};
    int Distance_of_Clearance_end[120]	= {0};
    // flag를 이용해 Obstacles와 clearances를 구분하는 function.
    for(int i = 0; i<num_readings; i++){
        Distance_of_Obstacle[i]=6000;
        if((msg->ranges[i]*1000)<10)
            ranges2[i]=Max_distance;
        else if((msg->ranges[i]*1000)>Max_distance)
            ranges2[i]=Max_distance;
        else
            ranges2[i]=msg->ranges[i]*1000;
    }
    
    for(int i=0; i<num_readings; i++)
    {
        ranges2[i]=ranges2[i]-(int)ranges2[i]%10;
        
        //ROS_INFO("%f",ranges2[i]);
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
    Obstacle_flag=0;
    Obstacle_flag_pre=0;
    Clearance_flag=0;
    Clearance_flag_pre=0;	
//========================Obstacle,Clearance info =============================
    for(int i=0; i<num_readings; i++)
    {
        
        
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
            // ROS_INFO("Obstacle Start, %d", i);
            // ROS_INFO("Obstacle StartPoint: %d",i);
            if(i<119)
            Obstacle_StartAngle.push_back(i);
            //Obstacle_point_start[i] = i; // start point pos
            Distance_of_Obstacle[i]=ranges2[i]; // distance to start point
            Num_of_obstacle_point[i] = Num_of_obstacle_point[i] + 1; //

        }
        else if((Obstacle_flag == 1) && (Obstacle_flag_pre == 1))
        {
            // ROS_INFO("Obstacle ing");
            Num_of_obstacle_point[i] = Num_of_obstacle_point[i] + 1; // Obstacle을 구성하는 Points의 개수를 counting
            Distance_of_Obstacle[i] = ranges2[i];
            if(i > 118)
            {
                // ROS_INFO("Obstacle EndPoint: %d",i);
                //Obstacle_point_end[i] = i;// * 0.25 - 60;	// Obstacle Ending Point
                Obstacle_EndAngle.push_back(i);
                Distance_of_Obstacle[i] = ranges2[i];///////////////////////////////////////////////////////
                //i++;	// Obstacle_point : the number of obstacle points
            }
        }
        else if((Obstacle_flag == 0) && (Obstacle_flag_pre == 1))
        {
            Obstacle_EndAngle.push_back(i);
            // ROS_INFO("Obstacle End, %d",i);
            // ROS_INFO("Obstacle End Clearance Flag=%d, %d",Clearance_flag,Clearance_flag_pre);
            
            //Obstacle_point_end[i] = i;// * 0.25 - 60;	// Obstacle Ending Point
            Distance_of_Obstacle[i] = ranges2[i];///////////////////////////////////////////////////////
            //i++;
        }
        // ROS_INFO("Distance Obstacle, %d, %d, %f",Distance_of_Obstacle[i],i,ranges2[i]);
        // // else
        // {
        //     i = i;

        if((Clearance_flag == 1) && (Clearance_flag_pre == 0))
        {
            
            // ROS_INFO("Clearance Start, %d", i);
            // ROS_INFO("Clear startPoint: %d",i);
            //Clearance_point_start[i] = i;
            if(i<119)
            Clearance_StartAngle.push_back(i);
        }
        else if((Clearance_flag == 1) && (Clearance_flag_pre == 1))
        {
            // ROS_INFO("Clearance ing");
        //	Obstacle_Area[Num_of_obstacle][Num_of_obstacle_point[Num_of_obstacle]] = ds_data.ranges[i];
        //	Obstacle_Area[1][Num_of_obstacle][Num_of_obstacle_point[Num_of_obstacle]] = i;
            Num_of_Clearance_point[i] = Num_of_Clearance_point[i] + 1;
            if(i > 118)
            {
                // ROS_INFO("Clearance End1, %d, Ranges:%f", i,ranges2[i]);
                // ROS_INFO("Clear EndPoint: %d",i);
                Clearance_EndAngle.push_back(i);
                //Clearance_point_end[i] = i;// * 0.25 - 60;	// Obstacle Ending Point
                Distance_of_Clearance[i] = ranges2[i];

            }
        //	Num_of_obstacle_point[Num_of_obstacle] = Num_of_obstacle_point[Num_of_obstacle] + 1;
        }
        else if((Clearance_flag == 0) && (Clearance_flag_pre == 1))
        {
            // ROS_INFO("Clearance End2, %d", i);
            // //Clearance_point_end[i] = i;
            // ROS_INFO("Clear EndPoint: %d",i);
            Distance_of_Clearance[i] = ranges2[i];
            Clearance_EndAngle.push_back(i);

        }

        Obstacle_flag_pre = Obstacle_flag;
        Clearance_flag_pre = Clearance_flag;
    //Known Obstacle Num, Angle, distance, points
    // ROS_INFO("Clear: %d",Num_of_Clearance);
        
    }
    //start num --> Vector(angle num)
    //Angle of obstacle and cleanrance
    //Clearance num + Obstacle num 
    vector<int> GapAngle;
    vector<double> FinalAngle;
    vector<double> MinDistance;
    double Now_FinalAngle=0.0;
    double Max_FinalAngle=0.0;
    double Now_GapAngle=0;
    double Max_GapAngle = 0;
    int Max_GapAngle_Num=0;
    float Min_Distance=6000.0;
    float Now_Distance=0;
    float Gain_GapAngle = 10;
    float Gain_GoalAngle= 1;
    float GoalAngle=atan2f(DesiredPosition_y-LocalPosition.pose.position.y,DesiredPosition_x-LocalPosition.pose.position.x);//DesiredYaw;//atan2f(DesiredPosition_y,DesiredPosition_x);World 기준 Angle
    //Desired_Position(0) = Desired_Position(0)*cosf(Final_Angle)-Desired_Position(1)*sinf(Final_Angle);
    //Desired_Position(1) = Desired_Position(0)*sinf(Final_Angle)+Desired_Position(1)*cosf(Final_Angle);
    //float GoalAngle=DesiredYaw;//
    // if(GoalAngle>M_PI)
    //     GoalAngle=GoalAngle-(M_PI*2);
    // else if (GoalAngle<-M_PI)
    //     GoalAngle=GoalAngle+(M_PI*2);
    // else
    //     GoalAngle=GoalAngle;
    ROS_INFO("size: %d",Clearance_StartAngle.size());
    if(Clearance_StartAngle.size()>0)
    {
     for(int i=0; i<Clearance_StartAngle.size(); i++)
     {  
        ROS_INFO("Distance Error Check");
        ROS_INFO("Original Start Angle: %d, End Angle: %d",Clearance_StartAngle[i],Clearance_EndAngle[i]);
        ROS_INFO("Start Angle: %f, End Angle: %f",Clearance_StartAngle[i]-60.0+(yaw*180.0/M_PI),Clearance_EndAngle[i]-60.0+(yaw*180.0/M_PI));
        ROS_INFO("CHecking Clearance_EndDistance: %d, StartDistance: %d",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
            
        if(Distance_of_Obstacle[Clearance_EndAngle[i]] < Distance_of_Obstacle[Clearance_StartAngle[i]])
            {
                // if(Clearance_EndAngle[i]==num_readings-1)
                //         Min_Distance = ranges2[Clearance_EndAngle[i]];
                // else
                MinDistance.push_back(Distance_of_Obstacle[Clearance_EndAngle[i]]);
                //Min_Distance = Distance_of_Obstacle[Clearance_EndAngle[i]];
                //ROS_INFO("CHecking2 Clearance_EndAngle: %d, StartAngle: %d",Clearance_EndAngle[i],Clearance_StartAngle[i]);
                //ROS_INFO("CHecking2 Clearance_EndDistance: %d, StartDistance: %d",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
            
            }
        else
            {
                // if(Clearance_StartAngle[i]==0)
                //     Min_Distance = ranges2[Clearance_StartAngle[i]]; // how do robot when approach to zero 
                // else
                MinDistance.push_back(Distance_of_Obstacle[Clearance_StartAngle[i]]);
                //Min_Distance = Distance_of_Obstacle[Clearance_StartAngle[i]]; // how do robot when approach to zero 
                //ROS_INFO("CHecking1 Clearance_EndAngle: %d, StartAngle: %d",Clearance_EndAngle[i],Clearance_StartAngle[i]);
                //ROS_INFO("CHecking1 Clearance_EndDistance: %d, StartDistance: %d",Distance_of_Obstacle[Clearance_EndAngle[i]],Distance_of_Obstacle[Clearance_StartAngle[i]]);
            
            }

        //Gap_CenterAngle
        Now_GapAngle = (((Clearance_EndAngle[i]-60) - (Clearance_StartAngle[i]-60))/2);//+Clearance_StartAngle[i]-60;
        
        GapAngle.push_back(Now_GapAngle);

        
        ROS_INFO("Error Check");
        
        // FinalAngle.push_back(Now_FinalAngle);
        if(abs(Max_GapAngle)<abs(Now_GapAngle))
        {
           Max_GapAngle = Now_GapAngle;
           
        }
        ROS_INFO("Error Check2");
    }
    ROS_INFO("GapAngle size %d",GapAngle.size());
    if(find(GapAngle.begin(),GapAngle.end(), Max_GapAngle) != GapAngle.end())
        Max_GapAngle_Num = find(GapAngle.begin(), GapAngle.end(), Max_GapAngle) - GapAngle.begin();
    ROS_INFO("Error Check3");
    //must be modified !!!##################################
    if(Obstacle_StartAngle.size()>0)
    {
        for(int i=0; i<Obstacle_StartAngle.size(); i++)
        {
            ROS_INFO("Error Check4 size: %d",Obstacle_StartAngle.size());
            for(int j=Obstacle_StartAngle[i]; j<Obstacle_EndAngle[i]; j++)
            {
                //ROS_INFO("Error Check5 : %f", );
                if(Min_Distance>Distance_of_Obstacle[j])
                {
                    ROS_INFO("Error Check6");
                Min_Distance=Distance_of_Obstacle[j];
                ROS_INFO("Error Check7 angle: %d, dist:%f ",j,Min_Distance);
                }
            }
        }
    }
    //Min_Distance = MinDistance[Max_GapAngle_Num];
    if(Min_Distance<6000.0)
        Min_Distance=Min_Distance-1000.0;
    Min_Distance=(Min_Distance)/1000.0;
    ROS_INFO("Min_Distance: %f",Min_Distance);
    double HeadingAngle = (yaw*180.0/M_PI);
    // if(HeadingAngle>180.0)
    //     HeadingAngle=HeadingAngle-(180.0*2);
    // else if (HeadingAngle<-180.0)
    //     HeadingAngle=HeadingAngle+(180.0*2);
    // else
    //     HeadingAngle=HeadingAngle;
    Max_GapAngle = GapAngle[Max_GapAngle_Num]+(Clearance_StartAngle[Max_GapAngle_Num]-60+HeadingAngle);
    Max_GapAngle = Max_GapAngle*M_PI/180.0;
    Final_Angle = ((Gain_GapAngle/Min_Distance*Max_GapAngle)+(Gain_GoalAngle*GoalAngle)) / ((Gain_GapAngle/Min_Distance)+Gain_GoalAngle);
        
    // if(find(FinalAngle.begin(),FinalAngle.end(), Max_FinalAngle) != FinalAngle.end())
    //     Max_GapAngle_Num = find(FinalAngle.begin(), FinalAngle.end(), Max_FinalAngle) - FinalAngle.begin();
    // // //}or SORT -> 0th index 
    //Final_Angle = FinalAngle[Max_GapAngle_Num];
    ROS_INFO("Gap Angle: %f, Min_Distance: %f",Max_GapAngle*180.0/M_PI,Min_Distance);
    ROS_INFO("GoalAngle: %f Final_Angle: %f Heading: %f", GoalAngle*180.0/M_PI,Final_Angle*180.0/M_PI, yaw*180.0/M_PI);
    ROS_INFO("LocalPosition x:%f y:%f z:%f ",LocalPosition.pose.position.x,LocalPosition.pose.position.y,LocalPosition.pose.position.z);
    //Known GapAngle Num, Max_GapAngle, obstacle distance
    }

    //ranges2[Clearance_StartAngle[Max_GapAngle_Num]]

}
void Control_Robot()//차이 알아보기
{
    Eigen::Vector3f Local_Position;;
    Eigen::Vector3f Desired_Position;
    Eigen::Vector3f PositionError;
    Eigen::Vector3f AttractiveForce;
    Eigen::Vector3f RuepulsiveForce;
    float K_att = 0.1;
    float K_rep = 0.01;
    float Distance=0;
    
    float YawError=0;
    Desired_Position << DesiredPosition_x,DesiredPosition_y,DesiredPosition_z;
    Local_Position << LocalPosition.pose.position.x,LocalPosition.pose.position.y,LocalPosition.pose.position.z;
    //norm --> Local_Position.norm()
    Final_Angle = Final_Angle;
    YawError = (Final_Angle - yaw);
    //Desired_Position(0) = Desired_Position(0)*cosf(Final_Angle)-Desired_Position(1)*sinf(Final_Angle);
    //Desired_Position(1) = Desired_Position(0)*sinf(Final_Angle)+Desired_Position(1)*cosf(Final_Angle);
    PositionError(0) = (Desired_Position(0) - LocalPosition.pose.position.x);
    PositionError(1) = (Desired_Position(1) - LocalPosition.pose.position.y);
    PositionError(2) = (Desired_Position(2) - LocalPosition.pose.position.z);
    DesiredYaw=atan2f(PositionError(1),PositionError(0));
    Distance = sqrt(pow(PositionError(0),2)+pow(PositionError(1),2));
    AttractiveForce(0) = (K_att+0.1)*PositionError(0)/Distance;
    AttractiveForce(1) = K_att*PositionError(1)/Distance;
    AttractiveForce(2) = 0.5*PositionError(2);
    //DesiredYaw = (DesiredYaw-yaw);

    ROS_INFO("Position Error: %f, %f",PositionError(0),PositionError(1));
    //ROS_INFO("LocalPosition x:%f y:%f z:%f ",LocalPosition.pose.position.x,LocalPosition.pose.position.y,LocalPosition.pose.position.z);
    if(PositionError(2)<0.3){
    cmd_vel.linear.x = AttractiveForce(0)*cosf(Final_Angle)-AttractiveForce(1)*sinf(Final_Angle);
    cmd_vel.linear.y = AttractiveForce(0)*sinf(Final_Angle)+AttractiveForce(1)*cosf(Final_Angle);
    //cmd_vel.linear.x = AttractiveForce(0);
    //cmd_vel.linear.y = AttractiveForce(1);
    }
    cmd_vel.linear.z = AttractiveForce(2);
    cmd_vel.angular.z=0.5*YawError;
    
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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok()){
        Control_Robot();
        Velocity_Publisher.publish(cmd_vel);
        

        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
