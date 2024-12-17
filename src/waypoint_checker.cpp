#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <cmath>
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

struct Person
{
    std::vector<geometry_msgs::Point> points;  // 検出された点の集まり
    double average_distance;                     // 平均距離
    double length;                               // バウンディングボックスの長さ
    double aspect_ratio;                         // アスペクト比
    geometry_msgs::Point min_point;             // バウンディングボックスの最小点
    geometry_msgs::Point max_point;             // バウンディングボックスの最大点
    bool is_matched = false;
    bool is_matched_track = false;//追加11/1
    int id;
    int lost_num = 0;
};



class WaypointChecker {
public:
    WaypointChecker() : tfBuffer(), tfListener(tfBuffer) {
        waypoint_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
        waypoint_checker_pub_ = nh_.advertise<std_msgs::Int16>("waypoint_checker", 1);
        waypoint_skip_flag_pub_ = nh_.advertise<std_msgs::Int16>("waypoint_skip_flag", 1);

        waypoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("waypoint", 1, &WaypointChecker::waypointCallback, this);
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &WaypointChecker::scanCallback, this);
        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 10, &WaypointChecker::odomCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &WaypointChecker::amclPoseCallback, this);

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        is_timer_start_ = false;
        is_stay_obstacle_ = false;

        waypoint_checker_msg_.data = 0;
        waypoint_skip_flag_msg_.data = 0;
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber waypoint_sub_, scan_sub_, odom_sub_, amcl_sub_;
    ros::Publisher waypoint_marker_pub_, waypoint_checker_pub_, waypoint_skip_flag_pub_;
    std_msgs::Int16 waypoint_checker_msg_;
    std_msgs::Int16 waypoint_skip_flag_msg_;
    geometry_msgs::PoseStamped waypoint_;
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;

    tf2_ros::Buffer tfBuffer;                
    tf2_ros::TransformListener tfListener;

    geometry_msgs::PointStamped pointBaseLink;
    geometry_msgs::PointStamped pointMap;

    geometry_msgs::TransformStamped transformStampedScan;


    std::vector<std::vector<geometry_msgs::Point>> clusters_;

    ros::Time start_;
    ros::Time now_;

    bool is_timer_start_;
    bool is_stay_obstacle_;


    double robot_x_, robot_y_;
    double robot_odom_x_, robot_odom_y_;

    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
};

double WaypointChecker::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void WaypointChecker::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    waypoint_.pose.position.x = msg->pose.position.x;
    waypoint_.pose.position.y = msg->pose.position.y;
    waypoint_.pose.position.z = msg->pose.position.z;
}

void WaypointChecker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = msg;
    clusters_.clear();



    std::vector<geometry_msgs::Point> current_cluster;
    double distance_threshold = 0.1;

    try {
        transformStampedScan = tfBuffer.lookupTransform("map", "laser", ros::Time(0), ros::Duration(1.0));

        for (size_t i = 0; i < scan_->ranges.size(); ++i)
        {
            if (std::isfinite(scan_->ranges[i]))
            {
                // geometry_msgs::Point p;
                // double angle = scan_->angle_min + i * scan_->angle_increment;
                // double point_x = scan_->ranges[i] * cos(angle);
                // double point_y = scan_->ranges[i] * sin(angle);

                pointBaseLink.header.frame_id = "laser";
                pointBaseLink.header.stamp = ros::Time::now();

                double angle = scan_->angle_min + i * scan_->angle_increment;
                double x_base_link = scan_->ranges[i] * cos(angle);
                double y_base_link = scan_->ranges[i] * sin(angle);

                pointMap.header.frame_id = "map";
                pointMap.header.stamp = ros::Time::now();

                pointBaseLink.point.x = x_base_link;
                pointBaseLink.point.y = y_base_link;
                pointBaseLink.point.z = 0.0;

                tf2::doTransform(pointBaseLink, pointMap, transformStampedScan);


                geometry_msgs::Point p;

                double point_x = pointMap.point.x;
                double point_y = pointMap.point.y;


                p.x = point_x;
                p.y = point_y;

                if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
                {
                    current_cluster.push_back(p);
                }
                else
                {
                    if (!current_cluster.empty())
                    {
                        clusters_.push_back(current_cluster);
                        current_cluster.clear();
                    }
                    current_cluster.push_back(p);
                }
            }
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform failed: %s", ex.what());

    }

    if (!current_cluster.empty())
    {
        clusters_.push_back(current_cluster);
    }

    // クラスタとwaypointの判定
    waypoint_skip_flag_msg_.data = 0;

    is_stay_obstacle_ = false;

    for (const auto& cluster : clusters_)
    {
        // is_stay_obstacle_ = false;
        for (const auto& point : cluster)
        {
            
            //変更後
            if(distance(point, waypoint_.pose.position) <= 0.5){
                is_stay_obstacle_ = true;

                if(is_timer_start_ == false){
                    start_ = ros::Time::now();
                    is_timer_start_ = true;
                    // is_stay_obstacle_ = true;
                }

                // std::cout << "point" << point.x << "," << point.y << std::endl;

                std::cout << "distance" << distance(point, waypoint_.pose.position) << std::endl;

                // waypoint_skip_flag_msg_.data = 1;
                std::cout << "is_timer_start_" << is_timer_start_ << std::endl;



                ROS_INFO("TIMER START: %f", (now_ - start_).toSec());
                // ROS_INFO("NOW WAYPOINT: (%f, %f)", waypoint_.pose.position.x, waypoint_.pose.position.y);

                now_ = ros::Time::now();
                if (now_ - start_ > ros::Duration(5.0))
                {
                    ROS_INFO("3sec PASSED");
                    waypoint_skip_flag_msg_.data = 1;

                    ROS_INFO("WAYPOINT SKIP FLAG:%d", waypoint_skip_flag_msg_.data);
                    

                }
                break;
            }


            //変更前
            // if (distance(point, waypoint_.pose.position) <= 0.5)
            // {
            //     waypoint_skip_flag_msg_.data = 1;
            //     break;
            // }


        }
        if (waypoint_skip_flag_msg_.data == 1)
        {
            break;
        }
    }
    if(is_stay_obstacle_ == false){
        std::cout << "is_timer_start1" << is_timer_start_ << std::endl;
        is_timer_start_ = false;
        std::cout << "is_timer_start2" << is_timer_start_ << std::endl;

    }
    waypoint_skip_flag_pub_.publish(waypoint_skip_flag_msg_);
}

void WaypointChecker::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    robot_odom_x_ = odom->pose.pose.position.x;
    robot_odom_y_ = odom->pose.pose.position.y;
}

void WaypointChecker::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_r_ = msg->pose.pose.orientation;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_checker");

    WaypointChecker wc;

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
