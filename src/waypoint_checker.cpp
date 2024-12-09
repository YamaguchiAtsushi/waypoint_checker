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
    WaypointChecker(){  // プライベートノードハンドルを初期化
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        waypoint_checker_pub_ = nh_.advertise<geometry_msgs::Int16>("waypoint_checker", 1);

        waypoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("waypoint", 1, &WaypointChecker::waypointCallback, this);
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan", 1, &WaypointChecker::scanCallback, this);

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_odom_x_ = 0.0;
        robot_odom_y_ = 0.0;
        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;

        sum_x = 0.0;
        sum_y = 0.0;
        center_x = 0.0;
        center_y = 0.0;

        waypoint_checker_msg_.data = 0;

        // パラメータの取得
        pnh_.getParam("current_location", current_location_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;  // プライベートノードハンドル

    ros::Subscriber waypoint_sub_, cluster_sub_;
    ros::Publisher marker_pub_, waypoint_checker_pub_;
    ros::Timer timer_callback_;
    ros::Time timer_start_;
    ros::Time timer_now_;
    std_msgs::Int16 waypoint_checker_msg_;
    geometry_msgs::PoseStamped waypoint_;
    geometry_msgs::Point p_;
    geometry_msgs::Quaternion robot_r_;
    sensor_msgs::LaserScan::ConstPtr scan_;

    double sum_x, sum_y;
    double center_x, center_y;
 

    void timerCallback(const ros::TimerEvent&);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
};

// タイマコールバックの実装（内容は省略）
void WaypointChecker::timerCallback(const ros::TimerEvent&) {
    // タイマ処理のロジックを記述
    for (size_t i = 0; i < clusters.size(); ++i){
        for (size_t j = 0; j < clusters[i].size(); ++j){
            p_.x = clusters[i][j].x;
            p_.y = clusters[i][j].y;
            sum_x += p_.x;
            sum_y += p_.y;
            center_x = sum_x / clusters[i].size();//tfの変換が必要ーーーーーーーーーーーーーーーーーーーーーー
            center_y = sum_y / clusters[i].size();//CMakeLists.txtを復活させる
            if((waypoint_.pose.position.x - 0.25 < center_x && center_x < waypoint_.pose.position.x + 0.25) && (waypoint_.pose.position.y - 0.25 < center_y && center_y < waypoint_.pose.position.y + 0.25)){
                waypoint_checker_msg.data = 1;
            }
            else{
                waypoint_checker_msg.data = 0;
            }
            waypoint_checker_pub_.publish(waypoint_num_msg_);

        }
    }
    
}


void WaypointChecker::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    waypoint_.pose.position.x = msg->pose.position.x;
    waypoint_.pose.position.y = msg->pose.position.y;
    waypoint_.pose.position.z = msg->pose.position.z;
}

void WaypointChecker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = msg;
    std::vector<std::vector<geometry_msgs::Point>> clusters;
    std::vector<geometry_msgs::Point> current_cluster;

    double distance_threshold = 0.1;

    for (size_t i = 0; i < scan_->ranges.size(); ++i)
    {
        if (std::isfinite(scan_->ranges[i]))
        {
            geometry_msgs::Point p;
            double angle = scan_->angle_min + i * scan_->angle_increment;
            double point_x = scan_->ranges[i] * cos(angle);
            double point_y = scan_->ranges[i] * sin(angle);

            if((min_x < point_x && point_x < max_x) && (min_y < point_y && point_y < max_y)){
                p.x = point_x;
                p.y = point_y;
            }

            if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
            {
                current_cluster.push_back(p);
            }
            else
            {
                if (!current_cluster.empty())
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
                current_cluster.push_back(p);
            }
        }
    }

    if (!current_cluster.empty())
    {
        clusters.push_back(current_cluster);
    }

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
