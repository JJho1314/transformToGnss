#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class OdomConverter
{
public:
    OdomConverter(const ros::NodeHandle &nh) : nh_(nh)
    {
        nh.param<std::string>("common/odomTopic", odomTopic, "/Odometry");       // gnss
        nh.param<std::string>("common/gpsTopic", gpsTopic, "/gps/correct_odom"); // gnss
        nh.param<std::vector<double>>("common/initialPose", initialPose, std::vector<double>());
        nh.param<std::string>("common/intialMethod", intialMethod, "gps");
        // 订阅原始里程计话题
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odomTopic, 10, &OdomConverter::odomCallback, this);
        gps_sub_ = nh_.subscribe<nav_msgs::Odometry>(gpsTopic, 10, &OdomConverter::gpsHandler, this);

        // 发布转换后的里程计话题
        custom_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/custom_odom", 10);
        custom_path_pub_ = nh_.advertise<nav_msgs::Path>("/custom_path", 10);

        if (intialMethod == "human")
            mintialMethod = human;
        else if (intialMethod == "gps")
            mintialMethod = gps;
        else
        {
            std::cout << "Undefined intialMethod type " << std::endl;
            exit(-1);
        }
        if (mintialMethod == human)
            gps_initailized = true;

        // 初始化初始位置和姿态
        initial_pose_.position.x = initialPose.at(0);
        initial_pose_.position.y = initialPose.at(1);
        initial_pose_.position.z = initialPose.at(2);
        initial_pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(initialPose.at(3), initialPose.at(4), initialPose.at(5)); // 设置初始姿态角为0
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        if (gps_initailized = true)
        {
            // 计算转换后的里程计
            nav_msgs::Odometry custom_odom = *odom_msg;
            geometry_msgs::PoseStamped msg_custom_pose;
            custom_odom.header.stamp = odom_msg->header.stamp;
            custom_odom.header.frame_id = "camera_init";

            // 转换位置
            custom_odom.pose.pose.position.x -= initial_pose_.position.x;
            custom_odom.pose.pose.position.y -= initial_pose_.position.y;
            custom_odom.pose.pose.position.z -= initial_pose_.position.z;

            // 转换姿态角
            tf::Quaternion initial_quat, odom_quat, custom_quat;
            tf::quaternionMsgToTF(initial_pose_.orientation, initial_quat);
            tf::quaternionMsgToTF(custom_odom.pose.pose.orientation, odom_quat);
            tf::Quaternion initial_quat_inv = initial_quat.inverse();
            custom_quat = initial_quat_inv * odom_quat;
            custom_quat.normalize();
            tf::quaternionTFToMsg(custom_quat, custom_odom.pose.pose.orientation);
            std::cout << "custom pose is " << custom_odom.pose.pose.position.x << ", " << custom_odom.pose.pose.position.y << ", " << custom_odom.pose.pose.position.z << std::endl;

            // 发布转换后的里程计
            custom_odom_pub_.publish(custom_odom);

            // 构建并发布路径
            path.header = custom_odom.header;
            geometry_msgs::PoseStamped pose;
            pose.header = custom_odom.header;
            pose.pose = custom_odom.pose.pose;
            path.poses.push_back(pose);
            custom_path_pub_.publish(path);
        }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {
        if (mintialMethod == gps)
        {
            if (!gps_initailized && (gpsMsg->pose.pose.position.x != 0 || gpsMsg->pose.pose.position.y != 0) &&
                (gpsMsg->pose.covariance[0] < 0.003 && gpsMsg->pose.covariance[7] < 0.003))
            {
                double imu_roll, imu_pitch, imu_yaw;
                Eigen::Vector3d Pwi(gpsMsg->pose.pose.position.x, gpsMsg->pose.pose.position.y, gpsMsg->pose.pose.position.z);
                tf::Quaternion imu_quat(gpsMsg->pose.pose.orientation.x, gpsMsg->pose.pose.orientation.y,
                                        gpsMsg->pose.pose.orientation.z, gpsMsg->pose.pose.orientation.w);
                tf::Matrix3x3(imu_quat).getRPY(imu_roll, imu_pitch, imu_yaw); // 进行转换
                std::cout << "GPS initailizes" << std::endl;
                initialPose.at(0) = Pwi.x();
                initialPose.at(1) = Pwi.y();
                initialPose.at(2) = Pwi.z();
                initialPose.at(3) = imu_roll;
                initialPose.at(4) = imu_pitch;
                initialPose.at(5) = imu_yaw;
                std::cout << "initial pose is" << initialPose[0] << "," << initialPose[1] << "," << initialPose[2] << std::endl;
                gps_initailized = true;
            }
        }

        // cout<<"收到GPS"<<endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher custom_odom_pub_;
    ros::Publisher custom_path_pub_;
    geometry_msgs::Pose initial_pose_;
    std::vector<double> initialPose = {0, 0, 0, 0, 0, 0};
    enum eintialMethod
    {
        human = 0,
        gps = 1,
    } mintialMethod;
    std::string intialMethod;
    bool gps_initailized = false;
    std::string gpsTopic;
    std::string odomTopic;
    nav_msgs::Path path;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_converter");
    ros::NodeHandle nh;

    OdomConverter odom_converter(nh);

    ros::spin();

    return 0;
}
