#ifndef LEGGEDROBOT_PLUGIN_HH
#define LEGGEDROBOT_PLUGIN_HH

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define LINK_NUM 3  // 全リンク数 (total number of links)
#define JT_NUM   3  // 全ジョイント数 (total number of joints)                                           
#define LEG_NUM  4  // 全脚数 (total number of legs) 

namespace gazebo{
    
    class GazeboRosLeggedRobot : public ModelPlugin{
        public:
            GazeboRosLeggedRobot();
            ~GazeboRosLeggedRobot();

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Reset();
            double angle1, angle2, angle3;
            int posture = 2; // 姿勢(posture)

            protected:
                virtual void UpdateChild();
                virtual void FiniChild();

            private:
                void publishHingeTF();
                void publishJointState();
                GazeboRosPtr gazebo_ros_;
                physics::ModelPtr parent;
                event::ConnectionPtr update_connection_;

                int hinge_num_;
                double gain_;
                double hinge_torque_;
                std::string robot_namespace_;
                std::string command_topic_;
                std::string hinge_angle_topic_;
                std::string robot_base_frame_;

                // Update Rate
                double update_rate_;
                double update_period_;
                common::Time last_update_time_;

                // Flags
                bool publishHingeTF_;
                //bool publishOdomTF_;
                bool publishJointState_;
                bool isCmdReceived_;
                bool alive_;

                // ROS STUFF
                ros::Publisher joint_state_publisher_;
                ros::Subscriber cmd_vel_subscriber_;
                sensor_msgs::JointState hinge_msg_;
                sensor_msgs::JointState joint_state_;
                boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

                //callback stuff
                void cmdVelCallback(const sensor_msgs::JointState::ConstPtr& cmd_msg);

                // Custom Callback Queue
                ros::CallbackQueue queue_;
                boost::thread callback_queue_thread_;
                void QueueThread();

                // モデルへのポインタ
                physics::ModelPtr model;
                physics::WorldPtr world;

                // ワールド状態のサブスクライブ(講読)
                transport::NodePtr node;
                transport::SubscriberPtr statsSub;
                common::Time simTime;

                // 更新イベントコネクションへのポインタ
                std::vector<physics::JointPtr> joints_;

                double THETA[LEG_NUM*LINK_NUM];    // 関節目標角度 
                double gait[12][LEG_NUM][JT_NUM] ;  // 目標角度(target angle of gait)                           
                double l1, l2, l3;  // 脚長 (lenth of links)   
                double r1, r2, r3;  // 脚半径(leg radius)
    };
}//namespace

#endif