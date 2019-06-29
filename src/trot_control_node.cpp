#include<ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;
sensor_msgs::JointState msg;
bool isForward = true;

std::vector<double> forward_step = {  0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1,
                                        0.0, 0.0, 0.1,
                                        0.0, 0.0, 0.0  };
std::vector<double> reverse_step = {    0.0, 0.0, 0.1,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1   };

void trot_control(){
    msg.header.stamp = ros::Time::now();

    if(isForward){
        msg.position = forward_step;
    }else{
        msg.position = reverse_step;
    }

    pub.publish(msg);
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trot_control");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::JointState>("legged_robot/cmd_ang", 1);
    
    ROS_INFO("launch trot_control_node!");
    double frequency = 5;
    ros::Rate r(frequency);
    
    while(ros::ok()){
        trot_control();
        ros::spinOnce();
        r.sleep();
        isForward = !isForward;
    }
}
