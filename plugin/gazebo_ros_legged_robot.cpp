#include <legged_robot/gazebo_ros_legged_robot.h>

namespace gazebo{

    GazeboRosLeggedRobot::GazeboRosLeggedRobot(){
        isCmdReceived_ = false;
    }

    GazeboRosLeggedRobot::~GazeboRosLeggedRobot(){

    }

    void GazeboRosLeggedRobot::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
        this->parent = _parent;
        gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "LeggedRobot" ) );

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameterBoolean ( isAngleControl_, "angleControl", true);
        gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
        gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
        gazebo_ros_->getParameterBoolean ( publishHingeTF_, "publishHingeTF", true );
        gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", true );
        gazebo_ros_->getParameter<double> ( hinge_torque_, "hingeTorque", 5.0 );
        gazebo_ros_->getParameter<double> ( gain_, "gain", 0.1 );
        gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );

        char hinge_list[LEG_NUM*LINK_NUM][10] = {"hinge_lf1", "hinge_lf2", "hinge_lf3",
                                                "hinge_lr1", "hinge_lr2", "hinge_lr3",
                                                "hinge_rf1", "hinge_rf2", "hinge_rf3",
                                                "hinge_rr1", "hinge_rr2", "hinge_rr3"};
        char default_hinge_name[LEG_NUM*LINK_NUM][20] = {"hinge_left_front_1", "hinge_left_front_2", "hinge_left_front_3",
                                                        "hinge_left_rear_1", "hinge_left_rear_2", "hinge_left_rear_3",
                                                        "hinge_right_front_1", "hinge_right_front_2", "hinge_right_front_3",
                                                        "hinge_right_rear_1", "hinge_right_rear_2", "hinge_right_rear_3"};
        joints_.resize (LEG_NUM*LINK_NUM);
        for(int i=0;i<LEG_NUM*LINK_NUM;i++){
            joints_[i] = gazebo_ros_->getJoint ( parent, hinge_list[i], default_hinge_name[i]);
        }

        for ( int i = 0; i < joints_.size(); i++ ) {
            joints_[i]->SetParam ( "fmax", 0, hinge_torque_ );
            joints_[i]->SetPosition(0, 0.0);
            //joints_[i]>SetForce(0, hinge_torque_);
        }

        if (this->publishJointState_){
            joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
            ROS_INFO_NAMED("ros_legged_robot", "%s: Advertise joint_states", gazebo_ros_->info());
        }

        transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ROS_INFO_NAMED("ros_legged_robot", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

        if(isAngleControl_){
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<sensor_msgs::JointState>(command_topic_, 1,
                    boost::bind(&GazeboRosLeggedRobot::cmdAngCallback, this, _1),
                    ros::VoidPtr(), &queue_);
            cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
            ROS_INFO_NAMED("ros_legged_robot", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
        }else{//velControl
            ros::SubscribeOptions so =
            ros::SubscribeOptions::create<sensor_msgs::JointState>(command_topic_, 1,
                    boost::bind(&GazeboRosLeggedRobot::cmdVelCallback, this, _1),
                    ros::VoidPtr(), &queue_);
            cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
            ROS_INFO_NAMED("ros_legged_robot", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
        }

        alive_ = true;

        // Initialize update rate stuff
        if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        
        #if GAZEBO_MAJOR_VERSION >= 8
            last_update_time_ = parent->GetWorld()->SimTime();
        #else
            last_update_time_ = parent->GetWorld()->GetSimTime();
        #endif
        
        this->callback_queue_thread_ =
                boost::thread ( boost::bind ( &GazeboRosLeggedRobot::QueueThread, this ) );

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosLeggedRobot::UpdateChild, this ) );
    }

    void GazeboRosLeggedRobot::Reset(){
        #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent->GetWorld()->SimTime();
        #else
        last_update_time_ = parent->GetWorld()->GetSimTime();
        #endif

        for ( int i = 0; i < joints_.size(); i++ ) {
            joints_[i]->SetParam ( "fmax", 0, hinge_torque_ );
        }
    }

    void GazeboRosLeggedRobot::publishJointState(){
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize ( joints_.size() );
        joint_state_.position.resize ( joints_.size());

        for ( int i = 0; i < joints_.size(); i++ ) {
            physics::JointPtr joint = joints_[i];

            #if GAZEBO_MAJOR_VERSION >= 8
                double position = joint->Position ( 0 );
            #else
                double position = joint->Position ( 0 );//GetAngle ( 0 ).Radian();
            #endif

            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = position;
            //joint_state_.velocity[i] = joint->GetVelocity(0);
            //joint_state_.effort[i] = joint->GetForce(0);
        }
        joint_state_publisher_.publish ( joint_state_ );
    }

    void GazeboRosLeggedRobot::publishHingeTF(){
        ros::Time current_time = ros::Time::now();
        for ( int i = 0; i < joints_.size(); i++ ) {

            std::string hinge_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
            std::string hinge_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

            #if GAZEBO_MAJOR_VERSION >= 8
                ignition::math::Pose3d poseHinge = joints_[i]->GetChild()->RelativePose();
            #else
                ignition::math::Pose3d poseHinge = joints_[i]->GetChild()->GetRelativePose().Ign();
            #endif

            tf::Quaternion qt ( poseHinge.Rot().X(), poseHinge.Rot().Y(), poseHinge.Rot().Z(), poseHinge.Rot().W() );
            tf::Vector3 vt ( poseHinge.Pos().X(), poseHinge.Pos().Y(), poseHinge.Pos().Z() );

            tf::Transform tfHinge ( qt, vt );
            transform_broadcaster_->sendTransform (
                tf::StampedTransform ( tfHinge, current_time, hinge_parent_frame, hinge_frame ) );
        }
    }

    // Update the controller
    void GazeboRosLeggedRobot::UpdateChild()
    {
        for ( int i = 0; i < joints_.size(); i++ ) {
            if ( fabs(hinge_torque_ -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
                joints_[i]->SetParam ( "fmax", 0, hinge_torque_ );
            }
        }

        #if GAZEBO_MAJOR_VERSION >= 8
            common::Time current_time = parent->GetWorld()->SimTime();
        #else
            common::Time current_time = parent->GetWorld()->GetSimTime();
        #endif
            double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

        if ( seconds_since_last_update > update_period_ ) {
            //if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
            if ( publishHingeTF_ ) publishHingeTF();
            if ( publishJointState_ ) publishJointState();

            // Update robot in case new velocities have been requested
            //gethingeVelocities();

            if(isAngleControl_ && isCmdReceived_){
                for ( int i = 0; i < joints_.size(); i++ ) {
                    if(hinge_msg_.velocity.size() != joints_.size())
                        joints_[i]->SetParam ( "vel", 0, 0.0);
                    else
                        joints_[i]->SetParam ( "vel", 0, hinge_msg_.velocity[i]);
                }
            }

            last_update_time_+= common::Time ( update_period_ );
        }
    }

    // Finalize the controller
    void GazeboRosLeggedRobot::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void GazeboRosLeggedRobot::cmdVelCallback ( const sensor_msgs::JointState::ConstPtr& cmd_msg ){

        //boost::mutex::scoped_lock scoped_lock ( lock );
        //x_ = cmd_msg->linear.x;
        //rot_ = cmd_msg->angular.z;
        hinge_msg_ = *cmd_msg;
        isCmdReceived_ = true;
    }

    void GazeboRosLeggedRobot::cmdAngCallback ( const sensor_msgs::JointState::ConstPtr& cmd_msg ){

        if(cmd_msg->position.size() == joints_.size()){
            for(int i=0;i<joints_.size();i++){
                joints_[i]->SetPosition(0, cmd_msg->position[i]);
            }
        }
    }


    void GazeboRosLeggedRobot::QueueThread(){
        static const double timeout = 0.01;

        while ( alive_ && gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosLeggedRobot)
}//namespace