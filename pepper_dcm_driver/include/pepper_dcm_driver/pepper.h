/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef PEPPER_DCM_DRIVER_PEPPER_H
#define PEPPER_DCM_DRIVER_PEPPER_H

// Boost Headers
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>

// NAOqi Headers
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include <qi/os.hpp>

// ROS Headers
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <pepper_dcm_msgs/BoolService.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

using std::string;
using std::vector;

namespace AL
{
class ALBroker;
}

// Helper definition
template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

class Nao : public hardware_interface::RobotHW
{
private:
    // ROS Standard Variables
    ros::NodeHandle node_handle_;

    // ROS Topics/Messages
    ros::Subscriber cmd_vel_sub_;

    controller_manager::ControllerManager* manager_;

    // Member Variables
    AL::ALValue commands_;

    // Helper
    bool is_connected_;

    // Robot Parameters
    string version_, body_type_;
    int topic_queue_;
    string prefix_, odom_frame_;
    double controller_freq_, joint_precision_;

    // AL Proxies
    AL::ALMemoryProxy memory_proxy_;
    AL::DCMProxy dcm_proxy_;

    // Joints
    vector<string> joints_names_;
    // Joint States
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    int number_of_joints_;
    vector<string> joint_names_;
    vector<double> joint_commands_;
    vector<double> joint_angles_;
    vector<double> joint_velocities_;
    vector<double> joint_efforts_;
public:
    // Constructor/Destructor
    Nao( const std::vector< std::string> &joint_names );
    ~Nao();

    bool initialize();
    bool initializeControllers(controller_manager::ControllerManager& cm);

    // Connect/Disconnet to ALProxies
    bool connect(const boost::shared_ptr<AL::ALBroker> &broker, const ros::NodeHandle &nh);
    void disconnect();

    // Parameter Server
    void loadParams();

    // Helper
    void brokerDisconnected(const string& event_name, const string &broker_name, const string& subscriber_identifier);

    // DCMProxy Wrapper Methods
//    void DCMTimedCommand(const string& key, const AL::ALValue& value, const int& timeOffset,
//                         const string& type="Merge");
    void DCMAliasTimedCommand(const string& alias, const vector<float>& values, const vector<int>& timeOffsets,
                              const string& type="Merge", const string& type2="time-mixed");

    // General Methods
    void controllerLoop();
    bool connected();

    void readJoints();

    void writeJoints();

    bool setStiffness(float stiffness);

    void run();

};

#endif // PEPPER_DCM_DRIVER_PEPPER_H
