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

#include <iostream>
#include "pepper_dcm_driver/pepper.h"
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

//#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

using std::string;
using std::cerr;
using std::endl;

int main( int argc, char** argv )
{
    int pport = 9559;
    string pip = "127.0.0.1";
    ros::init(argc, argv, "nao_dcm_driver");
    ros::NodeHandle n;
    ros::NodeHandle n_p("~");
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }

    // load urdf model from param server
    std::vector< std::string > joint_names;
    try{
      std::string robot_description;
      n.getParam("robot_description", robot_description);
      ROS_INFO_STREAM("robot description loaded !!");

      KDL::Tree tree;
      kdl_parser::treeFromString(robot_description, tree);
      KDL::SegmentMap segment_map = tree.getSegments();

      typedef KDL::SegmentMap::const_iterator kdl_iter;
      for (kdl_iter it = segment_map.begin(); it != segment_map.end(); ++it)
      {
        // filter out non-actuated joints
        if (it->second.segment.getJoint().getType() != KDL::Joint::None)
        {
          ROS_INFO_STREAM("joint found: " << it->second.segment.getJoint().getName()
                          << it->second.segment.getJoint().getType()) ;
          joint_names.push_back(it->second.segment.getJoint().getName());
          }
      }
    }
    catch (std::exception e)
    {
      ROS_ERROR_STREAM("no robot descption found ... exiting! ");
      ROS_ERROR_STREAM(e.what());
      return -1;
    }

    // A broker needs a name, an IP and a port:
    string broker_name = "Nao Driver Broker";
    // FIXME: would be a good idea to look for a free port first
    int broker_port = 54000;
    // listen port of the broker (here an anything)
    string broker_ip = "0.0.0.0";

    // Load Params from Parameter Server

    n_p.param("RobotIP", pip, string("127.0.0.1"));
    n_p.param("RobotPort", pport,9559);
    n_p.param("DriverBrokerPort", broker_port, 54000);
    n_p.param("DriverBrokerIP", broker_ip, string("0.0.0.0"));

    // Create your own broker
    boost::shared_ptr<AL::ALBroker> broker;
    try
    {
        broker = AL::ALBroker::createBroker(broker_name,broker_ip,broker_port,pip,pport,0);
    }
    catch(...)
    {
        ROS_ERROR("Failed to connect to Broker at %s:%d!",pip.c_str(),pport);
        return -1;
    }

    // Deal with ALBrokerManager singleton (add your broker into NAOqi)
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);


    // Run the spinner in a separate thread to prevent lockups
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Now it's time to load your module
    boost::shared_ptr<Nao> nao = AL::ALModule::createModule<Nao>(broker, "Nao" );
    nao->connect(n);
    if(!nao->connected())
    {
        ROS_ERROR("Could not connect to Nao robot!");
        AL::ALBrokerManager::getInstance()->killAllBroker();
        AL::ALBrokerManager::kill();
        return -1;
    }

    nao->setStiffness( 1.0f );

    if(broker->isModulePresent("Nao"))
        ROS_INFO("Nao Module loaded succesfully!");
    else
    {
        ROS_ERROR("Nao Module is not loaded!");
        return -1;
    }

    nao->run();

    nao->setStiffness ( 0.0f );
    spinner.stop();

    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();

//    spinner.stop();
    ROS_INFO( "Quitting... " );
    return 0;
}
