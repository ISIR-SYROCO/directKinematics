#ifndef OROCOS_DIRECTKINEMATICS_COMPONENT_HPP
#define OROCOS_DIRECTKINEMATICS_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

#include <sensor_msgs/typekit/Types.hpp>
#include <sensor_msgs/JointState.h>

#include <directKinematics/directKinematics-types.hpp>

using namespace KDL;

class DirectKinematics : public RTT::TaskContext
{
	public:
		KDL::Chain chain;//Definition of a kinematic chain 
 		//RTT::InputPort< sensor_msgs::JointState > inJState; //input joint state	
 		RTT::InputPort< std::vector <double> > inJntPos; //input joint state	
		RTT::OutputPort< std::vector<DirectKinematicsData> > sortie;
		RTT::OutputPort <unsigned int> sizeChain ;

		DirectKinematics(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		void addPorts();
};
#endif
