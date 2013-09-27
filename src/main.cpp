#include <rtt/RTT.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
 
#include <sensor_msgs/typekit/Types.hpp>
#include <sensor_msgs/JointState.h>

using namespace KDL;
 
 
int main( int argc, char** argv )
{
    	//Definition of a kinematic chain & add segments to the chain. Guillaume -->TO DO : use an URDL file
    	KDL::Chain chain;
	chain.addSegment(Segment("segment_0",Joint("shoulder_0",Joint::RotZ),Frame(Rotation::RPY( 0.503323678478,-0.189146421623,0.277526064738))));
	chain.addSegment(Segment("segment_1",Joint("shoulder_1",Joint::RotY),Frame(Rotation::RPY( 0.239538165056,0.00537995201801,0.0492155584082))));
	chain.addSegment(Segment("segment_2",Joint("shoulder_2",Joint::RotZ),Frame(Rotation::RPY( 0.252444092172,-0.0167820344187,0.0575928285281))));
	chain.addSegment(Segment("segment_3",Joint("elbow_0",Joint::RotY),Frame(Rotation::RPY(-0.211279484805,-0.0186487662306,-0.00721347462905))));
	chain.addSegment(Segment("segment_4",Joint("wist_0",Joint::RotZ),Frame(Rotation::RPY( 0.259090664398,-0.00225330262131,0.0469419954367))));
	chain.addSegment(Segment("segment_5",Joint("wist_1",Joint::RotX),Frame(Rotation::RPY(-0.399346437469,-0.0238815557936,0.011018430381))));
	chain.addSegment(Segment("segment_6",Joint("wist_2",Joint::RotZ),Frame(Rotation::RPY( 0.0273247151986,-0.416007068876,-0.06463859276))));
	chain.addSegment(Segment("segment_7",Joint(Joint::None),Frame(Rotation::RPY(-0.00140457256351,-0.0113238487485,-0.62193962389))));
 
	// Create solver based on kinematic chain
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = JntArray(nj);
 	
	// capture of the joints states provided at the input port of this "bloc" (G)
	RTT::InputPort< sensor_msgs::JointState > inJState; 
	
	sensor_msgs::JointState dataJState;
	RTT::FlowStatus fs = inJState.read(dataJState);

	if(fs == RTT::NewData) //joints states have changed? (G)
	{
		std::vector<double> q = dataJState.position;//dataJState.velocity  dataJState.effort
		// Assign some values to the joint positions
		for(unsigned int i=0;i<nj;i++)
		{
			jointpositions(i)=q[i];
		}
		std::cout << "Joint State" << std::endl;
		for(std::vector<double>::iterator iter = q.begin(); iter!=q.end(); ++iter)
		{
			std::cout << *iter << " ";
		}
		std::cout << std::endl;
	}

	// Create the frame that will contain the results
	KDL::Frame cartpos;    

	// Calculate forward position kinematics
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions,cartpos);

	if(kinematics_status>=0)
	{
		std::cout << cartpos <<std::endl;
		printf("%s \n","Succes, thanks KDL!");
	}
	else
	{
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}
	return 0;
}
