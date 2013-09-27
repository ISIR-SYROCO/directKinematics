#include "directKinematics-component.hpp"
#include <rtt/Component.hpp>


DirectKinematics::DirectKinematics(std::string const& name) : TaskContext(name)
{
	//tell the RTT the name and type of this struct
	RTT::types::Types()->addType(new DirectKinematicsDataTypeInfo());
	RTT::types::Types()->type("DirectKinematicsData")->addConstructor( RTT::types::newConstructor(&createCD) );
	
	//register a std::vector (or compatible) of some type
	RTT::types::Types()->addType(new RTT::types::SequenceTypeInfo< std::vector<DirectKinematicsData> > ("std::vector<DirectKinematicsData>"));

	this->addProperty("chain",chain)
		.doc("the kinematic chain of the kuka robot");

	this->addEventPort("inJointState", inJState)
		.doc("joints' position from kuka robot");
}
//----------------------------------
bool DirectKinematics::configureHook()
{
	// add segments to the chain, (RPY from kuka_robot.urdf)
	chain.addSegment(Segment("world",Joint(Joint::None),Frame(Rotation::RPY( 0,0,3.14159265359/6),Vector(0,  0, 0)))); //sovan
	chain.addSegment(Segment("world_robot",Joint("shoulder_0",Joint::RotZ),Frame(Rotation::RPY( 0,0,3.14159265359),Vector(0,  0, 0.3105)))); 
	chain.addSegment(Segment("segment_1",Joint("shoulder_1",Joint::RotY),Frame(Rotation::RPY( 0,0,0),Vector(0,  0,  0))));
	chain.addSegment(Segment("segment_2",Joint("shoulder_2",Joint::RotZ),Frame(Rotation::RPY(  0,0,3.14159265359),Vector(0, 0, 0.4))));
	chain.addSegment(Segment("segment_3",Joint("elbow_0",Joint::RotY),Frame(Rotation::RPY(0, 0, 0),Vector(0, 0, 0))));
	chain.addSegment(Segment("segment_4",Joint("wist_0",Joint::RotZ),Frame(Rotation::RPY(0,  0, -1.570796326795),Vector(0, 0, 0.39))));
	chain.addSegment(Segment("segment_5",Joint("wist_1",Joint::RotX),Frame(Rotation::RPY(0, 0, 0),Vector(0,  0,  0))));
	chain.addSegment(Segment("segment_6",Joint("wist_2",Joint::RotZ),Frame(Rotation::RPY(0,  0,  0),Vector(0.016617,  -0.016932,  0.238283))));
	chain.addSegment(Segment("segment_7",Joint(Joint::None),Frame(Rotation::RPY( 0,0,0),Vector(0,  0,  0))));
	
	addPorts();
	return true;
	std::cout << "DirectKinematics configured !" <<std::endl;
}
//----------------------------------
bool DirectKinematics::startHook()
{
	
	std::cout << "DirectKinematics started !" <<std::endl;
	return true;
}
//----------------------------------
void DirectKinematics::updateHook()
{
	// Create solver based on kinematic chain
	KDL::ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	KDL::ChainFkSolverVel_recursive fksolverVel = ChainFkSolverVel_recursive(chain); //vel

	unsigned int nj = chain.getNrOfJoints();
	KDL::JntArray jointpositions = JntArray(nj);
	//KDL::JntArrayVel  jointpositionsVel = JntArrayVel(nj); //vel

	sensor_msgs::JointState dataJState;		//received robot joints state
	RTT::FlowStatus fs = inJState.read(dataJState);

	if(fs == RTT::NewData)
	{
		std::vector<double> q = dataJState.position; //dataJState.velocity  dataJState.effort
		//std::vector<double> qdot = dataJState.velocity; //dataJState.velocity  dataJState.effort
		for(std::vector<double>::iterator iter = dataJState.position.begin(); iter!=dataJState.position.end(); ++iter)
		{
			q.push_back(*iter);
			//std::cout << *iter << " ";

			
		}
/*
		for(std::vector<double>::iterator iter = dataJState.velocity.begin(); iter!=dataJState.velocity.end(); ++iter)
		{
			qdot.push_back(*iter);		//vel
			std::cout << *iter << " ";	//vel
		}
*/
		// Assign values to the joint positions
		for(unsigned int i=0;i<nj;i++)
		{
			jointpositions(i)=q[i];
			//jointpositionsVel.q(i)=q[i]; //vel
			//jointpositionsVel.qdot(i)=qdot[i]; //vel
		}
		//std::cout << "Joint State" << std::endl;
/*
		for(std::vector<double>::iterator iter = q.begin(); iter!=q.end(); ++iter)
		{
			std::cout << *iter << " ";
		}
		std::cout << std::endl;
	*/

	unsigned int size(chain.getNrOfSegments());
	std::vector<DirectKinematicsData> donnee;	//sent data(rotation+vector=Frame) of the robot segments
	donnee.resize(size);

	KDL::Frame cartpos;
	//KDL::FrameVel cartposVel;
	bool kinematics_status, vel_status;

	for(unsigned int i = 0 ; i < size ; i++)
	{
		// Calculate forward kinematics position
		kinematics_status = fksolver.JntToCart(jointpositions,cartpos,i);
		//vel_status = fksolverVel.JntToCart(jointpositionsVel,cartposVel,i);
		if(kinematics_status>=0)
		{
			donnee[i].frame_id = chain.getSegment(i).getName();
			donnee[i].vector[0] = cartpos.p.x();
			donnee[i].vector[1] = cartpos.p.y();
			donnee[i].vector[2] = cartpos.p.z();

			cartpos.M.GetQuaternion(donnee[i].rotation[0],donnee[i].rotation[1],donnee[i].rotation[2],donnee[i].rotation[3]);

			//std::cout <<"Succes on forward kinematics!" <<std::endl;		
		}
		/*
		else
		{
			std::cout << "Error: could not calculate forward kinematics!" <<std::endl;
		}
		
	
		if(vel_status>=0)
		{
			std::cout <<"Succes on forward kinematics vel!" <<std::endl;		
		}
		
		else
		{
			std::cout << "Error: could not calculate forward kinematics vel!" <<std::endl;
		}
		*/
	}

	sortie.write(donnee);
}
}
//----------------------------------
void DirectKinematics::stopHook() 
{
	std::cout << "DirectKinematics executes stopping !" <<std::endl;
}
//----------------------------------
void DirectKinematics::cleanupHook() 
{
	std::cout << "DirectKinematics cleaning up !" <<std::endl;
}
void DirectKinematics::addPorts()
{
	std::vector<DirectKinematicsData> donnee;
	unsigned int size = chain.getNrOfSegments();
	donnee.resize(size);
	for(unsigned int i = 0 ;  i < size ; i++)
	{
		donnee[i] = DirectKinematicsData("NameOfTheRobotSegment",100.00,100.0);
	}
	this->addPort("sortie",sortie);
	sortie.setDataSample(donnee);
	this->addPort("sizeChain",sizeChain);
	sizeChain.setDataSample(size);
}//addPorts
//----------------------------------
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(DirectKinematics)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//----------------------------------
ORO_CREATE_COMPONENT(DirectKinematics)
//----------------------------------
