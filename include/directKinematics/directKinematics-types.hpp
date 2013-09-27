/**
 * In this header you may define a number of classes or structs,
 * which will be picked up by the typegen tool.
 *
 * typegen will not process headers included by this header.
 * If you want to generate type support for another header than
 * this one, add that header in the CMakeLists.txt file.
 */
#include <vector>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
/**
 * Just an example struct. You may remove/modify it.
 * Note that there are restrictions. Take a look at the
 * Orocos typekit plugin manuals and the typegen documentation.
 */
struct DirectKinematicsData
{
	DirectKinematicsData():frame_id("NameOfTheRobotSegment"), vector(3,100.0), rotation(4,100.0){}
	DirectKinematicsData(std::string frame_id,double vector, double rotation):frame_id("NameOfTheRobotSegment"), vector(3,100.0), rotation(4,100.0){}

	std::string frame_id;
	std::vector<double> vector;		
	std::vector<double> rotation;	

};

DirectKinematicsData createCD(std::string frame_id,double vector, double rotation)
{
	return DirectKinematicsData(frame_id, vector,  rotation);
}

namespace boost
{
	namespace serialization 
	{
		//the helper function
		template<class Archive>
		void serialize(Archive & a, DirectKinematicsData & cd, unsigned int) 
		{
			using boost::serialization::make_nvp;
			a & make_nvp("frame_id",cd.frame_id);
			a & make_nvp("vector",cd.vector);
			a & make_nvp("rotation",cd.rotation);
		}
	}
}

//RTT helper class which uses the above function behind the scences 
struct DirectKinematicsDataTypeInfo
	:public RTT::types::StructTypeInfo<DirectKinematicsData>
{
	DirectKinematicsDataTypeInfo()
		:RTT::types::StructTypeInfo<DirectKinematicsData>("DirectKinematicsData"){}
};
