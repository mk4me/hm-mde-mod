/********************************************************************
    created:  2013/02/03
    created:  3:2:2013   19:14
    filename: main.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#include <IMU/Parsers/XSENSDataReader.h>
#include <IMU/Data/XSENSDataSample.h>
#include <IMU/Parsers/VICONDataReader.h>
#include <IMU/Data/VICONDataSample.h>
#include <osg/Math>
#include <boost/filesystem.hpp>
#include <iostream>
#include <IMU/Algorithms/OrientationFitting.h>
#include <QuatUtils/QuatUtils.h>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternion<double>)


void fitData(const std::string & outFile,
	const std::vector<IMU::OrientationFitting::Quat> & xsensOrientations,
	const std::vector<IMU::OrientationFitting::Quat> & viconOrientations){

	unsigned long int start = 0;
	unsigned long int offset = 0;
	unsigned long int step = 5;
	IMU::OrientationFitting::Quat rotation(1.0, 0.0, 0.0, 0.0);

	if(xsensOrientations.size() > viconOrientations.size()){
		
		IMU::OrientationFitting::overlap(viconOrientations.begin(),
			viconOrientations.end(), xsensOrientations.begin(),
			xsensOrientations.end(), rotation, offset, 5, 0);

		auto localOffset = offset;
		auto range = step;

		if(localOffset >= step){
			localOffset -= step;
			range += step;
		}
		/*
		if(localOffset + range + viconOrientations.size() > xsensOrientations.size()){
			range = xsensOrientations.size() - viconOrientations.size() - localOffset;
		}

		IMU::OrientationFitting::fitOrientation(viconOrientations.begin(),
			viconOrientations.end(), xsensOrientations.begin() + localOffset,
			range, rotation, offset);

		offset += localOffset;
		*/

	}else{
		IMU::OrientationFitting::overlap(xsensOrientations.begin(),
			xsensOrientations.end(), viconOrientations.begin(),
			viconOrientations.end(), rotation, offset, 5, 0);

		auto localOffset = offset;
		auto range = step;

		if(localOffset >= step){
			localOffset -= step;
			range += step;
		}

		/*
		if(localOffset + range + xsensOrientations.size() > viconOrientations.size()){
			range = viconOrientations.size() - xsensOrientations.size() - localOffset;
		}

		IMU::OrientationFitting::fitOrientation(xsensOrientations.begin(),
			xsensOrientations.end(), viconOrientations.begin() + localOffset,
			range, rotation, offset);

		offset += localOffset;
		*/

		start = offset;
		offset = 0;
		rotation = rotation.inverse();
	}

	{
		IMU::VICONDataSample::Vec3 o;
		osg::QuatUtils::quaterionToEuler(rotation.x(), rotation.y(), rotation.z(), rotation.w(), o.x(), o.y(), o.z());
		std::cout	<< "Roll : " << osg::RadiansToDegrees(rotation.x()) << std::endl
					<< "Pitch : " << osg::RadiansToDegrees(rotation.y()) << std::endl
					<< "Yaw : " << osg::RadiansToDegrees(rotation.z()) << std::endl;
	}

	std::vector<IMU::OrientationFitting::Quat> xsensModifiedOrientations;
	xsensModifiedOrientations.reserve(xsensOrientations.size());

	for(auto it = xsensOrientations.begin(); it != xsensOrientations.end(); ++it){
		xsensModifiedOrientations.push_back(*it * rotation);
	}

	unsigned long int stop = std::min(viconOrientations.size() - start, xsensOrientations.size() + offset);

	std::ofstream out(outFile);
	out << "XSENS_Roll;XSENS_Pitch;XSENS_Yaw;XSENS_MOD_Roll;XSENS_MOD_Pitch;XSENS_MOD_Yaw;VICON_Roll;VICON_Pitch;VICON_Yaw;"
		<< std::endl;


	for(auto i = start; i < stop; ++i){

		auto q = xsensOrientations[i - start + offset];
		IMU::VICONDataSample::Vec3 xo;
		osg::QuatUtils::quaterionToEuler(q.x(), q.y(), q.z(), q.w(), xo.x(), xo.y(), xo.z());

		q = xsensModifiedOrientations[i - start + offset];
		IMU::VICONDataSample::Vec3 xmo;
		osg::QuatUtils::quaterionToEuler(q.x(), q.y(), q.z(), q.w(), xmo.x(), xmo.y(), xmo.z());

		q = viconOrientations[i];
		IMU::VICONDataSample::Vec3 vo;
		osg::QuatUtils::quaterionToEuler(q.x(), q.y(), q.z(), q.w(), vo.x(), vo.y(), vo.z());
		
		out << osg::RadiansToDegrees(xo.x()) << ";"
			<< osg::RadiansToDegrees(xo.y()) << ";"
			<< osg::RadiansToDegrees(xo.z()) << ";"
			<< osg::RadiansToDegrees(xmo.x()) << ";"
			<< osg::RadiansToDegrees(xmo.y()) << ";"
			<< osg::RadiansToDegrees(xmo.z()) << ";"
			<< osg::RadiansToDegrees(vo.x()) << ";"
			<< osg::RadiansToDegrees(vo.y()) << ";"
			<< osg::RadiansToDegrees(vo.z()) << ";" << std::endl;
		
	}

	out.close();

}


int main( int argc, char **argv )
{

	if(argc < 3){
		std::cerr << "Za ma³o parametrów - wymagane s¹ conajmniej 2.\n" \
			"Pierwszy parametr opisuje œcie¿kê do pliku z danymi VICON.\n" \
			"Drugi parametr opisuje œcie¿kê do odpowiadaj¹cego pliku z danymi XSENS" << std::endl;

		return -1;
	}

	std::string viconDataFile(argv[1]);
	std::string xsensDataFile(argv[2]);

	if(!boost::filesystem::exists(viconDataFile) || !boost::filesystem::exists(xsensDataFile)){
		std::cerr << "Jeden z plików:\n" << viconDataFile << "\nlub\n" << xsensDataFile << "\nnie istnieje." << std::endl;

		return -2;
	}

	//czytanie danych vicon
	IMU::VICONDataReader viconReader(viconDataFile);
	IMU::VICONDataSample viconSample;
	std::vector<IMU::VICONDataSample> viconSamples;

	//czytanie danych xsens
	IMU::XSENSDataReader xsensReader(xsensDataFile);
	IMU::XSENSDataSample xsensSample;
	std::vector<IMU::XSENSDataSample> xsensSamples;

	while(viconReader.readNextSample(viconSample) == IMU::UniversalDataReader<IMU::VICONDataSample>::RESULT_OK){
		viconSamples.push_back(viconSample);
	}

	while(xsensReader.readNextSample(xsensSample) == IMU::UniversalDataReader<IMU::XSENSDataSample>::RESULT_OK){
		xsensSamples.push_back(xsensSample);
	}

	std::vector<IMU::OrientationFitting::Quat> viconOrientations;
	viconOrientations.reserve(viconSamples.size());

	for(auto it = viconSamples.begin(); it != viconSamples.end(); ++it){
		auto euler = IMU::VICONDataSample::estimateOrientation(*it);
		IMU::OrientationFitting::Quat q;
		osg::QuatUtils::eulerToQuaternion(euler.x(), euler.y(), euler.z(),
			q.x(), q.y(), q.z(), q.w());
		viconOrientations.push_back(q);
	}

	std::vector<IMU::OrientationFitting::Quat> xsensOrientations;
	xsensOrientations.reserve(xsensSamples.size());

	for(auto it = xsensSamples.begin(); it != xsensSamples.end(); ++it){
		IMU::OrientationFitting::Quat q;
		osg::QuatUtils::eulerToQuaternion(
			osg::DegreesToRadians((*it).estimatedOrientationSample().x()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().y()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().z()),
			q.x(), q.y(), q.z(), q.w());
		xsensOrientations.push_back(q);
	}

	fitData("orientacje_VICON.csv", xsensOrientations, viconOrientations);

	return 0;
}