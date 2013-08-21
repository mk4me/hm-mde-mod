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
	const std::vector<IMU::Quat> & xsensOrientations,
	const std::vector<IMU::Quat> & viconOrientations){

	unsigned long int start = 0;
	unsigned long int offset = 0;
	unsigned long int step = 5;
	IMU::Quat rotation(1.0, 0.0, 0.0, 0.0);

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
		IMU::Vec3 o = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(rotation);

		std::cout	<< "Roll : " << osg::RadiansToDegrees(rotation.x()) << std::endl
					<< "Pitch : " << osg::RadiansToDegrees(rotation.y()) << std::endl
					<< "Yaw : " << osg::RadiansToDegrees(rotation.z()) << std::endl;
	}

	std::vector<IMU::Quat> xsensModifiedOrientations;
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
		IMU::Vec3 xo = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(q);

		q = xsensModifiedOrientations[i - start + offset];
		IMU::Vec3 xmo = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(q);

		q = viconOrientations[i];
		IMU::Vec3 vo = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(q);
		
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

	boost::filesystem::path viconDataFile(argv[1]);
	boost::filesystem::path xsensDataFile(argv[2]);

	if(!boost::filesystem::exists(viconDataFile) || !boost::filesystem::exists(xsensDataFile)){
		std::cerr << "Jeden z plików:\n" << viconDataFile << "\nlub\n" << xsensDataFile << "\nnie istnieje." << std::endl;

		return -2;
	}

	//czytanie danych vicon
	IMU::VICONDataReader viconReader(viconDataFile.string());
	IMU::VICONDataSample viconSample;
	std::vector<IMU::VICONDataSample> viconSamples;

	//czytanie danych xsens
	IMU::XSENSDataReader xsensReader(xsensDataFile.string());
	IMU::XSENSDataSample xsensSample;
	std::vector<IMU::XSENSDataSample> xsensSamples;

	while(viconReader.readNextSample(viconSample) == IMU::UniversalDataReaderBase::RESULT_OK){
		viconSamples.push_back(viconSample);
	}

	while(xsensReader.readNextSample(xsensSample) == IMU::UniversalDataReaderBase::RESULT_OK){
		xsensSamples.push_back(xsensSample);
	}
	
	//osie
	IMU::Vec3 x;
	IMU::Vec3 y;
	IMU::Vec3 z;

	IMU::VICONDataSample::getAxis(viconSample, x, y, z);

	//orientacja
	IMU::Vec3 euler = osg::QuatUtils::axisToEuler
		<IMU::Vec3, IMU::Vec3>(x, y, z);

	//kwaternion
	IMU::Quat q = osg::QuatUtils::eulerToQuaternion
		<IMU::Quat, IMU::Vec3>(euler);

	//weryfikacja odwrotnoœci operacji euler<->kwaternion
	IMU::Vec3 euler2 = osg::QuatUtils::quaterionToEuler
		<IMU::Quat, IMU::Vec3>(q);

	//punkty cia³a
	IMU::Vec3 m1 = IMU::Vec3::Zero();
	IMU::Vec3 m2 = viconSample.positionM2() - viconSample.positionM1();
	IMU::Vec3 m3 = viconSample.positionM3() - viconSample.positionM1();
	IMU::Vec3 m4 = viconSample.positionM4() - viconSample.positionM1();
	IMU::Vec3 m5 = viconSample.positionM5() - viconSample.positionM1();
	IMU::Vec3 m6 = viconSample.positionM6() - viconSample.positionM1();

	//pocz¹tkowa pozycja cia³a
	IMU::Vec3 im5 = IMU::Vec3(- m5.norm(), 0.0, 0.0);
	IMU::Vec3 im6 = IMU::Vec3(m6.norm(), 0.0, 0.0);
	IMU::Vec3 im4 = IMU::Vec3(0.0, -m4.norm(), 0.0);

	//cia³o po zmianie
	im5 = q._transformVector(im5);
	im6 = q._transformVector(im6);
	im4 = q._transformVector(im4);

	
	std::vector<IMU::Quat> viconOrientations;
	viconOrientations.reserve(viconSamples.size());

	for(auto it = viconSamples.begin(); it != viconSamples.end(); ++it){
		auto euler = IMU::VICONDataSample::estimateOrientation(*it);
		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(euler);
		viconOrientations.push_back(q);
	}

	std::vector<IMU::Quat> xsensOrientations;
	xsensOrientations.reserve(xsensSamples.size());

	for(auto it = xsensSamples.begin(); it != xsensSamples.end(); ++it){

		IMU::Vec3 rad(
			osg::DegreesToRadians((*it).estimatedOrientationSample().x()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().y()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().z())
			);

		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(rad);
		xsensOrientations.push_back(q);
	}

	fitData("orientacje_VICON_" + xsensDataFile.stem().string() + "_" + viconDataFile.stem().string() + ".csv", xsensOrientations, viconOrientations);

	return 0;
}