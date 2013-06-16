/********************************************************************
    created:  2013/02/03
    created:  3:2:2013   19:14
    filename: main.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#include <GeneralAlgorithms/KalmanFilters/KalmanFilter.h>
#include <GeneralAlgorithms/KalmanFilters/EKF.h>
#include <GeneralAlgorithms/KalmanFilters/UKF.h>
#include <IMU/Parsers/IMUDataReader.h>
#include <IMU/Parsers/XSENSDataReader.h>
#include <IMU/Parsers/VICONDataReader.h>
#include <IMU/Data/IMUDataSample.h>
#include <IMU/Data/XSENSDataSample.h>
#include <IMU/Data/VICONDataSample.h>
#include <IMU/OrientationTestingFramework/TestingFramework.h>
#include <IMU/OrientationTestingFramework/IIMUOrientationEstimator.h>
#include <IMU/OrientationTestingFramework/MJ_UKF_OrientationEstimator.h>
#include <IMU/OrientationTestingFramework/CSVTestResultSerializer.h>
#include <osg/Math>

#include <boost/filesystem.hpp>

using namespace GeneralAlgorithms;

//! Klasa pustego estymatora
class EmptyIMUOrientationEstimator : public IMU::IIMUOrietnationEstimator
{
public:
	EmptyIMUOrientationEstimator() {}
	virtual ~EmptyIMUOrientationEstimator() {}

	//! Metoda resetuje stan estymatora
	virtual void reset() {}

	//! \param sample Kolejna próbka z IMU
	//! \param orientation [out] Estymowana orientacja
	virtual void estimate(const IMU::IMUDataSample & sample, IMU::IMUDataSample::Vec3 & orientation)
	{
		orientation.x() = sample.gyroscopeSample().x();
		orientation.y() = sample.gyroscopeSample().y();
		orientation.z() = sample.gyroscopeSample().z();
	}

	//! \return Nazwa algorytmu
	virtual const std::string name() const
	{
		return std::string("Dummy IMU orientation estimator");
	}

	//! \return Autor algorytmu (implementacja)
	virtual const std::string author() const
	{
		return std::string("Dummy author");
	}
};

int main( int argc, char **argv )
{

	if(argc < 3){
		std::cerr << "Za ma³o parametrów - wymagane s¹ conajmniej 2.\n" \
			"Pierwszy parametr opisuje œcie¿kê do pliku z danymi IMU.\n" \
			"Drugi parametr opisuje œcie¿kê do odpowiadaj¹cego pliku z danymi XSENS" << std::endl;

		return -1;
	}

	std::string imuDataFile(argv[1]);
	std::string xsensDataFile(argv[2]);

	if(!boost::filesystem::exists(imuDataFile) || !boost::filesystem::exists(xsensDataFile)){
		std::cerr << "Jeden z plików:\n" << imuDataFile << "\nlub\n" << xsensDataFile << "\nnie istnieje." << std::endl;

		return -2;
	}


	IMU::TestingFramework::IMUData imuSamples;
	IMU::TestingFramework::OrientationData orientationSamples;

	//czytanie danych imu
	IMU::IMUDataReader imuReader(imuDataFile);
	IMU::IMUDataSample imuSample;

	while(imuReader.readNextSample(imuSample) == IMU::UniversalDataReader<IMU::IMUDataSample>::RESULT_OK){
		imuSamples.push_back(imuSample);		
	}	

	//czytanie danych xsens
	IMU::XSENSDataReader xsensReader(xsensDataFile);
	IMU::XSENSDataSample xsensSample;

	while(xsensReader.readNextSample(xsensSample) == IMU::UniversalDataReader<IMU::XSENSDataSample>::RESULT_OK){
		orientationSamples.push_back(xsensSample.estimatedOrientationSample());
	}

	//œrodowisko testowe z pustym przyk³adowym estymatorem
	IMU::TestingFramework imuTest;

	//szum procesu - zak³adam 10 stopni na sekundê dla prêdkoœci k¹towych
	// 2 stopnie na sekundê dla orientacji
	MJ_UKF_OrientationEstimator::ProcessCovarianceMatrix processNoise = MJ_UKF_OrientationEstimator::ProcessCovarianceMatrix::Zero();

	//orientacja
	processNoise(0,0) = processNoise(1,1) = processNoise(2,2) = 2.0;
	//prêdkoœæ k¹towa
	processNoise(3,3) = processNoise(4,4) = processNoise(5,5) = 0.5;

	MJ_UKF_OrientationEstimator::MeasurementCovarianceMatrix measurementNoise = MJ_UKF_OrientationEstimator::MeasurementCovarianceMatrix::Zero();

	//¿yroskop
	measurementNoise(0,0) = measurementNoise(1,1) = measurementNoise(2,2) = 0.5;	
	//akcelerometr
	measurementNoise(3,3) = measurementNoise(4,4) = measurementNoise(5,5) = 2.1;
	//magnetometr
	measurementNoise(6,6) = measurementNoise(7,7) = measurementNoise(8,8) = 0.004;	

	imuTest.registerEstimator(new MJ_UKF_OrientationEstimator(processNoise, measurementNoise));

	auto size = std::max(imuSamples.size(), orientationSamples.size());

	if(imuSamples.size() != size){
		imuSamples.resize(size);
	}

	if(orientationSamples.size() != size){
		orientationSamples.resize(size);
	}

	auto testResults = imuTest.test(imuSamples, orientationSamples);

	//wyniki estymacji
	for(auto it = testResults.begin(); it != testResults.end(); ++it){
		std::cout << "Estimation results for " << it->first << std::endl;
		std::cout << "Estimation error statistics" << std::endl;
		std::cout << "Min error: " << it->second.estimationErrorStatistics.min << std::endl;
		std::cout << "Max error: " << it->second.estimationErrorStatistics.max << std::endl;
		std::cout << "Mean error: " << it->second.estimationErrorStatistics.mean << std::endl;
		std::cout << "Median error: " << it->second.estimationErrorStatistics.median << std::endl;
		std::cout << "StdDev error: " << it->second.estimationErrorStatistics.stdDev << std::endl;
		std::cout << "Skewness error: " << it->second.estimationErrorStatistics.skewness << std::endl;
		std::cout << "Kurtosis error: " << it->second.estimationErrorStatistics.kurtosis << std::endl;
	}

	IMU::CSVTestResultSerializer serializer("imuEstimatorTestsResult.csv");

	serializer.serialize(testResults, orientationSamples);

	return 0;
}