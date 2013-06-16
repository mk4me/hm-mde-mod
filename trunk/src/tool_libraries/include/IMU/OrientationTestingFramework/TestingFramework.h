/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   18:57
    filename: TestingFramework.h
    author:   Mateusz Janiak
    
    purpose:  Klasa pozwalaj�ca testowa� r�ne implementacje estymacji
				orientacji na bazie danych z IMU
*********************************************************************/
#ifndef HEADER_GUARD___TESTINGFRAMEWORK_H__
#define HEADER_GUARD___TESTINGFRAMEWORK_H__

#include <IMU/Data/IMUDataSample.h>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

namespace IMU {

class IIMUOrietnationEstimator;

class TestingFramework
{
public:
	//! Typ smart pointera do interfejsu estymatora
	typedef boost::shared_ptr<IIMUOrietnationEstimator> EstimatorPtr;
	//! Typ danych z IMU
	typedef std::vector<IMUDataSample> IMUData;
	//! Typ danych orientacji
	typedef std::vector<IMUDataSample::Vec3> OrientationData;
	//! Typ opisuj�cy statystyki pomiar�w
	struct EstimationStatistics
	{
		//! Suma b��d�w
		IMUDataSample::Vec3 sum;
		//! �rednia
		IMUDataSample::Vec3 mean;
		//! Mediana
		IMUDataSample::Vec3 median;
		//! Maksymalna warto��
		IMUDataSample::Vec3 max;
		//! Minimalna warto��
		IMUDataSample::Vec3 min;
		//! Odchylenie standardowe
		IMUDataSample::Vec3 stdDev;
		//! Kurtoza
		IMUDataSample::Vec3 kurtosis;
		//! Sko�no��
		IMUDataSample::Vec3 skewness;
	};

	//! Typ opisuj�cy wyniki estymacji
	struct EstimatorResults
	{
		//! Estymowane warto�ci
		OrientationData results;
		//! Statystyki b��d�w dla danych referencyjnych
		EstimationStatistics estimationErrorStatistics;
	};

	//! Typ mapy estymatora (nazwy) i jego wynik�w wraz z opisem b��d�w
	typedef std::map<std::string, EstimatorResults> EstimationResults;

private:
	
	//! Typ zbioru algorytm�w do testowania
	typedef std::vector<EstimatorPtr> Estimators;

public:
	//! Domy�lny konstruktor
	TestingFramework();
	//! Destruktor
	~TestingFramework();

	//! \param estimator Estymator orientacji dla danych IMU do test�w
	void registerEstimator(IIMUOrietnationEstimator * estimator);
	//! \param input Dane wej�ciowe z IMU
	//! \param referenceOrientations Dane referencyjne z poprawnymi warto�ciami orientacji
	//! \return Wyniki estymacji z opisem b��d�w
	const EstimationResults test(const IMUData & input, const OrientationData & referenceOrientations);

private:
	//! Zarejestrowane estymatory
	Estimators estimators_;
};

}

#endif	//	HEADER_GUARD___TESTINGFRAMEWORK_H__
