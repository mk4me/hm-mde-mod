/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   18:57
    filename: TestingFramework.h
    author:   Mateusz Janiak
    
    purpose:  Klasa pozwalaj¹ca testowaæ ró¿ne implementacje estymacji
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
	//! Typ opisuj¹cy statystyki pomiarów
	struct EstimationStatistics
	{
		//! Suma b³êdów
		IMUDataSample::Vec3 sum;
		//! Œrednia
		IMUDataSample::Vec3 mean;
		//! Mediana
		IMUDataSample::Vec3 median;
		//! Maksymalna wartoœæ
		IMUDataSample::Vec3 max;
		//! Minimalna wartoœæ
		IMUDataSample::Vec3 min;
		//! Odchylenie standardowe
		IMUDataSample::Vec3 stdDev;
		//! Kurtoza
		IMUDataSample::Vec3 kurtosis;
		//! Skoœnoœæ
		IMUDataSample::Vec3 skewness;
	};

	//! Typ opisuj¹cy wyniki estymacji
	struct EstimatorResults
	{
		//! Estymowane wartoœci
		OrientationData results;
		//! Statystyki b³êdów dla danych referencyjnych
		EstimationStatistics estimationErrorStatistics;
	};

	//! Typ mapy estymatora (nazwy) i jego wyników wraz z opisem b³êdów
	typedef std::map<std::string, EstimatorResults> EstimationResults;

private:
	
	//! Typ zbioru algorytmów do testowania
	typedef std::vector<EstimatorPtr> Estimators;

public:
	//! Domyœlny konstruktor
	TestingFramework();
	//! Destruktor
	~TestingFramework();

	//! \param estimator Estymator orientacji dla danych IMU do testów
	void registerEstimator(IIMUOrietnationEstimator * estimator);
	//! \param input Dane wejœciowe z IMU
	//! \param referenceOrientations Dane referencyjne z poprawnymi wartoœciami orientacji
	//! \return Wyniki estymacji z opisem b³êdów
	const EstimationResults test(const IMUData & input, const OrientationData & referenceOrientations);

private:
	//! Zarejestrowane estymatory
	Estimators estimators_;
};

}

#endif	//	HEADER_GUARD___TESTINGFRAMEWORK_H__
