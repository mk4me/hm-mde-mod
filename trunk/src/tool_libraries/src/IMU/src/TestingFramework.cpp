#include <IMU/OrientationTestingFramework/TestingFramework.h>
#include <IMU/OrientationTestingFramework/IIMUOrientationEstimator.h>
#include <limits>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <iostream>

using namespace IMU;

TestingFramework::TestingFramework()
{

}

TestingFramework::~TestingFramework()
{

}

void TestingFramework::registerEstimator(IIMUOrietnationEstimator * estimator)
{
	estimators_.push_back(EstimatorPtr(estimator));
}

const TestingFramework::EstimationResults TestingFramework::test(const IMUData & input, const OrientationData & referenceOrientations)
{
	//assert(input.size() == referenceOrientations.size());

	IMUData::size_type s = std::min(input.size(), referenceOrientations.size());

	using namespace boost::accumulators;
	typedef accumulator_set<IMUDataSample::Vec3::Scalar, stats<tag::mean, tag::moment<2>, tag::variance, tag::median,
		tag::skewness, tag::kurtosis, tag::sum, tag::max, tag::min> > IMUAccumulator;

	// Warto�c zwracana - wyniki testu
	EstimationResults ret;
	// testuj� ka�dy estumator
	for(auto estimatorIT = estimators_.begin(); estimatorIT != estimators_.end(); ++estimatorIT){
		// resetujemy stan estymatora
		(*estimatorIT)->reset();
		std::cout << "Estymator = " << (*estimatorIT)->name() << std::endl;

		// wynik testu estymatora
		EstimatorResults res;

		/*
		// grupa akumlator�w
		IMUAccumulator accX;
		IMUAccumulator accY;
		IMUAccumulator accZ;
		*/
		// najwi�kszy b��d
		//double maxDist = std::numeric_limits<double>::min();
		// najmniejszy b��d
		//double minDist = std::numeric_limits<double>::max();

		// estymuj� dla wszystkich pr�bek
		for(IMUData::size_type i = 0; i < s; ++i) {

			// estymowana orientacja
			IMUDataSample::Vec3 estimatedOrientation;
			// estymuj�
			(*estimatorIT)->estimate(input[i], estimatedOrientation);
			//std::cout << "Sample ID = " << i << "\tEstimated orientation:\t" << estimatedOrientation << std::endl;
			// zapami�tuj� wynik estymacji
			res.results.push_back(estimatedOrientation);
			/*
			// r�nica pomi�dzy warto�ci� estymowan� i faktyczn�
			auto diff = referenceOrientations[i] - estimatedOrientation;
			// b��d estymacji
			auto dist2 = std::pow(diff.x(), 2) + std::pow(diff.y(), 2) + std::pow(diff.z(), 2);

			// aktualizuj� maksymaln� warto�� b��du
			if(dist2 > maxDist){
				maxDist = dist2;
				res.estimationErrorStatistics.max = diff;
			}

			// akrualizuj� minimaln� warto�� b��du
			if(dist2 < minDist){
				minDist = dist2;
				res.estimationErrorStatistics.min = diff;
			}

			// aktualizuj� akumulatory
			accX(diff.x());
			accY(diff.y());
			accZ(diff.z());
			*/
		}
		/*
		// zapami�tuj� statystyki b��d�w estymacji
		res.estimationErrorStatistics.mean.x() = mean(accX);
		res.estimationErrorStatistics.mean.y() = mean(accY);
		res.estimationErrorStatistics.mean.z() = mean(accZ);

		res.estimationErrorStatistics.median.x() = median(accX);
		res.estimationErrorStatistics.median.y() = median(accY);
		res.estimationErrorStatistics.median.z() = median(accZ);

		res.estimationErrorStatistics.sum.x() = sum(accX);
		res.estimationErrorStatistics.sum.y() = sum(accY);
		res.estimationErrorStatistics.sum.z() = sum(accZ);

		res.estimationErrorStatistics.kurtosis.x() = kurtosis(accX);
		res.estimationErrorStatistics.kurtosis.y() = kurtosis(accY);
		res.estimationErrorStatistics.kurtosis.z() = kurtosis(accZ);

		res.estimationErrorStatistics.skewness.x() = skewness(accX);
		res.estimationErrorStatistics.skewness.y() = skewness(accY);
		res.estimationErrorStatistics.skewness.z() = skewness(accZ);

		res.estimationErrorStatistics.stdDev.x() = moment<2>(accX);
		res.estimationErrorStatistics.stdDev.y() = moment<2>(accY);
		res.estimationErrorStatistics.stdDev.z() = moment<2>(accZ);
		*/

		// zapisuj� wyniki estymacji
		ret.insert(EstimationResults::value_type((*estimatorIT)->name() + " implemented by " + (*estimatorIT)->author() , res));
	}

	return ret;
}