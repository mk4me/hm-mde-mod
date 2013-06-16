/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   9:13
    filename: GeneralUKF.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___GENERALUKF_H__
#define HEADER_GUARD___GENERALUKF_H__

#include <GeneralAlgorithms/KalmanFilters/IUnscentedKalmanFilter.h>
#include <boost/bind.hpp>

namespace GeneralAlgorithms {

namespace KalmanFilters
{
	//! Realizacja UKF w oparciu o Scaled Unscented Transform -> SUKF
	//! Wystarczy podaæ funkcje procesu i pomiaru wprost w postaci funktorów
	//! Reszta zajmuje siê algorytm: generacja punktów sigma, estymacja, wyliczanie niepewnoœci
	template<unsigned int StateS, unsigned int MeasurementS = 9, typename DataType = double>
	class ScaledUnscentedKalmanFilterT : public IUnscentedKalmanFilterT<UnscentedTransforms::ScaledUnscentedTransformSigmaPointsSizeHelper<StateS>::size,
		StateS, MeasurementS, DataType>
	{
	private:
		//! Generator punktów sigma
		UnscentedTransforms::ScaledUnscentedTransformSigmaPointsGenerator<StateSize> sigmaPointsGenerator;

	private:
		//! \param estimatedState Stan wokó³ którego generujemy punkty sigma
		//! \param sigmaPoints [out] Punkty sigma
		virtual void generateSigmaPoints(const StateType & estimatedState, const ProcessCovarianceMatrix & estimatedProcessCovariance, SigmaPointsArrayType & sigmaPoints)
		{
			ProcessCovarianceMatrix sqrtInCov = Eigen::SelfAdjointEigenSolver<ProcessCovarianceMatrix>(estimatedProcessCovariance).operatorSqrt();
			sigmaPointsGenerator.generateSigmaPoints(estimatedState, sqrtInCov, sigmaPoints);
		}

		//! \param
		virtual void estimateProcess(StateType & estimatedProcessState,
			ProcessCovarianceMatrix & estimatedProcessCovariance,
			ProcessSigmaPointsArrayType & processSigmaPoints,
			const ProcessCovarianceMatrix & processCovariance,
			const ProcessFunction & processFunction,
			const SigmaPointsArrayType & sigmaPoints)
		{
			processFunction(sigmaPoints[0].point, processSigmaPoints[0]);
			estimatedProcessState = processSigmaPoints[0] *= sigmaPoints[0].weight;

			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				processFunction(sigmaPoints[i].point, processSigmaPoints[i]);
				estimatedProcessState += processSigmaPoints[i] *= sigmaPoints[i].weight;
			}			

			estimatedProcessCovariance = (sigmaPoints[0].weight + 1.0 + sigmaPointsGenerator.b() - std::pow(sigmaPointsGenerator.a(), 2)) * processSigmaPoints[0] * processSigmaPoints[0].transpose();

			processSigmaPoints[0] -= estimatedProcessState;

			//TODO
			//ewentualnie poprawic generowanie centralizeProcessResults (str. 2 Eq. 13 -> Unscented KalmanFilter Tutorial)
			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				processSigmaPoints[i] -= estimatedProcessState;
				estimatedProcessCovariance += sigmaPoints[i].weight * processSigmaPoints[i] * processSigmaPoints[i].transpose() + processCovariance;
			}	
		}

		//! \param
		virtual void estimateMeasurement(MeasurementType & estimatedMeasurement,
			MeasurementCovarianceMatrix & estimatedMeasurementCovariance,
			MeasurementSigmaPointsArrayType & measurementSigmaPoints,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const MeasurementFunction & measurementFunction,
			const SigmaPointsArrayType & sigmaPoints)
		{
			measurementFunction(sigmaPoints[0].point, measurementSigmaPoints[0]);
			estimatedMeasurement = measurementSigmaPoints[0] *= sigmaPoints[0].weight;

			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				measurementFunction(sigmaPoints[i].point, measurementSigmaPoints[i]);
				estimatedMeasurement += measurementSigmaPoints[i] *= sigmaPoints[i].weight;
			}

			estimatedMeasurementCovariance = sigmaPoints[0].weight * measurementSigmaPoints[0] * measurementSigmaPoints[0].transpose();

			for(unsigned int i = 0; i < SigmaPointsSize; ++i){
				measurementSigmaPoints[i] -= estimatedMeasurement;
				estimatedMeasurementCovariance += sigmaPoints[i].weight * measurementSigmaPoints[i] * measurementSigmaPoints[i].transpose() + measurementCovariance;
			}	
		}

	public:

		ScaledUnscentedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: IUnscentedKalmanFilterT<UnscentedTransforms::ScaledUnscentedTransformSigmaPointsSizeHelper<StateS>::size,
			StateS, MeasurementS, DataType>(processFunction,
			measurementFunction, processCovariance, measurementCovariance,
			initialState, stateEstimateCovariance)
		{
			
		}

		const double k() const
		{
			return sigmaPointsGenerator.k();
		}

		void k(const double kk)
		{
			sigmaPointsGenerator.k(kk);
		}

		const double a() const
		{
			return sigmaPointsGenerator.a();
		}

		void a(const double aa)
		{
			sigmaPointsGenerator.a(aa);
		}

		const double b() const
		{
			return sigmaPointsGenerator.b();
		}

		void b(const double bb)
		{
			return sigmaPointsGenerator.b(bb);
		}
	};


	//! Realizacja UKF w oparciu o Scaled Unscented Transform dla procesów z sygna³em kontrolnym -> CSUKF
	//! Wystarczy podaæ funkcje procesu i pomiaru wprost w postaci funktorów uwzglêdniaj¹c przy tym sygna³ kontrolny
	//! Reszta zajmuje siê algorytm: generacja punktów sigma, estymacja, wyliczanie niepewnoœci
	template<unsigned int StateS, unsigned int ControlS, unsigned int MeasurementS = 9, typename DataType = double>
	class ControlScaledUnscentedKalmanFilterT : public IControlUnscentedKalmanFilterT<UnscentedTransforms::ScaledUnscentedTransformSigmaPointsSizeHelper<StateS>::size,
		StateS, ControlS, MeasurementS, DataType>
	{
	private:
		//! Generator punktów sigma
		UnscentedTransforms::ScaledUnscentedTransformSigmaPointsGenerator<StateSize> sigmaPointsGenerator;

	private:
		//! \param estimatedState Stan wokó³ którego generujemy punkty sigma
		//! \param sigmaPoints [out] Punkty sigma
		virtual void generateSigmaPoints(const StateType & estimatedState, const ProcessCovarianceMatrix & estimatedProcessCovariance, SigmaPointsArrayType & sigmaPoints)
		{
			ProcessCovarianceMatrix sqrtInCov = Eigen::SelfAdjointEigenSolver<ProcessCovarianceMatrix>(estimatedProcessCovariance).operatorSqrt();
			sigmaPointsGenerator.generateSigmaPoints(estimatedState, sqrtInCov, sigmaPoints);
		}

		//! \param
		virtual void estimateProcess(StateType & estimatedProcessState,
			const ControlType & control,
			ProcessCovarianceMatrix & estimatedProcessCovariance,
			ProcessSigmaPointsArrayType & processSigmaPoints,
			const ProcessCovarianceMatrix & processCovariance,
			const ProcessFunction & processFunction,
			const SigmaPointsArrayType & sigmaPoints)
		{

			processFunction(sigmaPoints[0].point, control, processSigmaPoints[0]);
			estimatedProcessState = processSigmaPoints[0] *= sigmaPoints[0].weight;

			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				processFunction(sigmaPoints[i].point, control, processSigmaPoints[i]);
				estimatedProcessState += processSigmaPoints[i] *= sigmaPoints[i].weight;
			}

			estimatedProcessCovariance = (sigmaPoints[0].weight + 1.0 + sigmaPointsGenerator.b() - std::pow(sigmaPointsGenerator.a(), 2)) * processSigmaPoints[0] * processSigmaPoints[0].transpose();

			processSigmaPoints[0] -= estimatedProcessState;

			//TODO
			//ewentualnie poprawic generowanie centralizeProcessResults (str. 2 Eq. 13 -> Unscented KalmanFilter Tutorial)
			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				processSigmaPoints[i] -= estimatedProcessState;
				estimatedProcessCovariance += sigmaPoints[i].weight * processSigmaPoints[i] * processSigmaPoints[i].transpose() + processCovariance;
			}	
		}

		//! \param
		virtual void estimateMeasurement(MeasurementType & estimatedMeasurement,
			MeasurementCovarianceMatrix & estimatedMeasurementCovariance,
			MeasurementSigmaPointsArrayType & measurementSigmaPoints,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const MeasurementFunction & measurementFunction,
			const SigmaPointsArrayType & sigmaPoints)
		{

			 measurementFunction(sigmaPoints[0].point, measurementSigmaPoints[0]);
			estimatedMeasurement = measurementSigmaPoints[0] *= sigmaPoints[0].weight;

			for(unsigned int i = 1; i < SigmaPointsSize; ++i){
				measurementFunction(sigmaPoints[i].point, measurementSigmaPoints[i]);
				estimatedMeasurement += measurementSigmaPoints[i] *= sigmaPoints[i].weight;
			}

			estimatedMeasurementCovariance = sigmaPoints[0].weight * measurementSigmaPoints[0] * measurementSigmaPoints[0].transpose();

			for(unsigned int i = 0; i < SigmaPointsSize; ++i){
				measurementSigmaPoints[i] -= estimatedMeasurement;
				estimatedMeasurementCovariance += sigmaPoints[i].weight * measurementSigmaPoints[i] * measurementSigmaPoints[i].transpose() + measurementCovariance;
			}	
		}

	public:

		ControlScaledUnscentedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: IControlUnscentedKalmanFilterT<UnscentedTransforms::ScaledUnscentedTransformSigmaPointsSizeHelper<StateS>::size,
			StateS, ControlS, MeasurementS, DataType>(processFunction,
			measurementFunction, processCovariance, measurementCovariance,
			initialState, stateEstimateCovariance)
		{

		}

		const double k() const
		{
			return sigmaPointsGenerator.k();
		}

		void k(const double kk)
		{
			sigmaPointsGenerator.k(kk);
		}

		const double a() const
		{
			return sigmaPointsGenerator.a();
		}

		void a(const double aa)
		{
			sigmaPointsGenerator.a(aa);
		}

		const double b() const
		{
			return sigmaPointsGenerator.b();
		}

		void b(const double bb)
		{
			return sigmaPointsGenerator.b(bb);
		}
	};
}

typedef KalmanFilters::ControlScaledUnscentedKalmanFilterT<3, 1, 2> MyCSUKF;
typedef KalmanFilters::ScaledUnscentedKalmanFilterT<3, 2> MySUKF;

void constProcessFunction(const MyCSUKF::StateType & s, const MyCSUKF::ControlType & c, MyCSUKF::StateType & outS)
{
	outS = MyCSUKF::StateType::Random();
}

void constProcessFunction(const MySUKF::StateType & s, MySUKF::StateType & outS)
{
	outS = MySUKF::StateType::Random();
}

void constMeasurementFunction(const MyCSUKF::StateType & s, MyCSUKF::MeasurementType & m)
{
	m = MyCSUKF::MeasurementType::Random();
}

void exampleSUKF()
{
	MyCSUKF cFilter(MyCSUKF::ProcessFunction(boost::bind(&constProcessFunction, _1, _2, _3)),
		MyCSUKF::MeasurementFunction(boost::bind(&constMeasurementFunction, _1, _2)),
		MyCSUKF::ProcessCovarianceMatrix::Identity(),
		MyCSUKF::MeasurementCovarianceMatrix::Identity());

	MySUKF filter(MySUKF::ProcessFunction(boost::bind(&constProcessFunction, _1, _2)),
		MySUKF::MeasurementFunction(boost::bind(&constMeasurementFunction, _1, _2)),
		MySUKF::ProcessCovarianceMatrix::Identity(),
		MySUKF::MeasurementCovarianceMatrix::Identity());

	std::cout << "CSUKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

	std::cout << "SUKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;


	for(int i = 0; i < 100; ++i) {		
		cFilter.estimate(MyCSUKF::MeasurementType::Random(), MyCSUKF::ControlType::Random());
		filter.estimate(MySUKF::MeasurementType::Random());

		std::cout << "CSUKF:" << std::endl;
		std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
		std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

		std::cout << "SUKF:" << std::endl;
		std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
		std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;

	}

	std::cout << "CSUKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

	std::cout << "SUKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;
}

}

#endif	//	HEADER_GUARD___GENERALUKF_H__
