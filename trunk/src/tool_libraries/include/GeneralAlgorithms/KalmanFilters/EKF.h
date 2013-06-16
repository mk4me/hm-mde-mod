/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   8:19
    filename: EKF.h
    author:   Mateusz Janiak
    
    purpose:  Klasa realizuj¹ca Extended Kalman Filter
*********************************************************************/
#ifndef HEADER_GUARD___EKF_H__
#define HEADER_GUARD___EKF_H__

#include <GeneralAlgorithms/KalmanFilters/IExtendedKalmanFilter.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>

namespace GeneralAlgorithms{

namespace KalmanFilters {

	//! Realizacja EKF operuj¹ca na macierzach funktorów realizuj¹cych Jakobiany
	//! Ka¿dy wpis macierzy powinien reprezentowaæ pochodn¹ cz¹stkow¹ odpowiadaj¹c¹
	//! zadanej zmiennej i równaniu
	template<unsigned int StateS, unsigned int MeasurementS = 9, typename DataType = double>
	class ExtendedKalmanFilterT : public IExtendedKalmanFilterT<StateS, MeasurementS, DataType>
	{
	public:
		//! Typ funkcji pochodnej czasteczkowej na potrzeby Jakobianu procesu
		typedef boost::function<DataType(const StateType &)> ProcessPartialDerrivativeFunctor;
		//! Typ funkcji pochodnej czastkowej na potrzeby Jakobianu pomiaru
		typedef ProcessPartialDerrivativeFunctor MeasurementPartialDerrivativeFunctor;
		//! Jeden wiersz / kolumna jakobianu procesu
		typedef boost::array<ProcessPartialDerrivativeFunctor, StateSize> ProcessJacobianDimension;
		//! Jakobian procesu
		typedef boost::array<ProcessJacobianDimension, StateSize> ProcessJacobianMatrix;	
		//! Jeden wiersz / kolumna jakobianu pomiaru
		typedef boost::array<MeasurementPartialDerrivativeFunctor, StateSize> MeasurementJacobianDimension;
		//! Jakobian pomiaru
		typedef boost::array<MeasurementJacobianDimension, MeasurementSize> MeasurementJacobianMatrix;

	private:
		//! Jakobian procesu
		ProcessJacobianMatrix processJacobian_;
		//! Jakobian pomiaru
		MeasurementJacobianMatrix measurementJacobian_;

	private:

		virtual void computeProcessTransition(const StateType & state, ProcessTransitionMatrix & processTransition)
		{
			for(unsigned int i = 0; i < StateSize; ++i){
				for(unsigned int j = 0; j < StateSize; ++j){
					processTransition(i,j) = processJacobian_[i][j](state);
				}
			}
		}

		virtual void computeMeasurementTransition(const StateType & state, MeasurementTransitionMatrix & measurementTransition)
		{
			for(unsigned int i = 0; i < MeasurementSize; ++i){
				for(unsigned int j = 0; j < StateSize; ++j){
					measurementTransition(i,j) = measurementJacobian_[i][j](state);
				}
			}
		}

	public:
		//! Publiczny konstruktor
		//! \param initialState
		//! \param processTransition Macierz przejscia procesu
		//! \param processCovariance Inicjalny stan macierzy kowariancji dla procesu
		//! \param measurementTransition Macierz przejœcia dla modelu pomiarowego
		//! \param measurementCovariance Macierz kowariancji dla modelu pomiarowego
		//! \param stateEstimateCovariance Inicjalny stan macierzy kowariancji dla estymacji stanu
		ExtendedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessJacobianMatrix & processJacobian,
			const MeasurementJacobianMatrix & measurementJacobian,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: IExtendedKalmanFilterT<StateS, MeasurementS, DataType>(processFunction, measurementFunction,
			processCovariance, measurementCovariance, initialState, stateEstimateCovariance), processJacobian_(processJacobian),
			measurementJacobian_(measurementJacobian)
		{

		}
	};

	//! Realizacja EKF z sygna³em kontrolnym operuj¹ca na macierzach funktorów realizuj¹cych Jakobiany
	//! Ka¿dy wpis macierzy powinien reprezentowaæ pochodn¹ cz¹stkow¹ odpowiadaj¹c¹
	//! zadanej zmiennej i równaniu uwzglêdniaj¹c przy tym sygna³ kontrolny
	template<unsigned int StateS, unsigned int ControlS = 3, unsigned int MeasurementS = 6, typename DataType = double>
	class ControlExtendedKalmanFilterT : public IControllExtendedKalmanFilterT<StateS, ControlS, MeasurementS, DataType>
	{
	public:
		//! Typ funkcji pochodnej czasteczkowej na potrzeby Jakobianów
		typedef boost::function<DataType(const StateType &, const ControlType &)> ProcessPartialDerrivativeFunctor;
		//! Typ funkcji pochodnej czasteczkowej na potrzeby Jakobianów
		typedef boost::function<DataType(const StateType &)> MeasurementPartialDerrivativeFunctor;
		//! Jeden wiersz / kolumna jakobianu procesu
		typedef boost::array<ProcessPartialDerrivativeFunctor, StateSize> ProcessJacobianDimension;
		//! Jakobian procesu
		typedef boost::array<ProcessJacobianDimension, StateSize> ProcessJacobianMatrix;	
		//! Jeden wiersz / kolumna jakobianu pomiaru
		typedef boost::array<MeasurementPartialDerrivativeFunctor, StateSize> MeasurementJacobianDimension;
		//! Jakobian pomiaru
		typedef boost::array<MeasurementJacobianDimension, MeasurementSize> MeasurementJacobianMatrix;

	private:
		//! Jakobian procesu
		ProcessJacobianMatrix processJacobian_;
		//! Jakobian pomiaru
		MeasurementJacobianMatrix measurementJacobian_;

	private:

		virtual void computeProcessTransition(const StateType & state, const ControlType & control, ProcessTransitionMatrix & processTransition)
		{
			for(unsigned int i = 0; i < StateSize; ++i){
				for(unsigned int j = 0; j < StateSize; ++j){
					processTransition(i,j) = processJacobian_[i][j](state, control);
				}
			}
		}

		virtual void computeMeasurementTransition(const StateType & state, MeasurementTransitionMatrix & measurementTransition)
		{
			for(unsigned int i = 0; i < MeasurementSize; ++i){
				for(unsigned int j = 0; j < StateSize; ++j){
					measurementTransition(i,j) = measurementJacobian_[i][j](state);
				}
			}
		}

	public:
		//! Publiczny konstruktor
		//! \param initialState
		//! \param processTransition Macierz przejscia procesu
		//! \param processCovariance Inicjalny stan macierzy kowariancji dla procesu
		//! \param measurementTransition Macierz przejœcia dla modelu pomiarowego
		//! \param measurementCovariance Macierz kowariancji dla modelu pomiarowego
		//! \param stateEstimateCovariance Inicjalny stan macierzy kowariancji dla estymacji stanu
		ControlExtendedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessJacobianMatrix & processJacobian,
			const MeasurementJacobianMatrix & measurementJacobian,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: IControllExtendedKalmanFilterT<StateS, ControlS, MeasurementS, DataType>(processFunction, measurementFunction,
			processCovariance, measurementCovariance, initialState, stateEstimateCovariance), processJacobian_(processJacobian),
			measurementJacobian_(measurementJacobian)
		{

		}
	};
}

typedef KalmanFilters::ControlExtendedKalmanFilterT<3, 1, 2> MyCEKF;
typedef KalmanFilters::ExtendedKalmanFilterT<3, 2> MyEKF;

void constProcessTransition(const MyCEKF::StateType & s, const MyCEKF::ControlType & c, MyCEKF::StateType & outS)
{
	outS = MyCEKF::StateType::Random();
}

void constProcessTransition(const MyEKF::StateType & s, MyEKF::StateType & outS)
{
	outS = MyEKF::StateType::Random();
}

void constMeasurementTransition(const MyEKF::StateType & s, MyEKF::MeasurementType & m)
{
	m = MyEKF::MeasurementType::Random();
}

MyCEKF::DataType constProcessPartialDerrivative(const MyCEKF::StateType & s, const MyCEKF::ControlType & c)
{
	return MyCEKF::DataType(2);
}

MyEKF::DataType constProcessPartialDerrivative(const MyEKF::StateType & s)
{
	return MyEKF::DataType(2);
}

MyEKF::DataType constMeasurementPartialDerrivative(const MyEKF::StateType & s)
{
	return MyEKF::DataType(1);
}

void exampleEKF()
{
	MyCEKF::ProcessJacobianMatrix cProcessJacobianMatrix;
	MyEKF::ProcessJacobianMatrix processJacobianMatrix;

	for(unsigned int i = 0; i < MyCEKF::StateSize; ++i){
		for(unsigned int j = 0; j < MyCEKF::StateSize; ++j){
			cProcessJacobianMatrix[i][j] = MyCEKF::ProcessPartialDerrivativeFunctor(boost::bind(&constProcessPartialDerrivative, _1, _2));
			processJacobianMatrix[i][j] = MyEKF::ProcessPartialDerrivativeFunctor(boost::bind(&constProcessPartialDerrivative, _1));
		}
	}

	MyCEKF::MeasurementJacobianMatrix cMeasurementJacobianMatrix;
	MyEKF::MeasurementJacobianMatrix measurementJacobianMatrix;

	for(unsigned int i = 0; i < MyCEKF::MeasurementSize; ++i){
		for(unsigned int j = 0; j < MyCEKF::StateSize; ++j){
			cMeasurementJacobianMatrix[i][j] = MyCEKF::MeasurementPartialDerrivativeFunctor(boost::bind(&constMeasurementPartialDerrivative, _1));
			measurementJacobianMatrix[i][j] = MyEKF::MeasurementPartialDerrivativeFunctor(boost::bind(&constMeasurementPartialDerrivative, _1));
		}
	}

	MyCEKF cFilter(MyCEKF::ProcessFunction(boost::bind(&constProcessTransition, _1, _2, _3)),
		MyCEKF::MeasurementFunction(boost::bind(&constMeasurementTransition, _1, _2)),
		cProcessJacobianMatrix,
		cMeasurementJacobianMatrix,
		MyCEKF::ProcessCovarianceMatrix::Identity(),
		MyCEKF::MeasurementCovarianceMatrix::Identity());

	MyEKF filter(MyEKF::ProcessFunction(boost::bind(&constProcessTransition, _1, _2)),
		MyEKF::MeasurementFunction(boost::bind(&constMeasurementTransition, _1, _2)),
		processJacobianMatrix,
		measurementJacobianMatrix,
		MyEKF::ProcessCovarianceMatrix::Identity(),
		MyEKF::MeasurementCovarianceMatrix::Identity());

	std::cout << "CEKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

	std::cout << "EKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;


	for(int i = 0; i < 100; ++i) {
		MyEKF::MeasurementType m = MyEKF::MeasurementType::Random();
		cFilter.estimate(m, MyCEKF::ControlType::Random());
		filter.estimate(m);

		std::cout << "CEKF:" << std::endl;
		std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
		std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

		std::cout << "EKF:" << std::endl;
		std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
		std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;

	}

	std::cout << "CEKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << cFilter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << cFilter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << cFilter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << cFilter.state() << std::endl;

	std::cout << "EKF:" << std::endl;
	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;
}

}

#endif	//	HEADER_GUARD___EKF_H__
