/********************************************************************
    created:  2013/03/14
    created:  14:3:2013   10:11
    filename: KalmanFilter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___KALMANFILTER_H__
#define HEADER_GUARD___KALMANFILTER_H__

#include <GeneralAlgorithms/KalmanFilters/IKalmanFilter.h>
#include <Eigen/Dense>

#include <iostream>

namespace GeneralAlgorithms {

namespace KalmanFilters {

	//! Wzorzec filtru kalmana implementuj¹ce wszystkie operacje na macierzach. Na wejsciu definiujemy rozmiary problemu
	//! i typ danych z nim zwi¹zanych. Podajemy wiêc rozmiar wektora stanu oraz rozmiar wektora pomiarów
	template<int StateS, int MeasurementS = 9, typename DataType = double>
	class KalmanFilterT : public IKalmanFilterT<StateS, MeasurementS, DataType>
	{
	public:
		//! Macierz przejœcia procesu (kwadratowa)
		typedef ProcessCovarianceMatrix ProcessTransitionMatrix;
		//! Macierz modelu pomiarowego (niekoniecznie kwadratowa)
		typedef Eigen::Matrix<DataType, MeasurementSize, StateSize> MeasurementTransitionMatrix;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Szum modelu pomiarowego
		ProcessCovarianceMatrix processCovariance_;
		//! Macierz kowariancji estymacji stanu
		ProcessCovarianceMatrix stateEstimateCovariance_;
		//! Macierz kowariancji szumu modelu pomiarowego
		MeasurementCovarianceMatrix measurementCovariance_;
		//! Macierz przejœcia procesu
		ProcessTransitionMatrix processTransitionMatrix_;
		//! Macierz modelu pomiarowego
		MeasurementTransitionMatrix measurementTransitionMatrix_;
		//! Szacowany stan procesu
		StateType estimatedState_;

	private:

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementsCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}
		//! \param m Pomiar modelu
		//! \param stateEstimateCovariance [out] Macierz niepewnoœci oszacowania stanu modelu
		//! \param processTransitionMatrix [out] Macierz przejscia
		//! \param measurementTransitionMatrix [out] Macierz pomiaru
		virtual void kalmanFilter(const MeasurementType & m, StateType & estimatedState, ProcessCovarianceMatrix & stateEstimateCovariance,
			ProcessTransitionMatrix & processTransitionMatrix, MeasurementTransitionMatrix & measurementTransitionMatrix) 
		{
			//! Filtr kalmana

			//! Predict
			//! Szacowany stan procesu ze wzglêdu na macierz przejscia i poprzednie oszacowanie stanu - bez pomiarów
			estimatedState = processTransitionMatrix * estimatedState;
			//! Wstêpne oszacowanie macierzy kowariancji dla estymacji stanu
			stateEstimateCovariance = processTransitionMatrix * stateEstimateCovariance * processTransitionMatrix.transpose() + processCovariance();

			//! Update
			//! Innowacja pomiarów
			MeasurementType measurementInnovation = m - measurementTransitionMatrix * estimatedState;
			//! Kowariancja innowacji pomiarów
			MeasurementCovarianceMatrix measurementInnovationCovariance = measurementTransitionMatrix * stateEstimateCovariance * measurementTransitionMatrix.transpose() + measurementCovariance();
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = stateEstimateCovariance * measurementTransitionMatrix.transpose() * measurementInnovationCovariance.inverse();
			//! Aktualizacja stanu
			estimatedState += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance = (ProcessCovarianceMatrix::Identity() - kalmanGain * measurementTransitionMatrix) * stateEstimateCovariance;
		}

	public:
		//! Publiczny konstruktor
		//! \param initialState
		//! \param processTransition Macierz przejscia procesu
		//! \param processCovariance Inicjalny stan macierzy kowariancji dla procesu
		//! \param measurementTransition Macierz przejœcia dla modelu pomiarowego
		//! \param measurementCovariance Macierz kowariancji dla modelu pomiarowego
		//! \param stateEstimateCovariance Inicjalny stan macierzy kowariancji dla estymacji stanu
		KalmanFilterT(const ProcessTransitionMatrix & processTransition,
			const ProcessCovarianceMatrix & processCovariance,
			const MeasurementTransitionMatrix & measurementTransition,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity()
			) : processTransitionMatrix_(processTransition), processCovariance_(processCovariance),
			measurementTransitionMatrix_(measurementTransition), measurementCovariance_(measurementCovariance),
			stateEstimateCovariance_(stateEstimateCovariance)
		{

		}

		const ProcessTransitionMatrix & processTransition() const
		{
			return processTransitionMatrix_;
		}

		const MeasurementTransitionMatrix & measurementTransition() const
		{
			return measurementTransitionMatrix_;
		}

		//! \return Aktualna macierz kowariancji dla procesu
		const ProcessCovarianceMatrix & processCovariance() const
		{
			return processCovariance_;
		}

		//! \return Aktualna macierz kowariancji dla modelu pomiarowego
		const MeasurementCovarianceMatrix & measurementCovariance() const
		{
			return measurementCovariance_;
		}

		//! \return Aktualna macierz kowariancji dla estymacji stanu
		const ProcessCovarianceMatrix & estimationCovariance() const
		{
			return stateEstimateCovariance_;
		}

		virtual void estimate(const MeasurementType & m)
		{
			kalmanFilter(m, estimatedState_, stateEstimateCovariance_, processTransitionMatrix_,
				measurementTransitionMatrix_);
			//! Aktualizacja macierzy kowariancji procesu i modelu pomiarowego
			updateProcessCovariance(processCovariance_);
			updateMeasurementsCovariance(measurementCovariance_);
		}

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const
		{
			return estimatedState_;
		}
	};

	//! Wzorzec filtru kalmana implementuj¹ce wszystkie operacje na macierzach. Na wejsciu definiujemy rozmiary problemu
	//! i typ danych z nim zwi¹zanych. Podajemy wiêc rozmiar wektora stanu oraz rozmiar wektora pomiarów
	template<unsigned int StateS, unsigned int ControlS = 3, unsigned int MeasurementS = 6, typename DataType = double>
	class ControlKalmanFilterT : public IControlKalmanFilterT<StateS, ControlS, MeasurementS, DataType>
	{
	public:
		//! Macierz przejœcia procesu (kwadratowa)
		typedef ProcessCovarianceMatrix ProcessTransitionMatrix;
		//! Macierz kontroli procesu (niekoniecznie kwadratowa)
		typedef Eigen::Matrix<DataType, StateSize, ControlSize> ControlTransitionMatrix;
		//! Macierz modelu pomiarowego (niekoniecznie kwadratowa)
		typedef Eigen::Matrix<DataType, MeasurementSize, StateSize> MeasurementTransitionMatrix;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Szum modelu pomiarowego
		ProcessCovarianceMatrix processCovariance_;
		//! Macierz kowariancji estymacji stanu
		ProcessCovarianceMatrix stateEstimateCovariance_;
		//! Macierz kowariancji szumu modelu pomiarowego
		MeasurementCovarianceMatrix measurementCovariance_;
		//! Macierz przejœcia procesu
		ProcessTransitionMatrix processTransitionMatrix_;
		//! Macierz przejœci asygna³u kontrolnego
		ControlTransitionMatrix controlTransitionMatrix_;
		//! Macierz modelu pomiarowego
		MeasurementTransitionMatrix measurementTransitionMatrix_;
		//! Szacowany stan procesu
		StateType estimatedState_;

	private:

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementsCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}
		//! \param m Pomiar modelu
		//! \param c Dane kontrolne modelu (wymuszenie)
		//! \param stateEstimateCovariance [out] Macierz niepewnoœci oszacowania stanu modelu
		//! \param processTransitionMatrix [out] Macierz przejscia
		//! \param measurementTransitionMatrix [out] Macierz pomiaru
		//! \param controlTransitionMatrix [out] Macierz kontroli modelu
		virtual void kalmnaFilter(const MeasurementType & m, const ControlType & c, StateType & estimatedState,
			ProcessCovarianceMatrix & stateEstimateCovariance, ProcessTransitionMatrix & processTransitionMatrix,
			MeasurementTransitionMatrix & measurementTransitionMatrix, ControlTransitionMatrix & controlTransitionMatrix)
		{
			//! Filtr kalmana

			//! Predict
			//! Szacowany stan procesu ze wzglêdu na macierz przejscia i poprzednie oszacowanie stanu - bez pomiarów, z uwzglêdnieniem sygna³ów kontrolnych i macierzy przejœcia dla nich
			estimatedState = processTransitionMatrix * estimatedState + controlTransitionMatrix * c;
			//! Wstêpne oszacowanie macierzy kowariancji dla estymacji stanu
			stateEstimateCovariance = processTransitionMatrix * stateEstimateCovariance * processTransitionMatrix.transpose() + processCovariance();

			//! Update
			//! Innowacja pomiarów
			MeasurementType measurementInnovation = m - measurementTransitionMatrix * predictedState;
			//! Kowariancja innowacji pomiarów
			MeasurementCovarianceMatrix measurementInnovationCovariance = measurementTransitionMatrix * stateEstimateCovariance * measurementTransitionMatrix.transpose() + measurementCovariance();
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = stateEstimateCovariance * measurementTransitionMatrix.transpose() * measurementInnovationCovariance.inverse();
			//! Aktualizacja stanu
			estimatedState += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance_ = (ProcessCovarianceMatrix::Identity() - kalmanGain * measurementTransitionMatrix) * stateEstimateCovariance;
		}

	public:
		//! Publiczny konstruktor
		//! \param initialState
		//! \param processTransition Macierz przejscia procesu
		//! \param processCovariance Inicjalny stan macierzy kowariancji dla procesu
		//! \param measurementTransition Macierz przejœcia dla modelu pomiarowego
		//! \param measurementCovariance Macierz kowariancji dla modelu pomiarowego
		//! \param stateEstimateCovariance Inicjalny stan macierzy kowariancji dla estymacji stanu
		ControlKalmanFilterT(const ProcessTransitionMatrix & processTransition,
			const ProcessCovarianceMatrix & processCovariance,
			const ControlTransitionMatrix & controlTransition,
			const MeasurementTransitionMatrix & measurementTransition,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity()
			) : processTransitionMatrix_(processTransition), processCovariance_(processCovariance),
			controlTransitionMatrix_(controlTransition), measurementTransitionMatrix_(measurementTransition),
			measurementCovariance_(measurementCovariance), stateEstimateCovariance_(stateEstimateCovariance)
		{

		}

		//! \return Aktualna macierz kowariancji dla procesu
		const ProcessCovarianceMatrix & processCovariance() const
		{
			return processCovariance_;
		}

		//! \return Aktualna macierz kowariancji dla modelu pomiarowego
		const MeasurementCovarianceMatrix & measurementCovariance() const
		{
			return measurementCovariance_;
		}

		//! \return Aktualna macierz kowariancji dla estymacji stanu
		const ProcessCovarianceMatrix & estimationCovariance() const
		{
			return stateEstimateCovariance_;
		}

		virtual void estimateStep(const MeasurementType & m, const ControlType & c)
		{
			kalmanFilter(m, c, stateEstimateCovariance_, processTransitionMatrix_,
				measurementTransitionMatrix_, controlTransitionMatrix_);
			//! Aktualizacja macierzy kowariancji procesu i modelu pomiarowego
			updateProcessCovariance(processCovariance_);
			updateMeasurementsCovariance(measurementCovariance_);
		}

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const
		{
			return estimatedState_;
		}
	};
}

typedef KalmanFilters::KalmanFilterT<3, 2> MyKF;

void exampleKF()
{
	MyKF filter(MyKF::ProcessTransitionMatrix::Random(),
		MyKF::ProcessCovarianceMatrix::Identity(),
		MyKF::MeasurementTransitionMatrix::Random(),
		MyKF::MeasurementCovarianceMatrix::Identity());

	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;


	for(int i = 0; i < 100; ++i) {
		filter.estimate(MyKF::MeasurementType::Random());

		std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
		std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
		std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;

	}

	std::cout << "Macierz kowariancji dla estymacji stanu:" << std::endl << filter.estimationCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla procesu:" << std::endl << filter.processCovariance() << std::endl;
	std::cout << "Macierz kowariancji dla obserwacji:" << std::endl << filter.measurementCovariance() << std::endl;
	std::cout << "Estymowany stan procesu:" << std::endl << filter.state() << std::endl;
}

}

#endif	//	HEADER_GUARD___KALMANFILTER_H__
