/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   8:24
    filename: IExtendedKalmanFilter.h
    author:   Mateusz Janiak
    
    purpose:  Interfejs algorytmu realizuj¹cego Extended Kalman Filter
*********************************************************************/
#ifndef HEADER_GUARD___IEXTENDEDKALMANFILTER_H__
#define HEADER_GUARD___IEXTENDEDKALMANFILTER_H__

#include <GeneralAlgorithms/KalmanFilters/IKalmanFilter.h>
#include <boost/function.hpp>

namespace GeneralAlgorithms {

namespace KalmanFilters {

	//! Interfejs uogólnionego rozszerzonego filtru kalmana dla procesów/pomiarow nieliniowych
	//! Interfejs uwzglêdnia funkcjê procesu i pomiaru w formie funktorów
	//! Parametry wzorca patrz interfejs IKalmanFilter
	template<unsigned int StateS, unsigned int MeasurementS = 9, typename DataType = double>
	class IExtendedKalmanFilterT : public IKalmanFilterT<StateS, MeasurementS, DataType>
	{
	public:
		//! Typ funkcji opisuj¹cej przejœcie procesu
		typedef boost::function<void(const StateType &, StateType&)> ProcessFunction;
		//! Typ funkcji opisuj¹cej pomiar
		typedef boost::function<void(const StateType &, MeasurementType&)> MeasurementFunction;
		//! Macierz przejœcia procesu (kwadratowa)
		typedef ProcessCovarianceMatrix ProcessTransitionMatrix;
		//! Macierz modelu pomiarowego (niekoniecznie kwadratowa)
		typedef Eigen::Matrix<DataType, MeasurementSize, StateSize> MeasurementTransitionMatrix;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Funkcja przejœcia
		ProcessFunction processFunction_;
		//! Funkcja pomiaru
		MeasurementFunction measurementFunction_;
		//! Macierz kowariancji modelu pomiarowego
		ProcessCovarianceMatrix processCovariance_;
		//! Macierz kowariancji estymacji stanu
		ProcessCovarianceMatrix stateEstimateCovariance_;
		//! Macierz kowariancji modelu pomiarowego
		MeasurementCovarianceMatrix measurementCovariance_;
		//! Szacowany stan procesu
		StateType estimatedState_;

	private:

		//! \param state Stan dla ktorego wyliczamy jakobian funkcji przejscia
		//! \param processTransition [out] Wyjsciowa macierz - Jakobian funkcji przejscia dla zadanego stanu
		virtual void computeProcessTransition(const StateType & state, ProcessTransitionMatrix & processTransition) = 0;

		//! \param state Stan dla ktorego wyliczamy jakobian funkcji pomiaru
		//! \param measurementTransition [out] Wyjsciowa macierz - Jakobian funkcji pomiaru dla zadanego stanu
		virtual void computeMeasurementTransition(const StateType & state, MeasurementTransitionMatrix & measurementTransition) = 0;

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}

		//! Funkcja realizuje kolejny krok rozszerzonego filtru kalmana dla nowego pomiaru
		virtual void extendedKalmanFilter( const MeasurementType & m, StateType & estimatedState,
			ProcessFunction & processFunction,
			MeasurementFunction & measurementFunction,
			ProcessCovarianceMatrix & stateEstimateCovariance) 
		{
			//! Filtr EKF

			//! Predict
			//! Szacowany stan procesu ze wzglêdu na macierz przejscia i poprzednie oszacowanie stanu - bez pomiarów			
			processFunction(StateType(estimatedState), estimatedState);


			ProcessTransitionMatrix processTransition;
			computeProcessTransition(estimatedState, processTransition);

			//! Wstêpne oszacowanie macierzy kowariancji dla estymacji stanu
			stateEstimateCovariance = processTransition * stateEstimateCovariance * processTransition.transpose() + processCovariance();

			//! Update
			//! Innowacja pomiarów

			MeasurementType predictedMeasurements;
			measurementFunction(estimatedState, predictedMeasurements);
			MeasurementType measurementInnovation = m - predictedMeasurements;
			//! Kowariancja innowacji pomiarów

			MeasurementTransitionMatrix measurementTransition;
			computeMeasurementTransition(estimatedState, measurementTransition);

			MeasurementCovarianceMatrix measurementInnovationCovariance = measurementTransition * stateEstimateCovariance * measurementTransition.transpose() + measurementCovariance();
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = stateEstimateCovariance * measurementTransition.transpose() * measurementInnovationCovariance.inverse();
			//! Aktualizacja stanu
			estimatedState += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance = (ProcessCovarianceMatrix::Identity() - kalmanGain * measurementTransition) * stateEstimateCovariance;
		}

	protected:

		IExtendedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: processFunction_(processFunction),
			measurementFunction_(measurementFunction),
			processCovariance_(processCovariance),
			measurementCovariance_(measurementCovariance),
			stateEstimateCovariance_(stateEstimateCovariance)
		{
			//! Je¿eli brakuje nam któregoœ z funktorów to rzucamy wyj¹tkiem - nie damy rady nic estymowaæ
			if(processFunction.empty() == true || measurementFunction.empty() == true){
				throw std::invalid_argument("Process function or measurement function empty for EKF");
			}
		}

	public:

		virtual void estimate(const MeasurementType & m)
		{
			extendedKalmanFilter(m, estimatedState_, processFunction_,
				measurementFunction_, processCovariance_);

			//! Aktualizacja macierzy kowariancji procesu i modelu pomiarowego
			updateProcessCovariance(processCovariance_);
			updateMeasurementsCovariance(measurementCovariance_);
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

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const
		{
			return estimatedState_;
		}
	};

	//! Interfejs uogólnionego rozszerzonego filtru kalmana dla procesów/pomiarow nieliniowych z sygna³em kontrolnym
	//! Interfejs uwzglêdnia funkcjê procesu i pomiaru w formie funktorów
	//! Parametry wzorca patrz interfejs IControlKalmanFilter
	template<unsigned int StateS, unsigned int ControlS = 3, unsigned int MeasurementS = 6, typename DataType = double>
	class IControllExtendedKalmanFilterT : public IControlKalmanFilterT<StateS, ControlS, MeasurementS, DataType>
	{
	public:
		//! Typ funkcji opisuj¹cej przejœcie procesu
		typedef boost::function<void(const StateType &, const ControlType &, StateType&)> ProcessFunction;
		//! Typ funkcji opisuj¹cej pomiar
		typedef boost::function<void(const StateType &, MeasurementType&)> MeasurementFunction;

		//! Typ macierzy przejœcia procesu - tutaj wyliczony Jakobian dla zadanego stanu i wymuszenia (kwadratowa)
		typedef ProcessCovarianceMatrix ProcessTransitionMatrix;
		//! Typ macierzy pomiaru - tutaj wyliczony Jakobian dla zadanego stanu (niekoniecznie kwadratowa)
		typedef Eigen::Matrix<DataType, MeasurementSize, StateSize> MeasurementTransitionMatrix;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Funkcja przejœcia
		ProcessFunction processFunction_;
		//! Funkcja pomiaru
		MeasurementFunction measurementFunction_;
		//! Szum modelu pomiarowego
		ProcessCovarianceMatrix processCovariance_;
		//! Macierz kowariancji estymacji stanu
		ProcessCovarianceMatrix stateEstimateCovariance_;
		//! Macierz kowariancji szumu modelu pomiarowego
		MeasurementCovarianceMatrix measurementCovariance_;
		//! Szacowany stan procesu
		StateType estimatedState_;

	private:

		//! \param state Stan dla ktorego wyliczamy jakobian funkcji przejscia
		//! \param control Wymuszenie/Sygna³ kontrolny dla ktorego wyliczamy jakobian funkcji przejscia
		//! \param processTransition [out] Wyjsciowa macierz - Jakobian funkcji przejscia dla zadanego stanu i wymuszenia
		virtual void computeProcessTransition(const StateType & state, const ControlType & control, ProcessTransitionMatrix & processTransition) = 0;

		//! \param state Stan dla ktorego wyliczamy jakobian funkcji pomiaru
		//! \param measurementTransition [out] Wyjsciowa macierz - Jakobian funkcji pomiaru dla zadanego stanu
		virtual void computeMeasurementTransition(const StateType & state, MeasurementTransitionMatrix & measurementTransition) = 0;

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}

		//! Funkcja realizuje kolejn¹ estymacjê stanu rozszerzonego filtru kalmana dla zadanego pomiaru i sygna³u kontrolnego
		virtual void controlExtendedKalmanFilter( const MeasurementType & m, const ControlType & control,
			StateType & estimatedStep,
			ProcessFunction & processFunction,
			MeasurementFunction & measurementFunction,
			ProcessCovarianceMatrix & stateEstimateCovariance) 
		{
			//! Filtr EKF

			//! Predict
			//! Szacowany stan procesu ze wzglêdu na macierz przejscia i poprzednie oszacowanie stanu - bez pomiarów			
			processFunction(StateType(estimatedStep), control, estimatedStep);


			ProcessTransitionMatrix processTransition;
			computeProcessTransition(estimatedStep, control, processTransition);

			//! Wstêpne oszacowanie macierzy kowariancji dla estymacji stanu
			stateEstimateCovariance = processTransition * stateEstimateCovariance * processTransition.transpose() + processCovariance();

			//! Update
			//! Innowacja pomiarów

			MeasurementType predictedMeasurement;
			measurementFunction(estimatedStep, predictedMeasurement);
			MeasurementType measurementInnovation = m - predictedMeasurement;
			//! Kowariancja innowacji pomiarów

			MeasurementTransitionMatrix measurementTransition;
			computeMeasurementTransition(estimatedStep, measurementTransition);

			MeasurementCovarianceMatrix measurementInnovationCovariance = measurementTransition * stateEstimateCovariance * measurementTransition.transpose() + measurementCovariance();
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = stateEstimateCovariance * measurementTransition.transpose() * measurementInnovationCovariance.inverse();
			//! Aktualizacja stanu
			estimatedStep += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance = (ProcessCovarianceMatrix::Identity() - kalmanGain * measurementTransition) * stateEstimateCovariance;
		}

	protected:

		IControllExtendedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: processFunction_(processFunction),
			measurementFunction_(measurementFunction),
			processCovariance_(processCovariance),
			measurementCovariance_(measurementCovariance),
			stateEstimateCovariance_(stateEstimateCovariance)
		{
			if(processFunction.empty() == true || measurementFunction.empty() == true){
				throw std::invalid_argument("Process function or measurement function empty for EKF");
			}
		}

	public:

		virtual void estimate(const MeasurementType & m, const ControlType & c)
		{
			controlExtendedKalmanFilter(m, c, estimatedState_, processFunction_,
				measurementFunction_, processCovariance_);

			//! Aktualizacja macierzy kowariancji procesu i modelu pomiarowego
			updateProcessCovariance(processCovariance_);
			updateMeasurementsCovariance(measurementCovariance_);
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

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const
		{
			return estimatedState_;
		}
	};
}

}

#endif	//	HEADER_GUARD___IEXTENDEDKALMANFILTER_H__
