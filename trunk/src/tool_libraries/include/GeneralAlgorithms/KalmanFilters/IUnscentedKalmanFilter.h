/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   8:08
    filename: IUnscentedKalmanFilter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___IUNSCENTEDKALMANFILTER_H__
#define HEADER_GUARD___IUNSCENTEDKALMANFILTER_H__

#include <GeneralAlgorithms/KalmanFilters/IKalmanFilter.h>
#include <GeneralAlgorithms/UnscentedTransform/UnscentedTransform.h>
#include <boost/function.hpp>

namespace GeneralAlgorithms {

namespace KalmanFilters {

	//! Wzorzec interfejsu Unscented Kalman Filter, uwzglêdnia iloœc punktów sigma
	template<unsigned int SigmaSize, unsigned int StateS, unsigned int MeasurementS = 9, typename DataType = double>
	class IUnscentedKalmanFilterT : public IKalmanFilterT<StateS, MeasurementS, DataType>
	{
	public:
		//! Iloœæ punktów sigma
		static const unsigned int SigmaPointsSize = SigmaSize;

	public:
		//! Punkty sigma
		typedef UnscentedTransforms::SigmaPoint<StateType> SigmaPoint;
		//! Typ funkcji opisuj¹cej przejœcie procesu
		typedef boost::function<void(const StateType &, StateType&)> ProcessFunction;
		//! Typ funkcji opisuj¹cej przejœcie pomiaru
		typedef boost::function<void(const StateType &, MeasurementType&)> MeasurementFunction;
		//! Typ agregatu punktów sigma
		typedef typename UnscentedTransforms::SigmaPoints<StateType, SigmaPointsSize>::type SigmaPointsArrayType;
		//! Typ agregatu punktów sigma
		typedef boost::array<StateType, SigmaPointsSize> ProcessSigmaPointsArrayType;
		//! Typ agregatu punktów sigma
		typedef boost::array<MeasurementType, SigmaPointsSize> MeasurementSigmaPointsArrayType;
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
		//! \param estimatedState Stan dla którego generujemy punkty sigma
		//! \param estimatedProcessCovariance Kowariancja procesu na bazie której generujemy nowe punkty sigma
		//! \param sigmaPoints [out] Wygenerowane punkty sigma (stan + waga)
		virtual void generateSigmaPoints(const StateType & estimatedState, const ProcessCovarianceMatrix & estimatedProcessCovariance, SigmaPointsArrayType & sigmaPoints) = 0;

		//! Metoda estymuj¹ca stan procesu i jego kowariancjê
		virtual void estimateProcess(StateType & estimatedProcessState,
			ProcessCovarianceMatrix & estimatedProcessCovariance,
			ProcessSigmaPointsArrayType & processSigmaPoints,
			const ProcessCovarianceMatrix & processCovariance,
			const ProcessFunction & processFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! Metoda estymuj¹ca pomiar i jego kowariancjê
		virtual void estimateMeasurement(MeasurementType & estimatedMeasurement,
			MeasurementCovarianceMatrix & estimatedMeasurementCovariance,
			MeasurementSigmaPointsArrayType & measurementSigmaPoints,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const MeasurementFunction & measurementFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}

		//! Metoda realizuj¹ca kolejn¹ estymacjê UKF
		virtual void unscentedKalmanFilter( const MeasurementType & m, StateType & estimatedState, ProcessCovarianceMatrix & stateEstimateCovariance,
			const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,
			const MeasurementCovarianceMatrix & measurementCovariance) 
		{
			//! Filtr UKF

			//! Predict

			//generujemy punkty sigma dla poprzedniego stanu
			SigmaPointsArrayType sigmaPoints;
			generateSigmaPoints(estimatedState, stateEstimateCovariance, sigmaPoints);

			//szcowany stan procesu + szacowana kowariancja procesu + punkty sigma po przejsciu przez funkcjê przejscia procesu (proponowane stany)
			ProcessSigmaPointsArrayType centeredProcessSigmaPoints;
			estimateProcess(estimatedState, stateEstimateCovariance, centeredProcessSigmaPoints,
				processCovariance, processFunction, sigmaPoints);

			//! Update
			//! Innowacja pomiarów
			MeasurementSigmaPointsArrayType centeredMeasurementSigmaPoints;
			MeasurementType predictedMeasurements;
			MeasurementCovarianceMatrix predictedMeasurementCovariance;
			estimateMeasurement(predictedMeasurements, predictedMeasurementCovariance, centeredMeasurementSigmaPoints,
				measurementCovariance, measurementFunction, sigmaPoints);
			
			MeasurementType measurementInnovation = m - predictedMeasurements;
			//! Kowariancja innowacji pomiarów

			KalmanGainMatrix crossCovariance;
			crossCovariance.setZero();

			for(unsigned int i = 0; i < SigmaPointsSize; ++i){
				crossCovariance += sigmaPoints[i].weight * centeredProcessSigmaPoints[i] * centeredMeasurementSigmaPoints[i].transpose();
			}
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = crossCovariance * predictedMeasurementCovariance.inverse();
			//! Aktualizacja stanu
			estimatedState += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance -= kalmanGain * predictedMeasurementCovariance * kalmanGain.transpose();
		}

	protected:
		//! Chroniony kontruktor
		IUnscentedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: processFunction_(processFunction),
			measurementFunction_(measurementFunction),
			processCovariance_(processCovariance),
			measurementCovariance_(measurementCovariance),
			estimatedState_(initialState), stateEstimateCovariance_(stateEstimateCovariance)
		{
			if(processFunction.empty() == true || measurementFunction.empty() == true){
				throw std::invalid_argument("Process function or measurement function empty for EKF");
			}
		}

	public:

		virtual void estimate(const MeasurementType & m)
		{
			unscentedKalmanFilter(m, estimatedState_, stateEstimateCovariance_, processFunction_,
				measurementFunction_, processCovariance_, measurementCovariance_);

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


	//! Wzorzec interfejsu Unscented Kalman Filter dla procesów z sygna³em kontrolnym, uwzglêdnia iloœc punktów sigma
	template<unsigned int SigmaSize, unsigned int StateS, unsigned int ControlS, unsigned int MeasurementS = 9, typename DataType = double>		
	class IControlUnscentedKalmanFilterT : public IControlKalmanFilterT<StateS, ControlS, MeasurementS, DataType>
	{
	public:
		//! Iloœæ punktów sigma
		static const unsigned int SigmaPointsSize = SigmaSize;

	public:
		//! Punktu sigma
		typedef UnscentedTransforms::SigmaPoint<StateType> SigmaPoint;
		//! Typ funkcji opisuj¹cej przejœcie procesu
		typedef boost::function<void(const StateType &, const ControlType &, StateType &)> ProcessFunction;
		//! Typ funkcji opisuj¹cej przejœcie pomiaru
		typedef boost::function<void(const StateType &, MeasurementType &)> MeasurementFunction;
		//! Typ agregatu punktów sigma
		typedef typename UnscentedTransforms::SigmaPoints<StateType, SigmaPointsSize>::type SigmaPointsArrayType;
		//! Typ agregatu punktów sigma
		typedef boost::array<StateType, SigmaPointsSize> ProcessSigmaPointsArrayType;
		//! Typ agregatu punktów sigma
		typedef boost::array<MeasurementType, SigmaPointsSize> MeasurementSigmaPointsArrayType;
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
		//! \param estimatedState Stan wokó³ którego generujemy punkty sigma
		//! \param sigmaPoints [out] Punkty sigma
		virtual void generateSigmaPoints(const StateType & estimatedState, const ProcessCovarianceMatrix & estimatedProcessCovariance, SigmaPointsArrayType & sigmaPoints) = 0;

		//! \param
		virtual void estimateProcess(StateType & estimatedProcessState,
			const ControlType & control,
			ProcessCovarianceMatrix & estimatedProcessCovariance,
			ProcessSigmaPointsArrayType & processSigmaPoints,
			const ProcessCovarianceMatrix & processCovariance,
			const ProcessFunction & processFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! \param
		virtual void estimateMeasurement(MeasurementType & estimatedMeasurement,
			MeasurementCovarianceMatrix & estimatedMeasurementCovariance,
			MeasurementSigmaPointsArrayType & measurementSigmaPoints,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const MeasurementFunction & measurementFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! \param processCovariance Macierz kowariancji procesu któr¹ aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, któr¹ aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}

		virtual void unscentedKalmanFilter( const MeasurementType & m, const ControlType & control,
			StateType & estimatedState, ProcessCovarianceMatrix & stateEstimateCovariance,
			const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,
			const MeasurementCovarianceMatrix & measurementCovariance) 
		{
			//! Filtr UKF

			//! Predict

			//generujemy punkty sigma dla poprzedniego stanu
			SigmaPointsArrayType sigmaPoints;
			generateSigmaPoints(estimatedState, stateEstimateCovariance, sigmaPoints);

			//szcowany stan procesu + szacowana kowariancja procesu + punkty sigma po przejsciu przez funkcjê przejscia procesu (proponowane stany)
			ProcessSigmaPointsArrayType centeredProcessSigmaPoints;
			estimateProcess(estimatedState, control, stateEstimateCovariance, centeredProcessSigmaPoints,
				processCovariance, processFunction, sigmaPoints);

			//! Update
			//! Innowacja pomiarów
			MeasurementSigmaPointsArrayType centeredMeasurementSigmaPoints;
			MeasurementType predictedMeasurement;
			MeasurementCovarianceMatrix predictedMeasurementCovariance;
			estimateMeasurement(predictedMeasurement, predictedMeasurementCovariance, centeredMeasurementSigmaPoints,
				measurementCovariance, measurementFunction, sigmaPoints);

			MeasurementType measurementInnovation = m - predictedMeasurement;
			//! Kowariancja innowacji pomiarów

			KalmanGainMatrix crossCovariance;
			crossCovariance.setZero();

			for(unsigned int i = 0; i < SigmaPointsSize; ++i){
				crossCovariance += sigmaPoints[i].weight * centeredProcessSigmaPoints[i] * centeredMeasurementSigmaPoints[i].transpose();
			}
			//! Wzmocnienie kalmana
			KalmanGainMatrix kalmanGain = crossCovariance * predictedMeasurementCovariance.inverse();
			//! Aktualizacja stanu
			estimatedState += kalmanGain * measurementInnovation;
			//! Aktualizacja macierzy kowariancji estymacji stanu
			stateEstimateCovariance -= kalmanGain * predictedMeasurementCovariance * kalmanGain.transpose();
		}

	protected:

		IControlUnscentedKalmanFilterT(const ProcessFunction & processFunction,
			const MeasurementFunction & measurementFunction,
			const ProcessCovarianceMatrix & processCovariance,		
			const MeasurementCovarianceMatrix & measurementCovariance,
			const StateType & initialState = StateType::Zero(),
			const ProcessCovarianceMatrix & stateEstimateCovariance = ProcessCovarianceMatrix::Identity())
			: processFunction_(processFunction),
			measurementFunction_(measurementFunction),
			processCovariance_(processCovariance),
			measurementCovariance_(measurementCovariance),
			estimatedState_(initialState), stateEstimateCovariance_(stateEstimateCovariance)
		{
			if(processFunction.empty() == true || measurementFunction.empty() == true){
				throw std::invalid_argument("Process function or measurement function empty for EKF");
			}
		}

	public:

		virtual void estimate(const MeasurementType & m, const ControlType & c)
		{
			unscentedKalmanFilter(m, c, estimatedState_, stateEstimateCovariance_, processFunction_,
				measurementFunction_, processCovariance_, measurementCovariance_);

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

#endif	//	HEADER_GUARD___IUNSCENTEDKALMANFILTER_H__
