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

	//! Wzorzec interfejsu Unscented Kalman Filter, uwzgl�dnia ilo�c punkt�w sigma
	template<unsigned int SigmaSize, unsigned int StateS, unsigned int MeasurementS = 9, typename DataType = double>
	class IUnscentedKalmanFilterT : public IKalmanFilterT<StateS, MeasurementS, DataType>
	{
	public:
		//! Ilo�� punkt�w sigma
		static const unsigned int SigmaPointsSize = SigmaSize;

	public:
		//! Punkty sigma
		typedef UnscentedTransforms::SigmaPoint<StateType> SigmaPoint;
		//! Typ funkcji opisuj�cej przej�cie procesu
		typedef boost::function<void(const StateType &, StateType&)> ProcessFunction;
		//! Typ funkcji opisuj�cej przej�cie pomiaru
		typedef boost::function<void(const StateType &, MeasurementType&)> MeasurementFunction;
		//! Typ agregatu punkt�w sigma
		typedef typename UnscentedTransforms::SigmaPoints<StateType, SigmaPointsSize>::type SigmaPointsArrayType;
		//! Typ agregatu punkt�w sigma
		typedef boost::array<StateType, SigmaPointsSize> ProcessSigmaPointsArrayType;
		//! Typ agregatu punkt�w sigma
		typedef boost::array<MeasurementType, SigmaPointsSize> MeasurementSigmaPointsArrayType;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Funkcja przej�cia
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
		//! \param estimatedState Stan dla kt�rego generujemy punkty sigma
		//! \param estimatedProcessCovariance Kowariancja procesu na bazie kt�rej generujemy nowe punkty sigma
		//! \param sigmaPoints [out] Wygenerowane punkty sigma (stan + waga)
		virtual void generateSigmaPoints(const StateType & estimatedState, const ProcessCovarianceMatrix & estimatedProcessCovariance, SigmaPointsArrayType & sigmaPoints) = 0;

		//! Metoda estymuj�ca stan procesu i jego kowariancj�
		virtual void estimateProcess(StateType & estimatedProcessState,
			ProcessCovarianceMatrix & estimatedProcessCovariance,
			ProcessSigmaPointsArrayType & processSigmaPoints,
			const ProcessCovarianceMatrix & processCovariance,
			const ProcessFunction & processFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! Metoda estymuj�ca pomiar i jego kowariancj�
		virtual void estimateMeasurement(MeasurementType & estimatedMeasurement,
			MeasurementCovarianceMatrix & estimatedMeasurementCovariance,
			MeasurementSigmaPointsArrayType & measurementSigmaPoints,
			const MeasurementCovarianceMatrix & measurementCovariance,
			const MeasurementFunction & measurementFunction,
			const SigmaPointsArrayType & sigmaPoints) = 0;

		//! \param processCovariance Macierz kowariancji procesu kt�r� aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, kt�r� aktualizujemy
		virtual void updateMeasurementsCovariance(MeasurementCovarianceMatrix & measurementCovariance) {}

		//! Metoda realizuj�ca kolejn� estymacj� UKF
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

			//szcowany stan procesu + szacowana kowariancja procesu + punkty sigma po przejsciu przez funkcj� przejscia procesu (proponowane stany)
			ProcessSigmaPointsArrayType centeredProcessSigmaPoints;
			estimateProcess(estimatedState, stateEstimateCovariance, centeredProcessSigmaPoints,
				processCovariance, processFunction, sigmaPoints);

			//! Update
			//! Innowacja pomiar�w
			MeasurementSigmaPointsArrayType centeredMeasurementSigmaPoints;
			MeasurementType predictedMeasurements;
			MeasurementCovarianceMatrix predictedMeasurementCovariance;
			estimateMeasurement(predictedMeasurements, predictedMeasurementCovariance, centeredMeasurementSigmaPoints,
				measurementCovariance, measurementFunction, sigmaPoints);
			
			MeasurementType measurementInnovation = m - predictedMeasurements;
			//! Kowariancja innowacji pomiar�w

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


	//! Wzorzec interfejsu Unscented Kalman Filter dla proces�w z sygna�em kontrolnym, uwzgl�dnia ilo�c punkt�w sigma
	template<unsigned int SigmaSize, unsigned int StateS, unsigned int ControlS, unsigned int MeasurementS = 9, typename DataType = double>		
	class IControlUnscentedKalmanFilterT : public IControlKalmanFilterT<StateS, ControlS, MeasurementS, DataType>
	{
	public:
		//! Ilo�� punkt�w sigma
		static const unsigned int SigmaPointsSize = SigmaSize;

	public:
		//! Punktu sigma
		typedef UnscentedTransforms::SigmaPoint<StateType> SigmaPoint;
		//! Typ funkcji opisuj�cej przej�cie procesu
		typedef boost::function<void(const StateType &, const ControlType &, StateType &)> ProcessFunction;
		//! Typ funkcji opisuj�cej przej�cie pomiaru
		typedef boost::function<void(const StateType &, MeasurementType &)> MeasurementFunction;
		//! Typ agregatu punkt�w sigma
		typedef typename UnscentedTransforms::SigmaPoints<StateType, SigmaPointsSize>::type SigmaPointsArrayType;
		//! Typ agregatu punkt�w sigma
		typedef boost::array<StateType, SigmaPointsSize> ProcessSigmaPointsArrayType;
		//! Typ agregatu punkt�w sigma
		typedef boost::array<MeasurementType, SigmaPointsSize> MeasurementSigmaPointsArrayType;
		//! Macierz wzmocnienia kalmana
		typedef Eigen::Matrix<DataType, StateSize, MeasurementSize> KalmanGainMatrix;

	private:
		//! Funkcja przej�cia
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
		//! \param estimatedState Stan wok� kt�rego generujemy punkty sigma
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

		//! \param processCovariance Macierz kowariancji procesu kt�r� aktualizujemy
		virtual void updateProcessCovariance(ProcessCovarianceMatrix & processCovariance) {}
		//! \param measurementCovariance Macierz kowariancji szumu modelu pomiarowego, kt�r� aktualizujemy
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

			//szcowany stan procesu + szacowana kowariancja procesu + punkty sigma po przejsciu przez funkcj� przejscia procesu (proponowane stany)
			ProcessSigmaPointsArrayType centeredProcessSigmaPoints;
			estimateProcess(estimatedState, control, stateEstimateCovariance, centeredProcessSigmaPoints,
				processCovariance, processFunction, sigmaPoints);

			//! Update
			//! Innowacja pomiar�w
			MeasurementSigmaPointsArrayType centeredMeasurementSigmaPoints;
			MeasurementType predictedMeasurement;
			MeasurementCovarianceMatrix predictedMeasurementCovariance;
			estimateMeasurement(predictedMeasurement, predictedMeasurementCovariance, centeredMeasurementSigmaPoints,
				measurementCovariance, measurementFunction, sigmaPoints);

			MeasurementType measurementInnovation = m - predictedMeasurement;
			//! Kowariancja innowacji pomiar�w

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
