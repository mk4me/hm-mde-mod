/********************************************************************
    created:  2013/04/05
    created:  5:4:2013   8:10
    filename: IKalmanFilter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___IKALMANFILTER_H__
#define HEADER_GUARD___IKALMANFILTER_H__

#include <Eigen/Core>

namespace GeneralAlgorithms {

namespace KalmanFilters {

	//! Bazowy template dla uogólnionego schematu filtru kalmana
	//! Definiuje tylko interfejs i aktualizacjê filtru dla nowych pomiarów
	//! Obs³uguje procesy bez sygna³u kontrolnego
	//! \tparam StateS Rozmiar wektora stanu
	//! \tparam MeasurementS Rozmiar wektora pomiaru
	//! \tparam DataT Typ danych
	template<unsigned int StateS, unsigned int MeasurementS = 9, typename DataT = double>
	class IKalmanFilterT
	{
	public:
		//! Rozmiar wektora stanu
		static const int StateSize = StateS;
		//! Rozmiar wektora pomiaru
		static const int MeasurementSize = MeasurementS;

	public:
		//! Typ danych
		typedef DataT DataType;
		//! Stan procesu
		typedef Eigen::Matrix<DataType, StateSize, 1> StateType;
		//! Pomiary procesu
		typedef Eigen::Matrix<DataType, MeasurementSize, 1> MeasurementType;
		//! Macierz kowariancji szumu procesu
		typedef Eigen::Matrix<DataType, StateSize, StateSize> ProcessCovarianceMatrix;
		//! Macierz kowariancji szumu pomiarowego
		typedef Eigen::Matrix<DataType, MeasurementSize, MeasurementSize> MeasurementCovarianceMatrix;

	public:
		//! Destruktor wirtualny
		virtual ~IKalmanFilterT() {}

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const = 0;

		//! Metoda dokonuje jednego pe³nego szacunku stanu procesu
		//! \param m Pomiar procesu
		virtual void estimate(const MeasurementType & m) = 0;
	};

	//! Interfejs filtru kalmana dla danych z sygna³em kontrolnym
	//! \tparam StateS Rozmiar wektora stanu
	//! \tparam ControlS Rozmiar wektora sygna³u kontrolnego
	//! \tparam MeasurementS Rozmiar wektora pomiaru
	//! \tparam DataT Typ danych
	template<unsigned int StateS, unsigned int ControlS = 3, unsigned int MeasurementS = 6, typename DataT = double>
	class IControlKalmanFilterT
	{
	public:
		//! Rozmiar wektora stanu
		static const int StateSize = StateS;
		//! Rozmiar wektora sygna³u kontrolnego
		static const int ControlSize = ControlS;
		//! Rozmiar wektora pomiaru
		static const int MeasurementSize = MeasurementS;

	public:
		//! Typ danych
		typedef DataT DataType;
		//! Stan procesu
		typedef Eigen::Matrix<DataType, StateSize, 1> StateType;
		//! Sygna³ kontrolny
		typedef Eigen::Matrix<DataType, ControlSize, 1> ControlType;
		//! Pomiary procesu
		typedef Eigen::Matrix<DataType, MeasurementSize, 1> MeasurementType;
		//! Macierz kowariancji szumu procesu
		typedef Eigen::Matrix<DataType, StateSize, StateSize> ProcessCovarianceMatrix;
		//! Macierz kowariancji szumu pomiarowego
		typedef Eigen::Matrix<DataType, MeasurementSize, MeasurementSize> MeasurementCovarianceMatrix;

	public:
		//! Destruktor wirtualny
		virtual ~IControlKalmanFilterT() {}

		//! \return Aktualna estymacja stanu procesu
		virtual const StateType & state() const = 0;

		//! Metoda dokonuje jednego pe³nego szacunku stanu procesu
		//! \param m Pomiar procesu
		//! \param c Sygna³ kontrolny dla zadanego pomiaru
		virtual void estimate(const MeasurementType & m, const ControlType & c) = 0;
	};
}

}

#endif	//	HEADER_GUARD___IKALMANFILTER_H__
