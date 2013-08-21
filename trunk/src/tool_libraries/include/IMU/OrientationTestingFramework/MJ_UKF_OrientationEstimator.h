/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   16:24
    filename: MJ_UKF_OrientationEstimator.h
    author:   Mateusz Janiak
    
    purpose:  Implementacja estymatora oeirntacji dla danych IMU w oparciu o filtr UKF
*********************************************************************/
#ifndef HEADER_GUARD___MJ_UKF_ORIENTATIONESTIMATOR_H__
#define HEADER_GUARD___MJ_UKF_ORIENTATIONESTIMATOR_H__

#include <IMU/Data/Types.h>
#include <IMU/OrientationTestingFramework/IIMUOrientationEstimator.h>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <Eigen/Geometry>

//! Implementacja algorytmu wg artyku�u: Edgar Kraft "A Quaternion-based
//! Unscented Kalman Filter for Orientation Tracking"
class MJ_UKF_OrientationEstimator : public IMU::IIMUOrietnationEstimator
{
private:
	//! Rozmiar wektora stanu - tutaj 6 ale wynika to z tego
	//! �e przyjmujemy i� kwaternion ma d�ugo�� 1 dlatego
	//! jeden stopie� swobody odpada
	const static unsigned int StateSize = 6;
	//! Rozmiar wektora pomiaru
	const static unsigned int MeasurementSize = 9;
	//! Ilo�� punkt�w sigma ze wzgl�du na rozmiar wektora stanu
	const static unsigned int SigmaPointsSize = 2 * StateSize + 1;

private:

	//! Typ opisuj�cy stan procesu
	//! Pierwsze 4 elementy wektora to kwaternion orientacji:
	//! x, y, z, w, zas kolejne 3 to pr�dko�ci k�towe cia�a
	//typedef boost::array<double, 7> StateType;
	
	//! Struktura opisuj�ca stan procesu
	struct StateType {
		//! Orientacja cia�a
		IMU::Quat orientation;
		//! Pr�dko�ci k�towe cia��
		IMU::Vec3 angularVelocities;
	};

	//! Stan w formie wektora
	typedef Eigen::Matrix<double, StateSize, 1> VectorStateType;
	
	//! Agregat punkt�w sigma stanu
	typedef boost::array<StateType, SigmaPointsSize> SigmaPoints;
	//! Wektor pomiar�w
	typedef Eigen::Matrix<double, MeasurementSize, 1> MeasurementsVector;
	//! Agregat projekcji pomiar�w dla punkt�w sigma stanu
	typedef boost::array<MeasurementsVector, SigmaPointsSize> ProjectedMeasurements;
	//! Agregat r�nic orientacji wzgl�dem estymowanego stanu apriori
	typedef boost::array<IMU::Quat, SigmaPointsSize> OrientationErrors;
	//! Macierz wzmocnienia kalmana
	typedef Eigen::Matrix<double, StateSize, MeasurementSize> KalmanGainMatrix;

private:
	//! \param state Stan cia�a
	//! \return Kwaternion orientacji
	static const IMU::Quat getOrientation(const StateType & state);
	//! \param state Stan cia�a
	//! \return Wektor pr�dko�ci k�towych cia�a
	static const IMU::Vec3 getAngularVelocities(const StateType & state);
	//! \param state Stan cia�a
	//! \param orientation Kwaternion orientacji
	static void updateOrientation(StateType & state, const IMU::Quat & orientation);
	//! \param state Stan cia�a
	//! \param angularVelocities Wektor pr�dko�ci k�towych cia�a
	static void updateAngularVelocities(StateType & state, const IMU::Vec3 & angularVelocities);
	//! Metoda przelicza kwaternion do postaci k�t�w Yaw Pitch Roll
	//! \param quat Kwaternion opisuj�cy zmian�
	//! \return Wektor 3D w formie Yaw Pitch Roll
	static IMU::Vec3 quatToYawPitchRoll(const IMU::Quat & quat);


public:
	//! Typ opisuj�cy macierz kowariancji procesu
	//! lewa g�rna macierz 3x3 opisuje kowariancje/szum orientacji
	//! prawa dolna macierz 3x3 opisuje kowariancj�/szum pr�dko�ci kontowych 
	typedef Eigen::Matrix<double, 6, 6> ProcessCovarianceMatrix;
	//! Typ opisuj�cy macierz kowariancji pomiaru
	//! lewa g�rna macierz 3x3 opisuje kowariancj�/szum �yroskopu
	//! �rodkowa macierz 3x3 opisuje kowariancj�/szum akcelerometru
	//! dolna prawa macierz 3x3 opisuje kowariancje/szum magnetometru
	typedef Eigen::Matrix<double, 9, 9> MeasurementCovarianceMatrix;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiar�w
	MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise);

	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiar�w
	//! \param initialState Inicjalny stan procesu
	MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise,
		const StateType & initialState);

	//! Wirtualny destruktor
	virtual ~MJ_UKF_OrientationEstimator();

	//! Metoda resetuje stan estymatora
	virtual void reset();

	//! \param sample Kolejna pr�bka z IMU
	//! \param orientation [out] Estymowana orientacja
	virtual void estimate(const IMU::IMUDataSample & sample, IMU::Vec3 & orientation);
	
	//! \return Nazwa algorytmu
	virtual const std::string name() const;
	//! \return Autor algorytmu (implementacja)
	virtual const std::string author() const;

private:

	//! Wyznacza zmian� orientacji dla znanej pr�dko�ci k�towej i czasu trwania obrotu
	//! \param angularVelocity Pr�dko�c k�towa cia�a
	//! \param dt Czas trwania obrotu cia�a
	//! \return Kwaternion koduj�cy zmian� orientacji
	static const IMU::Quat orientationChange(const IMU::Vec3 & angularVelocity, const double dt);

	//! Metoda zamienia wektor 3D na kwaternion
	//! \param vec Wektor 3D
	//! \return Kwaternion powsta�y z wektora 3D
	static const IMU::Quat convert(const IMU::Vec3 & vec);

	//! Funkcja przej�ci aprocesu
	//! \param previousState Poprzedni stan procesu
	//! \param processCovarianceMatrix Mcierz kowariancji procesu
	//! \param dt Przyrost czasu procesu
	static const StateType processModel(const StateType & previousState,
		const ProcessCovarianceMatrix & processCovarianceMatrix,
		const double dt);

private:
	//! Stan procesu
	StateType state_;
	//! Macierz kowariancji procesu
	ProcessCovarianceMatrix processCovarianceMatrix_;
	//! Macierz szumu procesu
	ProcessCovarianceMatrix processNoiseMatrix_;
	//! Macierz szum�w pomiar�w
	MeasurementCovarianceMatrix measurementsNoiseMatrix_;
	//! Czas pomi�dzy pr�bkami
	double dt_;
};


class CustomNameMJ_UKF_OrientationEstimator : public MJ_UKF_OrientationEstimator
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		//! Konstruktor
		//! \param processNoise Szum procesu
		//! \param measurementsNoise Szum pomiar�w
		CustomNameMJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise, const std::string & name)
		: MJ_UKF_OrientationEstimator(processNoise, measurementsNoise), name_(name)
	{

	}
	/*
	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiar�w
	//! \param initialState Inicjalny stan procesu
	CustomNameMJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise,
		const StateType & initialState, const std::string & name)
		: MJ_UKF_OrientationEstimator(processNoise, measurementsNoise, initialState), name_(name)
	{

	}
	*/
	//! \return Nazwa algorytmu
	virtual const std::string name() const
	{
		return name_;
	}

private:
	//! Nazwa
	std::string name_;
};

#endif	//	HEADER_GUARD___MJ_UKF_ORIENTATIONESTIMATOR_H__
