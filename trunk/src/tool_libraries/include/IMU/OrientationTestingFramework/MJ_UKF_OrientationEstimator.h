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

//! Implementacja algorytmu wg artyku³u: Edgar Kraft "A Quaternion-based
//! Unscented Kalman Filter for Orientation Tracking"
class MJ_UKF_OrientationEstimator : public IMU::IIMUOrietnationEstimator
{
private:
	//! Rozmiar wektora stanu - tutaj 6 ale wynika to z tego
	//! ¿e przyjmujemy i¿ kwaternion ma d³ugoœæ 1 dlatego
	//! jeden stopieñ swobody odpada
	const static unsigned int StateSize = 6;
	//! Rozmiar wektora pomiaru
	const static unsigned int MeasurementSize = 9;
	//! Iloœæ punktów sigma ze wzglêdu na rozmiar wektora stanu
	const static unsigned int SigmaPointsSize = 2 * StateSize + 1;

private:

	//! Typ opisuj¹cy stan procesu
	//! Pierwsze 4 elementy wektora to kwaternion orientacji:
	//! x, y, z, w, zas kolejne 3 to prêdkoœci k¹towe cia³a
	//typedef boost::array<double, 7> StateType;
	
	//! Struktura opisuj¹ca stan procesu
	struct StateType {
		//! Orientacja cia³a
		IMU::Quat orientation;
		//! Prêdkoœci k¹towe cia³¹
		IMU::Vec3 angularVelocities;
	};

	//! Stan w formie wektora
	typedef Eigen::Matrix<double, StateSize, 1> VectorStateType;
	
	//! Agregat punktów sigma stanu
	typedef boost::array<StateType, SigmaPointsSize> SigmaPoints;
	//! Wektor pomiarów
	typedef Eigen::Matrix<double, MeasurementSize, 1> MeasurementsVector;
	//! Agregat projekcji pomiarów dla punktów sigma stanu
	typedef boost::array<MeasurementsVector, SigmaPointsSize> ProjectedMeasurements;
	//! Agregat ró¿nic orientacji wzglêdem estymowanego stanu apriori
	typedef boost::array<IMU::Quat, SigmaPointsSize> OrientationErrors;
	//! Macierz wzmocnienia kalmana
	typedef Eigen::Matrix<double, StateSize, MeasurementSize> KalmanGainMatrix;

private:
	//! \param state Stan cia³a
	//! \return Kwaternion orientacji
	static const IMU::Quat getOrientation(const StateType & state);
	//! \param state Stan cia³a
	//! \return Wektor prêdkoœci k¹towych cia³a
	static const IMU::Vec3 getAngularVelocities(const StateType & state);
	//! \param state Stan cia³a
	//! \param orientation Kwaternion orientacji
	static void updateOrientation(StateType & state, const IMU::Quat & orientation);
	//! \param state Stan cia³a
	//! \param angularVelocities Wektor prêdkoœci k¹towych cia³a
	static void updateAngularVelocities(StateType & state, const IMU::Vec3 & angularVelocities);
	//! Metoda przelicza kwaternion do postaci k¹tów Yaw Pitch Roll
	//! \param quat Kwaternion opisuj¹cy zmianê
	//! \return Wektor 3D w formie Yaw Pitch Roll
	static IMU::Vec3 quatToYawPitchRoll(const IMU::Quat & quat);


public:
	//! Typ opisuj¹cy macierz kowariancji procesu
	//! lewa górna macierz 3x3 opisuje kowariancje/szum orientacji
	//! prawa dolna macierz 3x3 opisuje kowariancjê/szum prêdkoœci kontowych 
	typedef Eigen::Matrix<double, 6, 6> ProcessCovarianceMatrix;
	//! Typ opisuj¹cy macierz kowariancji pomiaru
	//! lewa górna macierz 3x3 opisuje kowariancjê/szum ¿yroskopu
	//! œrodkowa macierz 3x3 opisuje kowariancjê/szum akcelerometru
	//! dolna prawa macierz 3x3 opisuje kowariancje/szum magnetometru
	typedef Eigen::Matrix<double, 9, 9> MeasurementCovarianceMatrix;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiarów
	MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise);

	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiarów
	//! \param initialState Inicjalny stan procesu
	MJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise,
		const StateType & initialState);

	//! Wirtualny destruktor
	virtual ~MJ_UKF_OrientationEstimator();

	//! Metoda resetuje stan estymatora
	virtual void reset();

	//! \param sample Kolejna próbka z IMU
	//! \param orientation [out] Estymowana orientacja
	virtual void estimate(const IMU::IMUDataSample & sample, IMU::Vec3 & orientation);
	
	//! \return Nazwa algorytmu
	virtual const std::string name() const;
	//! \return Autor algorytmu (implementacja)
	virtual const std::string author() const;

private:

	//! Wyznacza zmianê orientacji dla znanej prêdkoœci k¹towej i czasu trwania obrotu
	//! \param angularVelocity Prêdkoœc k¹towa cia³a
	//! \param dt Czas trwania obrotu cia³a
	//! \return Kwaternion koduj¹cy zmianê orientacji
	static const IMU::Quat orientationChange(const IMU::Vec3 & angularVelocity, const double dt);

	//! Metoda zamienia wektor 3D na kwaternion
	//! \param vec Wektor 3D
	//! \return Kwaternion powsta³y z wektora 3D
	static const IMU::Quat convert(const IMU::Vec3 & vec);

	//! Funkcja przejœci aprocesu
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
	//! Macierz szumów pomiarów
	MeasurementCovarianceMatrix measurementsNoiseMatrix_;
	//! Czas pomiêdzy próbkami
	double dt_;
};


class CustomNameMJ_UKF_OrientationEstimator : public MJ_UKF_OrientationEstimator
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		//! Konstruktor
		//! \param processNoise Szum procesu
		//! \param measurementsNoise Szum pomiarów
		CustomNameMJ_UKF_OrientationEstimator(const ProcessCovarianceMatrix & processNoise,
		const MeasurementCovarianceMatrix & measurementsNoise, const std::string & name)
		: MJ_UKF_OrientationEstimator(processNoise, measurementsNoise), name_(name)
	{

	}
	/*
	//! Konstruktor
	//! \param processNoise Szum procesu
	//! \param measurementsNoise Szum pomiarów
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
