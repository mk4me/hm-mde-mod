/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   16:25
    filename: IMUDataSample.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj¹ca próbki danych z IMU
*********************************************************************/
#ifndef HEADER_GUARD___IMUDATASAMPLE_H__
#define HEADER_GUARD___IMUDATASAMPLE_H__

#include <Eigen/Core>

namespace IMU {

class IMUDataSample
{
public:
	//! Typ reprezentuj¹cy próbkê danych pomiarowych 3D
	typedef Eigen::Matrix<double, 3, 1> Vec3;
	//! Typ reprezentuj¹cy identyfikator czasu z IMU
	typedef unsigned long long int TimeIDType;

public:

	//! Konstruktor
	//! \param timeID Identyfikator czasu z IMU dla danej próbki
	//! \param sequenceID Identyfikator próbki
	//! \param accelerometerSample Odczyt akcelerometru [m / s^2]
	//! \param gyroscopeSample Odczyt ¿yroskopu [radiamy]
	//! \param magnetometerSample Odczyt magnetometru [!!TODO - wyjaœniæ, wartoœci <-1;1>!!]
	IMUDataSample(TimeIDType timeID, TimeIDType sequenceID, const Vec3 & accelerometerSample,
		const Vec3 & gyroscopeSample, const Vec3 & magnetometerSample)
		: timeID_(timeID), sequenceID_(sequenceID), accelerometerSample_(accelerometerSample),
		gyroscopeSample_(gyroscopeSample), magnetometerSample_(magnetometerSample) {}

	//! Konstruktor kopiuj¹cy
	//! \param ds Kopiowana próbka IMU
	IMUDataSample(const IMUDataSample & ds) : timeID_(ds.timeID_), sequenceID_(ds.sequenceID_),
		accelerometerSample_(ds.accelerometerSample_), gyroscopeSample_(ds.gyroscopeSample_),
		magnetometerSample_(ds.magnetometerSample_) {}

	//! Domyœlny konstruktor
	IMUDataSample() : timeID_(0), sequenceID_(0), accelerometerSample_(Vec3::Zero()), gyroscopeSample_(Vec3::Zero()),
		magnetometerSample_(Vec3::Zero()) {}

	//! Destruktor
	~IMUDataSample() {}


	//! \return Identyfikator czasu próbki
	inline TimeIDType timeID() const { return timeID_; }
	//! \return Identyfikator próbki
	inline TimeIDType sequenceID() const { return sequenceID_; }
	//! \return Odczyt akcelerometru
	inline const Vec3 &  accelerometerSample() const { return accelerometerSample_; }
	//! \return Odczyt ¿yroskopu
	inline const Vec3 &  gyroscopeSample() const { return gyroscopeSample_; }
	//! \return Odczyt magnetometru
	inline const Vec3 &  magnetometerSample() const { return magnetometerSample_; }

	//! \param timeID Identyfikator czasu
	void setTimeID(TimeIDType timeID) { timeID_ = timeID; }
	//! \param sequenceID Identyfikator próbki
	void setSequenceID(TimeIDType sequenceID) { sequenceID_ = sequenceID; }
	//! \param accelerometerSample Dane akcelerometru
	void setAccelerometerSample(const Vec3 & accelerometerSample) { accelerometerSample_ = accelerometerSample; }
	//! \param gyroscopeSample Dane ¿yroskopu
	void setGyroscopeSample(const Vec3 &  gyroscopeSample) { gyroscopeSample_ = gyroscopeSample; }
	//! \param magnetometerSample Dane magnetometru
	void setMagnetometerSample(const Vec3 &  magnetometerSample) { magnetometerSample_ = magnetometerSample; }

private:
	//! Identyfikator czasu próbki
	TimeIDType timeID_;
	//! Identyfikator próbki
	TimeIDType sequenceID_;
	//! Odczyt akcelerometru
	Vec3 accelerometerSample_;
	//! Odczyt ¿yroskopu
	Vec3 gyroscopeSample_;
	//! Odczyt magnetometru
	Vec3 magnetometerSample_;

};

}

#endif	//	HEADER_GUARD___IMUDATASAMPLE_H__
