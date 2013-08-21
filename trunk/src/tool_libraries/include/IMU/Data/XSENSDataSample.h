/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   16:25
    filename: XSENSDataSample.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj¹ca próbki danych z modu³u XSENS
*********************************************************************/
#ifndef HEADER_GUARD___XSENSDATASAMPLE_H__
#define HEADER_GUARD___XSENSDATASAMPLE_H__

#include <IMU/Data/Types.h>

namespace IMU {

class XSENSDataSample
{
public:
	//! Typ reprezentuj¹cy identyfikator czasu z XSENS
	typedef unsigned long long int TimeIDType;

public:

	//! Konstruktor
	//! \param timeID Identyfikator czasu z XSENS dla danej próbki	
	//! \param accelerometerSample Odczyt akcelerometru [m / s^2]
	//! \param gyroscopeSample Odczyt ¿yroskopu [radiamy]
	//! \param magnetometerSample Odczyt magnetometru [!!TODO - wyjaœniæ, wartoœci <-1;1>!!]
	//! \param estimatedOrientationSample Estymowana przez XSENS orientacja dla pozosta³ych danych
	XSENSDataSample(TimeIDType timeID, const Vec3 & accelerometerSample,
		const Vec3 & gyroscopeSample, const Vec3 & magnetometerSample,
		const Vec3 & estimatedOrientationSample)
		: timeID_(timeID), accelerometerSample_(accelerometerSample),
		gyroscopeSample_(gyroscopeSample), magnetometerSample_(magnetometerSample),
		estimatedOrientationSample_(estimatedOrientationSample) {}

	//! Konstruktor kopiuj¹cy
	//! \param ds Kopiowana próbka XSENS
	XSENSDataSample(const XSENSDataSample & ds) : timeID_(ds.timeID_), accelerometerSample_(ds.accelerometerSample_),
		gyroscopeSample_(ds.gyroscopeSample_), magnetometerSample_(ds.magnetometerSample_),
		estimatedOrientationSample_(ds.estimatedOrientationSample_)	{}

	//! Domyœlny konstruktor
	XSENSDataSample() : timeID_(0), accelerometerSample_(Vec3::Zero()), gyroscopeSample_(Vec3::Zero()),
		magnetometerSample_(Vec3::Zero()), estimatedOrientationSample_(Vec3::Zero()) {}

	//! Destruktor
	~XSENSDataSample() {}


	//! \return Identyfikator czasu próbki
	inline TimeIDType timeID() const { return timeID_; }
	//! \return Odczyt akcelerometru
	inline const Vec3 &  accelerometerSample() const { return accelerometerSample_; }
	//! \return Odczyt ¿yroskopu
	inline const Vec3 &  gyroscopeSample() const { return gyroscopeSample_; }
	//! \return Odczyt magnetometru
	inline const Vec3 &  magnetometerSample() const { return magnetometerSample_; }
	//! \return Estymowana orientacja
	inline const Vec3 &  estimatedOrientationSample() const { return estimatedOrientationSample_; }

	//! \param timeID Identyfikator czasu
	void setTimeID(TimeIDType timeID) { timeID_ = timeID; }
	//! \param accelerometerSample Dane akcelerometru
	void setAccelerometerSample(const Vec3 & accelerometerSample) { accelerometerSample_ = accelerometerSample; }
	//! \param gyroscopeSample Dane ¿yroskopu
	void setGyroscopeSample(const Vec3 &  gyroscopeSample) { gyroscopeSample_ = gyroscopeSample; }
	//! \param magnetometerSample Dane magnetometru
	void setMagnetometerSample(const Vec3 &  magnetometerSample) { magnetometerSample_ = magnetometerSample; }
	//! \param estimatedOrientationSample Estymowana przez XSENS orientacja
	void setEstimatedOrientationSample(const Vec3 &  estimatedOrientationSample) { estimatedOrientationSample_ = estimatedOrientationSample; }

private:
	//! Identyfikator czasu próbki
	TimeIDType timeID_;
	//! Odczyt akcelerometru
	Vec3 accelerometerSample_;
	//! Odczyt ¿yroskopu
	Vec3 gyroscopeSample_;
	//! Odczyt magnetometru
	Vec3 magnetometerSample_;
	//! Estymowana orientacja
	Vec3 estimatedOrientationSample_;

};

}

#endif	//	HEADER_GUARD___XSENSDATASAMPLE_H__
