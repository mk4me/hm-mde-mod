/********************************************************************
    created:  2013/05/11
    created:  11:5:2013   16:25
    filename: VICONDataSample.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj¹ca próbki danych z systemu VICON dla eksperymentu.
				Próbki przedstawiaj¹ kolejne pozycje markerów u¿ytych do
				eksperymentu (³¹cznie 6). Markery naklejone by³y wg dostarczonego
				schematu (kszta³t T). Pliki dla rozró¿nienia powinny mieæ
				roszerzenie ".ref".
*********************************************************************/
#ifndef HEADER_GUARD___VICONDATASAMPLE_H__
#define HEADER_GUARD___VICONDATASAMPLE_H__

#include <Eigen/Core>
#include <boost/array.hpp>


namespace IMU {

class VICONDataSample
{
public:
	//! Typ reprezentuj¹cy pozycjê 3D
	typedef Eigen::Vector3d Vec3;
	//! Typ reprezentuj¹cy identyfikator czasu z VICON
	typedef unsigned long long int TimeIDType;
	//! Typ reprezentuj¹cy pozycje markerów ró¿d¿ki
	typedef boost::array<Vec3, 6> MarkerPositionsType;

public:

	//! Konstruktor
	//! \param timeID Identyfikator czasu z VICON dla danej próbki	
	//! \param positionM1 Pozycja markera M1 w laboratorium wg odczytów systemu VICON [mm]
	//! \param positionM2 Pozycja markera M2 w laboratorium wg odczytów systemu VICON [mm]
	//! \param positionM3 Pozycja markera M3 w laboratorium wg odczytów systemu VICON [mm]
	//! \param positionM4 Pozycja markera M4 w laboratorium wg odczytów systemu VICON [mm]
	//! \param positionM5 Pozycja markera M5 w laboratorium wg odczytów systemu VICON [mm]
	//! \param positionM6 Pozycja markera M6 w laboratorium wg odczytów systemu VICON [mm]
	VICONDataSample(TimeIDType timeID, const Vec3 & positionM1,	const Vec3 & positionM2,
		const Vec3 & positionM3, const Vec3 & positionM4, const Vec3 & positionM5,
		const Vec3 & positionM6);

	//! Konstruktor kopiuj¹cy
	//! \param ds Kopiowana próbka VICON
	VICONDataSample(const VICONDataSample & ds);

	//! Domyœlny konstruktor
	VICONDataSample();

	//! Destruktor
	~VICONDataSample();

	//! \return Identyfikator czasu próbki
	inline TimeIDType timeID() const { return timeID_; }
	//! \return Pozycja markera M1
	inline const Vec3 &  positionM1() const { return markerPositions_[0]; }
	//! \return Pozycja markera M2
	inline const Vec3 &  positionM2() const { return markerPositions_[1]; }
	//! \return Pozycja markera M3
	inline const Vec3 &  positionM3() const { return markerPositions_[2]; }
	//! \return Pozycja markera M4
	inline const Vec3 &  positionM4() const { return markerPositions_[3]; }
	//! \return Pozycja markera M5
	inline const Vec3 &  positionM5() const { return markerPositions_[4]; }
	//! \return Pozycja markera M6
	inline const Vec3 &  positionM6() const { return markerPositions_[5]; }

	//! \param timeID Identyfikator czasu
	void setTimeID(TimeIDType timeID) { timeID_ = timeID; }
	//! \param positionM1 Pozycja markera M1 w laboratorium wg odczytów systemu VICON [mm]
	void setPositionM1(const Vec3 & positionM1) { markerPositions_[0] = positionM1; }
	//! \param positionM2 Pozycja markera M2 w laboratorium wg odczytów systemu VICON [mm]
	void setPositionM2(const Vec3 &  positionM2) { markerPositions_[1] = positionM2; }
	//! \param positionM3 Pozycja markera M3 w laboratorium wg odczytów systemu VICON [mm]	
	void setPositionM3(const Vec3 &  positionM3) { markerPositions_[2] = positionM3; }
	//! \param positionM4 Pozycja markera M4 w laboratorium wg odczytów systemu VICON [mm]
	void setPositionM4(const Vec3 &  positionM4) { markerPositions_[3] = positionM4; }
	//! \param positionM5 Pozycja markera M5 w laboratorium wg odczytów systemu VICON [mm]
	void setPositionM5(const Vec3 &  positionM5) { markerPositions_[4] = positionM5; }
	//! \param positionM6 Pozycja markera M6 w laboratorium wg odczytów systemu VICON [mm]
	void setPositionM6(const Vec3 &  positionM6) { markerPositions_[5] = positionM6; }
	//! Metoda statyczna wyznaczaj¹ca œredni¹ orientacjê cia³¹ dla zadanego pomiaru
	//! Generuje wszystkie mo¿liwe trójki punktów z których generuje wektory p³aszczyzn i uœrednia je
	static const Vec3 estimateOrientation(const VICONDataSample & viconSample);

private:
	//! Identyfikator czasu próbki
	TimeIDType timeID_;
	//! Pozycje markerów
	MarkerPositionsType markerPositions_;	
};

}

#endif	//	HEADER_GUARD___VICONDATASAMPLE_H__

