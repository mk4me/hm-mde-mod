/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   11:41
    filename: 3PointsTStick.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj�ca r�d�k� na scenie 3D w formie 3 punkt�w
*********************************************************************/
#ifndef HEADER_GUARD_VISU__3POINTSTSTICK_H__
#define HEADER_GUARD_VISU__3POINTSTSTICK_H__

#include "ITStick.h"

#include <IMU/Data/VICONDataSample.h>
#include <boost/array.hpp>
#include "TStickDrawHelpers.h"

class _3PointsTStick : public ITStick
{
public:
	//! Typ opisuj�cy po�o�enie r�d�ki, pierwsze dwa elementy to kr�tszy bok
	//! (lewy, prawy), ostatni to d� d�u�szego boku
	typedef boost::array<IMU::Vec3, 3> StickPositionType;

public:
	//! Domy�lny konstruktor
	_3PointsTStick();
	//! Konstruktor
	//! \param initialPosition Inicjalna pozycja r�d�ki
	_3PointsTStick(const StickPositionType & initialPosition);
	//! Destruktor
	~_3PointsTStick();

	//! \param posiotion Pozycja r�d�ki
	void setPosition(const StickPositionType & position);
	//! \param color Kolor sfer reprezentuj�cych ko�ce r�d�ki
	virtual void setSpheresColor(const QColor & color);
	//! \param radius Promie� sfer reprezentuj�cych ko�ce r�d�ki
	virtual void setSpheresRadius(const double radius);
	//! \param color Kolor po��cze� w r�d�ce
	virtual void setConnectionsColor(const QColor & color);
	//! \param radius Promie� po��cze� w r�d�ce
	virtual void setConnectionsRadius(const double radius);
	//! \return R�d�ka jako w�ze� do dodania sceny 3D
	virtual osg::Node * asNode();

private:

	//! Inicjalizuje reprezentacj� na scenie 3D
	void initialize();

private:
	//! Grupa w�z��w dla sceny 3D
	osg::ref_ptr<osg::Group> stickGroup;
	//! Agregat sfer reprezentuj�cych ko�ce r�d�ki
	boost::array<SimpleTStickDrawHelper::ShpereDescription, 3> sferEnds;
	//! Agregat sfer reprezentuj�cych po��czenia
	boost::array<SimpleTStickDrawHelper::CylinderDescription, 2> connections;
};

#endif	//	HEADER_GUARD_VISU__3POINTSTSTICK_H__