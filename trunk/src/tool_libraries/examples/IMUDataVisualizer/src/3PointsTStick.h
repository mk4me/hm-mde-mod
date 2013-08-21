/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   11:41
    filename: 3PointsTStick.h
    author:   Mateusz Janiak
    
    purpose:  Klasa reprezentuj¹ca ró¿d¿kê na scenie 3D w formie 3 punktów
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
	//! Typ opisuj¹cy po³o¿enie ró¿d¿ki, pierwsze dwa elementy to krótszy bok
	//! (lewy, prawy), ostatni to dó³ d³u¿szego boku
	typedef boost::array<IMU::Vec3, 3> StickPositionType;

public:
	//! Domyœlny konstruktor
	_3PointsTStick();
	//! Konstruktor
	//! \param initialPosition Inicjalna pozycja ró¿d¿ki
	_3PointsTStick(const StickPositionType & initialPosition);
	//! Destruktor
	~_3PointsTStick();

	//! \param posiotion Pozycja ró¿d¿ki
	void setPosition(const StickPositionType & position);
	//! \param color Kolor sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresColor(const QColor & color);
	//! \param radius Promieñ sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresRadius(const double radius);
	//! \param color Kolor po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsColor(const QColor & color);
	//! \param radius Promieñ po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsRadius(const double radius);
	//! \return Ró¿d¿ka jako wêze³ do dodania sceny 3D
	virtual osg::Node * asNode();

private:

	//! Inicjalizuje reprezentacjê na scenie 3D
	void initialize();

private:
	//! Grupa wêz³ów dla sceny 3D
	osg::ref_ptr<osg::Group> stickGroup;
	//! Agregat sfer reprezentuj¹cych koñce ró¿d¿ki
	boost::array<SimpleTStickDrawHelper::ShpereDescription, 3> sferEnds;
	//! Agregat sfer reprezentuj¹cych po³¹czenia
	boost::array<SimpleTStickDrawHelper::CylinderDescription, 2> connections;
};

#endif	//	HEADER_GUARD_VISU__3POINTSTSTICK_H__