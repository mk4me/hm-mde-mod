/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   12:05
    filename: ITStick.h
    author:   Mateusz Janiak
    
    purpose:  Interfejs opisuj¹cy podstawowe operacje zwi¹zane z wygl¹dem
				ró¿d¿ki
*********************************************************************/
#ifndef HEADER_GUARD___ITSTICK_H__
#define HEADER_GUARD___ITSTICK_H__

#include <QtGui/QColor>
#include <osg/Node>

class ITStick
{
public:
	//! Destruktor wirtualny
	virtual ~ITStick() {}

	//! \param color Kolor sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresColor(const QColor & color) = 0;
	//! \param radius Promieñ sfer reprezentuj¹cych koñce ró¿d¿ki
	virtual void setSpheresRadius(const double radius) = 0;
	//! \param color Kolor po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsColor(const QColor & color) = 0;
	//! \param radius Promieñ po³¹czeñ w ró¿d¿ce
	virtual void setConnectionsRadius(const double radius) = 0;
	//! \return Ró¿d¿ka jako wêze³ sceny 3D
	virtual osg::Node * asNode() = 0;
};

#endif	//	HEADER_GUARD___ITSTICK_H__
