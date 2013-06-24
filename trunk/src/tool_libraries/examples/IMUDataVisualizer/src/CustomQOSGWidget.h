/********************************************************************
    created:  2013/06/22
    created:  22:6:2013   11:25
    filename: CustomQOSGWidget.h
    author:   Mateusz Janiak
    
    purpose:  Klasa ³¹cz¹ca kontekst sceny 3D z OSG razem z kontekstem Qt
*********************************************************************/
#ifndef HEADER_GUARD_VISU__CUSTOMQOSGWIDGET_H__
#define HEADER_GUARD_VISU__CUSTOMQOSGWIDGET_H__

#include <osgui/QOsgWidgets.h>
#include <QtCore/QMutex>
#include <boost/function.hpp>
#include <list>

class CustomQOSGWidget : public osgui::QOsgDefaultWidget
{
public:
	//! Funktor realizowany w w¹tku OSG
	typedef boost::function<void()> Functor;

public:
	//! Konstruktor
	CustomQOSGWidget(QWidget * parent = 0, const osg::GraphicsContext::Traits* traits = 0, Qt::WindowFlags f = 0);
	//! Destruktor wirtualny
	virtual ~CustomQOSGWidget();

	//! \param func Operacja wykonana w w¹tku UI przed renderowanie kolejnej klatki
	void addOperation(Functor func);

protected:

	//! Przeci¹¿amy metodê odrysowuj¹c¹ klatkê by uwzglêdniæ operacje na scenie
	void paintEvent( QPaintEvent* event );

private:
	//! Lista operacji do wykonania
	std::list<Functor> operations;
	//! Mutex synchronizuj¹cy rendering z dodawaniem operacji
	QMutex sync;

};

#endif	//	HEADER_GUARD_VISU__CUSTOMQOSGWIDGET_H__
