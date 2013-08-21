/********************************************************************
    created:  2013/07/22
    created:  22:7:2013   8:50
    filename: OrientationInput.h
    author:   Mateusz Janiak
    
    purpose:  Klasa widgeta obs³uguj¹cego wprowadzanie orientacji
*********************************************************************/
#ifndef HEADER_GUARD___ORIENTATIONINPUT_H__
#define HEADER_GUARD___ORIENTATIONINPUT_H__

#include <QtGui/QDialog>
#include "ui_OrientationInput.h"
#include <osg/Quat>

class OrientationInput : public QDialog, private Ui::OrientationInput
{
	Q_OBJECT

public:

	OrientationInput(QWidget * parent = nullptr, Qt::WindowFlags f = 0);

	virtual ~OrientationInput();

	const osg::Quat & orientation() const;

private slots:

	void quaternionChanged();
	void cardanChanged();

private:

	void updateQuaternionLimits();
	void updateCardanAngles();
	void updateQuaternion();

private:

	osg::Quat orientation_;
};

#endif	//	HEADER_GUARD___ORIENTATIONINPUT_H__
