/********************************************************************
    created:  2013/06/25
    created:  25:6:2013   17:36
    filename: IMUWidget.h
    author:   Mateusz Janiak
    
    purpose:  Widget pomagaj¹cy zarz¹dzaæ wizualizacj¹ naszej ró¿d¿ki
*********************************************************************/
#ifndef HEADER_GUARD___IMUWIDGET_H__
#define HEADER_GUARD___IMUWIDGET_H__

#include <QtGui/QWidget>
#include "ui_IMUWidget.h"
#include "ITStick.h"
#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include <IMU/Data/IMUDataSample.h>

class IMUWidget : public QWidget, public Ui::IMUWidget
{
	Q_OBJECT

public:
	IMUWidget(ITStick * stick, osg::Switch * root, QWidget * parent = 0, Qt::WindowFlags f = 0);

	virtual ~IMUWidget();

	void setLocalAttitude(const IMU::Vec3 & localAttitude);

private slots:

	void updateConnectionsView();
	void updateSpheresView();

	void updateAttitude();
	void updatePosition();

	void switchVisibility(bool visible);

private:
	//! Ró¿d¿ka do zarzadzania
	ITStick * stick_;
	//! PAT dla ró¿d¿ki
	osg::ref_ptr<osg::PositionAttitudeTransform> pat_;
	//! Root sceny
	osg::ref_ptr<osg::Switch> root_;
};

#endif	//	HEADER_GUARD___IMUWIDGET_H__
