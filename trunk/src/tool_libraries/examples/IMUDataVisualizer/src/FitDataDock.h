/********************************************************************
    created:  2013/06/26
    created:  26:6:2013   9:28
    filename: FitDataDock.h
    author:   Mateusz Janiak
    
    purpose:  Klasa realizuj¹ca wizualizacjê danych po dopasowaniu
*********************************************************************/
#ifndef HEADER_GUARD___FITDATADOCK_H__
#define HEADER_GUARD___FITDATADOCK_H__

#include "IMUWidget.h"
#include "PATTStick.h"
#include <QtGui/QDockWidget>
#include <list>

#include <IMU/Data/VICONDataSample.h>
#include <IMU/Data/XSENSDataSample.h>
#include "FittingWidget.h"
#include <VICONDockWidget.h>
#include <XSENSDockWidget.h>
#include <QtGui/QAction>

class FitDataDock : public QDockWidget
{
	Q_OBJECT

public:

	FitDataDock(osg::Switch * root, QWidget * parent = 0, Qt::WindowFlags f = 0);

	virtual ~FitDataDock();
	
	PATTStick & stick();

	void setData(const QString & viconFile, VICONDockWidget::DataConstPtr viconData,
		const QString & xsensFile, XSENSDockWidget::DataConstPtr xsensData);

private:

	void resetFitting();
	void updateFittingLimits();

private slots:

	void currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);
	void onFit();

	void saveData();

private:	

	PATTStick stickDrawer_;

	IMUWidget * imuWidget_;

	FittingWidget * fittingWidget_;

	QAction * saveAction;

	VICONDockWidget::DataConstPtr viconData_;
	XSENSDockWidget::DataConstPtr xsensData_;
	unsigned long int lastOffset_;
	unsigned int lastStart_;
};

#endif	//	HEADER_GUARD___FITDATADOCK_H__
