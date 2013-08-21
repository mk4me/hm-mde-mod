/********************************************************************
    created:  2013/06/25
    created:  25:6:2013   17:52
    filename: XSENSDockWidget.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___XSENSDOCKWIDGET_H__
#define HEADER_GUARD___XSENSDOCKWIDGET_H__

#include "IMUWidget.h"
#include "PATTStick.h"
#include <QtGui/QDockWidget>
#include <list>
#include <utils/SmartPtr.h>

#include <IMU/Data/XSENSDataSample.h>

class XSENSDockWidget : public QDockWidget
{
	Q_OBJECT

public:

	typedef std::list<IMU::XSENSDataSample> Data;

	typedef utils::shared_ptr<Data> DataPtr;
	typedef utils::shared_ptr<const Data> DataConstPtr;

public:

	XSENSDockWidget(osg::Switch * root, QWidget * parent = 0, Qt::WindowFlags f = 0);

	virtual ~XSENSDockWidget();

	void setData(DataPtr data);

	void setCurrentData(DataPtr data);

	DataConstPtr data() const;

	DataConstPtr currentData() const;

	PATTStick & stick();

private slots:

	void currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);

private:

	DataPtr data_;
	DataPtr currentData_;

	PATTStick stickDrawer_;

	IMUWidget * imuWidget_;
};

#endif	//	HEADER_GUARD___XSENSDOCKWIDGET_H__
