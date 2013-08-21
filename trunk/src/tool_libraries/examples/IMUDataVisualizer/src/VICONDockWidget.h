/********************************************************************
    created:  2013/06/25
    created:  25:6:2013   17:45
    filename: VICONDockWidget.h
    author:   Mateusz Janiak
    
    purpose:  Dock widget obs³uguj¹cy dane VICON
*********************************************************************/
#ifndef HEADER_GUARD___VICONDOCKWIDGET_H__
#define HEADER_GUARD___VICONDOCKWIDGET_H__

#include <utils/SmartPtr.h>
#include "IMUWidget.h"
#include "3PointsTStick.h"
#include "PATTStick.h"
#include <QtGui/QDockWidget>
#include <QtGui/QCheckBox>
#include <list>

#include <IMU/Data/VICONDataSample.h>

class VICONDockWidget : public QDockWidget
{
	Q_OBJECT

public:

	typedef std::list<IMU::VICONDataSample> Data;

	typedef utils::shared_ptr<Data> DataPtr;
	typedef utils::shared_ptr<const Data> DataConstPtr;

public:

	VICONDockWidget(osg::Switch * root, QWidget * parent = 0, Qt::WindowFlags f = 0);

	virtual ~VICONDockWidget();

	void setData(DataPtr data);
	void setCurrentData(DataPtr data);

	DataConstPtr data() const;
	DataConstPtr currentData() const;

	PATTStick & stick();

private slots:

	void currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous);
	void onHoldInPlaceChange(int hold);

private:

	DataPtr data_;
	DataPtr currentData_;

	PATTStick stickDrawer_;

	IMUWidget * imuWidget_;

	QCheckBox * holdInPlace;

	bool shift_;
};

#endif	//	HEADER_GUARD___VICONDOCKWIDGET_H__
