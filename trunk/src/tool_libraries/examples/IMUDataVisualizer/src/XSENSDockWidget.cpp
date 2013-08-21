#include "XSENSDockWidget.h"

Q_DECLARE_METATYPE(IMU::XSENSDataSample)

XSENSDockWidget::XSENSDockWidget(osg::Switch * root, QWidget * parent, Qt::WindowFlags f)
	: QDockWidget(parent, f), stickDrawer_(400, 1000)
{
	imuWidget_ = new IMUWidget(&stickDrawer_, root);

	stickDrawer_.setConnectionsColor(Qt::green);
	stickDrawer_.setSpheresColor(Qt::red);

	QStringList headers;
	headers << "ID" << "GyroX" << "GyroY" << "GyroZ" << "AccX" << "AccY" << "AccZ" << "MagX" << "MagY" << "MagZ" << "Roll" << "Pitch" << "Yaw";

	imuWidget_->dataTree->setColumnCount(headers.size());	
	imuWidget_->dataTree->setHeaderLabels(headers);	
	imuWidget_->dataTree->header()->resizeSections(QHeaderView::ResizeMode::ResizeToContents);	
	imuWidget_->dataTree->header()->setResizeMode(QHeaderView::ResizeToContents);

	imuWidget_->label->setText(tr("XSENS Data"));

	setWidget(imuWidget_);
	
	connect(imuWidget_->dataTree, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(currentSampleChanged(QTreeWidgetItem*,QTreeWidgetItem*)));
}

XSENSDockWidget::~XSENSDockWidget()
{

}

PATTStick & XSENSDockWidget::stick()
{
	return stickDrawer_;
}

void XSENSDockWidget::setData(DataPtr data)
{
	data_ = data;
	setCurrentData(data_);
}

void XSENSDockWidget::setCurrentData(DataPtr data)
{
	currentData_ = data;
	imuWidget_->dataTree->clear();

	for(auto it = currentData_->begin(); it != currentData_->end(); ++it){
		auto item = new QTreeWidgetItem;
		item->setText(0, QString::number((*it).timeID()));
		item->setText(1, QString::number((*it).gyroscopeSample().x()));
		item->setText(2, QString::number((*it).gyroscopeSample().y()));
		item->setText(3, QString::number((*it).gyroscopeSample().z()));
		item->setText(4, QString::number((*it).accelerometerSample().x()));
		item->setText(5, QString::number((*it).accelerometerSample().y()));
		item->setText(6, QString::number((*it).accelerometerSample().z()));
		item->setText(7, QString::number((*it).magnetometerSample().x()));
		item->setText(8, QString::number((*it).magnetometerSample().y()));
		item->setText(9, QString::number((*it).magnetometerSample().z()));
		item->setText(10, QString::number((*it).estimatedOrientationSample().x()));
		item->setText(11, QString::number((*it).estimatedOrientationSample().y()));
		item->setText(12, QString::number((*it).estimatedOrientationSample().z()));
		QVariant q;
		q.setValue(*it);
		item->setData(0, Qt::UserRole, q);
		imuWidget_->dataTree->addTopLevelItem(item);
	}

	if(currentData_->empty() == false){
		imuWidget_->dataTree->setItemSelected(imuWidget_->dataTree->topLevelItem(0), true);
		currentSampleChanged(imuWidget_->dataTree->topLevelItem(0), nullptr);
	}
}

XSENSDockWidget::DataConstPtr XSENSDockWidget::data() const
{
	return data_;
}

XSENSDockWidget::DataConstPtr XSENSDockWidget::currentData() const
{
	return currentData_;
}

void XSENSDockWidget::currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
	if(current != nullptr){
		QVariant q = current->data(0, Qt::UserRole);
		auto s = q.value<IMU::XSENSDataSample>();

		IMU::Vec3 orientation(
			osg::DegreesToRadians(s.estimatedOrientationSample().x()),
			osg::DegreesToRadians(s.estimatedOrientationSample().y()),
			osg::DegreesToRadians(s.estimatedOrientationSample().z())
			);


		stickDrawer_.setAttitude(orientation);

		imuWidget_->setLocalAttitude(orientation);
	}
}