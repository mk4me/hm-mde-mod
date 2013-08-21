#include "VICONDockWidget.h"
#include <QuatUtils/QuatUtils.h>
#include <IMU/Algorithms/OrientationFitting.h>
#include <QuatUtils/QuatUtils.h>

Q_DECLARE_METATYPE(IMU::VICONDataSample)

VICONDockWidget::VICONDockWidget(osg::Switch * root, QWidget * parent, Qt::WindowFlags f)
	: QDockWidget(parent, f), holdInPlace(new QCheckBox), shift_(false), stickDrawer_(400, 1000)
{
	imuWidget_ = new IMUWidget(&stickDrawer_, root);

	stickDrawer_.setConnectionsColor(Qt::green);
	stickDrawer_.setSpheresColor(Qt::blue);

	QStringList headers;
	headers << "ID" << "M1" << "M2" << "M3" << "M4" << "M5" << "M6";

	imuWidget_->dataTree->setColumnCount(headers.size());	
	imuWidget_->dataTree->setHeaderLabels(headers);	
	imuWidget_->dataTree->header()->resizeSections(QHeaderView::ResizeMode::ResizeToContents);
	imuWidget_->dataTree->header()->setResizeMode(QHeaderView::ResizeToContents);

	imuWidget_->label->setText(tr("VICON Data"));

	holdInPlace->setText(tr("Utrzymaj wzglêdem M1 w centrum"));
	holdInPlace->setCheckable(true);
	holdInPlace->setCheckState(Qt::CheckState::Unchecked);

	connect(holdInPlace, SIGNAL(stateChanged(int)), this, SLOT(onHoldInPlaceChange(int)));
	imuWidget_->layout()->addWidget(holdInPlace);

	setWidget(imuWidget_);

	connect(imuWidget_->dataTree, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(currentSampleChanged(QTreeWidgetItem*,QTreeWidgetItem*)));
}

VICONDockWidget::~VICONDockWidget()
{

}

void VICONDockWidget::onHoldInPlaceChange(int hold)
{
	if(hold == Qt::CheckState::Checked){
		shift_ = true;
	}else{
		shift_ = false;
	}

	auto selectedItems = imuWidget_->dataTree->selectedItems();
	if(selectedItems.isEmpty() == false){
		currentSampleChanged(selectedItems.front(), nullptr);
	}
}

inline QString posToQString(const IMU::Vec3 & position)
{
	std::stringstream ss;

	ss << "(" << position.x() << "; " << position.y() << "; " << position.z() << ")";

	return QString::fromStdString(ss.str());
}

void VICONDockWidget::setData(DataPtr data)
{
	data_ = data;
	setCurrentData(data_);
}

void VICONDockWidget::setCurrentData(DataPtr data)
{
	currentData_ = data;
	imuWidget_->dataTree->clear();

	for(auto it = currentData_->begin(); it != currentData_->end(); ++it){
		auto item = new QTreeWidgetItem;
		item->setText(0, QString::number((*it).timeID()));
		item->setText(1, posToQString((*it).positionM1()));
		item->setText(2, posToQString((*it).positionM2()));
		item->setText(3, posToQString((*it).positionM3()));
		item->setText(4, posToQString((*it).positionM4()));
		item->setText(5, posToQString((*it).positionM5()));
		item->setText(6, posToQString((*it).positionM6()));	
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

VICONDockWidget::DataConstPtr VICONDockWidget::data() const
{
	return data_;
}

VICONDockWidget::DataConstPtr VICONDockWidget::currentData() const
{
	return currentData_;
}

PATTStick & VICONDockWidget::stick()
{
	return stickDrawer_;
}

void VICONDockWidget::currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
	if(current != nullptr){
		QVariant q = current->data(0, Qt::UserRole);
		auto s = q.value<IMU::VICONDataSample>();
		
		IMU::Vec3 x = (s.positionM6() - s.positionM5()).normalized();
		IMU::Vec3 y = (s.positionM1() - s.positionM4()).normalized();
		IMU::Vec3 z = x.cross(y);

		IMU::Vec3 o = osg::QuatUtils::axisToEuler
			<IMU::Vec3,IMU::Vec3>(x, y, z);		
		
		if(shift_ == false){
			stickDrawer_.setPosition(s.positionM1());
		}else{
			stickDrawer_.setPosition(IMU::Vec3::Zero());
		}

		stickDrawer_.setAttitude(o);
		imuWidget_->setLocalAttitude(o);

		/*
		_3PointsTStick::StickPositionType pos;
		pos[0] = s.positionM5();
		pos[1] = s.positionM6();
		pos[2] = s.positionM4();

		if(shift_ == true){
			for(unsigned int i = 0; i < 3; ++i){
				pos[i] -= s.positionM1();
			}
		}

		IMU::VICONDataSample::Vec3 o2 = IMU::VICONDataSample::estimateOrientation(s);

		stickDrawer2_.setPosition(pos);
		*/		
		
	}
}