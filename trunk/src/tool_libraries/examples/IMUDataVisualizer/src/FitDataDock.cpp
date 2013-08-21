#include "FitDataDock.h"
#include <QuatUtils/QuatUtils.h>
#include <Eigen/StdVector>
#include <IMU/Algorithms/OrientationFitting.h>
#include <QuatUtils/QuatUtils.h>
#include <QtGui/QFileDialog>
#include <QtCore/QFile>
#include <fstream>
#include <boost/lexical_cast.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternion<double>)

Q_DECLARE_METATYPE(IMU::Vec3)

FitDataDock::FitDataDock(osg::Switch * root, QWidget * parent, Qt::WindowFlags f)
	: QDockWidget(parent, f), stickDrawer_(400, 1000), fittingWidget_(new FittingWidget),
	lastOffset_(0), lastStart_(0)
{
	imuWidget_ = new IMUWidget(&stickDrawer_, root);

	stickDrawer_.setConnectionsColor(Qt::green);
	stickDrawer_.setSpheresColor(Qt::yellow);

	QStringList headers;
	headers << "ID" << "Roll" << "Pitch" << "Yaw";

	imuWidget_->dataTree->setColumnCount(headers.size());	
	imuWidget_->dataTree->setHeaderLabels(headers);	
	imuWidget_->dataTree->header()->resizeSections(QHeaderView::ResizeMode::ResizeToContents);	
	imuWidget_->dataTree->header()->setResizeMode(QHeaderView::ResizeToContents);

	imuWidget_->label->setText(tr("Fit XSENS Data"));
	imuWidget_->layout()->addWidget(fittingWidget_);

	connect(fittingWidget_->fitPushButton, SIGNAL(clicked(bool)), this, SLOT(onFit()));

	setWidget(imuWidget_);

	connect(imuWidget_->dataTree, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(currentSampleChanged(QTreeWidgetItem*,QTreeWidgetItem*)));

	saveAction = new QAction(imuWidget_->dataTree);
	saveAction->setText(tr("Zapisz wyniki..."));
	saveAction->setEnabled(false);

	connect(saveAction, SIGNAL(triggered(bool)), this, SLOT(saveData()));

	imuWidget_->dataTree->addAction(saveAction);
	imuWidget_->dataTree->setContextMenuPolicy(Qt::ActionsContextMenu);

	resetFitting();
}

FitDataDock::~FitDataDock()
{

}

PATTStick & FitDataDock::stick()
{
	return stickDrawer_;
}

void FitDataDock::saveData()
{
	auto fileName = QFileDialog::getSaveFileName(this, tr("Zapisz dane po dopasowaniu"));

	//unsigned long int start = boost::lexical_cast<unsigned long int>(fittingWidget_->offsetLabel->text().toStdString());
	unsigned long int stop = std::min<unsigned int>(viconData_->size() - lastStart_, xsensData_->size() - lastOffset_);
	
	std::vector<IMU::Quat> xsensOrientations;
	std::vector<IMU::Quat> viconOrientations;
	std::vector<IMU::Vec3> xsensModifiedOrientations;

	xsensModifiedOrientations.reserve(imuWidget_->dataTree->topLevelItemCount());

	for(unsigned int i = 0; i < imuWidget_->dataTree->topLevelItemCount(); ++i){
		QVariant q = imuWidget_->dataTree->topLevelItem(i)->data(0, Qt::UserRole);
		xsensModifiedOrientations.push_back(q.value<IMU::Vec3>());
	}

	viconOrientations.reserve(viconData_->size());

	for(auto it = viconData_->begin(); it != viconData_->end(); ++it){
		auto euler = IMU::VICONDataSample::estimateOrientation(*it);
		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(euler);
		viconOrientations.push_back(q);
	}

	xsensOrientations.reserve(xsensData_->size());

	for(auto it = xsensData_->begin(); it != xsensData_->end(); ++it){

		IMU::Vec3 rad(
			osg::DegreesToRadians((*it).estimatedOrientationSample().x()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().y()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().z())
			);

		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(rad);
		xsensOrientations.push_back(q);
	}

	std::ofstream out(fileName.toStdString());
	out << "XSENS_Roll;XSENS_Pitch;XSENS_Yaw;XSENS_MOD_Roll;XSENS_MOD_Pitch;XSENS_MOD_Yaw;VICON_Roll;VICON_Pitch;VICON_Yaw;"
	<< std::endl;

	for(auto i = lastStart_; i < stop; ++i){

		auto q = xsensOrientations[i - lastStart_ + lastOffset_];
		IMU::Vec3 xo = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(q);

		IMU::Vec3 xmo = xsensModifiedOrientations[i - lastStart_ + lastOffset_];		

		q = viconOrientations[i];
		IMU::Vec3 vo = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(q);

		out << osg::RadiansToDegrees(xo.x()) << ";"
			<< osg::RadiansToDegrees(xo.y()) << ";"
			<< osg::RadiansToDegrees(xo.z()) << ";"
			<< osg::RadiansToDegrees(xmo.x()) << ";"
			<< osg::RadiansToDegrees(xmo.y()) << ";"
			<< osg::RadiansToDegrees(xmo.z()) << ";"
			<< osg::RadiansToDegrees(vo.x()) << ";"
			<< osg::RadiansToDegrees(vo.y()) << ";"
			<< osg::RadiansToDegrees(vo.z()) << ";" << std::endl;

	}

	out.close();
}

void FitDataDock::setData(const QString & viconFile, VICONDockWidget::DataConstPtr viconData,
	const QString & xsensFile, XSENSDockWidget::DataConstPtr xsensData)
{
	if(viconData == nullptr || xsensData == nullptr){
		throw std::invalid_argument("Niezainicjowane dane do dopasowania");
	}

	if(viconData->empty() == true || xsensData->empty() == true){
		throw std::runtime_error("Puste dane do dopasowania");
	}

	saveAction->setEnabled(true);

	resetFitting();

	viconData_ = viconData;
	xsensData_ = xsensData;

	updateFittingLimits();

	fittingWidget_->xsensPath->setText(xsensFile);
	fittingWidget_->viconPath->setText(viconFile);

	auto s = viconData_->front();

	const double ll = (s.positionM1() - s.positionM4()).norm();
	const double sl = (s.positionM5() - s.positionM6()).norm();

	IMU::Vec3 half = s.positionM5() + (s.positionM6() - s.positionM5()) / 2.0;

	const double o = (s.positionM1() - half).norm();
	const double dist51 = (s.positionM5() - s.positionM1()).norm();
	const double dist16 = (s.positionM6() - s.positionM1()).norm();

	stickDrawer_.setLongLength(ll);
	stickDrawer_.setShortLength(sl);
	stickDrawer_.setOffset( dist51 >= dist16 ? o : -o);
}

void FitDataDock::resetFitting()
{
	stickDrawer_.setAttitude(IMU::Vec3::Zero());
	imuWidget_->dataTree->clear();
	fittingWidget_->xsensPath->clear();
	fittingWidget_->viconPath->clear();
}

void FitDataDock::updateFittingLimits()
{
	fittingWidget_->startIDXSpinBox->setMaximum(viconData_->size());
	fittingWidget_->stepSpinBox->setMaximum(viconData_->size());
	fittingWidget_->fitStepSpinBox->setMaximum(xsensData_->size());
}

void fitData(const std::vector<IMU::Quat> & xsensOrientations,
	const std::vector<IMU::Quat> & viconOrientations,
	std::list<IMU::Vec3> & modifiedData,
	IMU::Quat & rotation,
	unsigned long & offset,
	unsigned int & start, unsigned int step, unsigned int fitStep){

	IMU::Quat tmpRotation(1.0, 0.0, 0.0, 0.0);

	if(xsensOrientations.size() > viconOrientations.size()){
		
		IMU::OrientationFitting::overlap(viconOrientations.begin(),
			viconOrientations.end(), xsensOrientations.begin(),
			xsensOrientations.end(), tmpRotation, offset, step, start, fitStep);

		auto localOffset = offset;
		auto range = step;

		if(localOffset >= step){
			localOffset -= step;
			range += step;
		}
		/*
		if(localOffset + range + viconOrientations.size() > xsensOrientations.size()){
			range = xsensOrientations.size() - viconOrientations.size() - localOffset;
		}

		IMU::OrientationFitting::fitOrientation(viconOrientations.begin(),
			viconOrientations.end(), xsensOrientations.begin() + localOffset,
			range, rotation, offset);

		offset += localOffset;
		*/

		tmpRotation = tmpRotation.inverse();

	}else{
		IMU::OrientationFitting::overlap(xsensOrientations.begin(),
			xsensOrientations.end(), viconOrientations.begin(),
			viconOrientations.end(), tmpRotation, offset, step, start, fitStep);

		auto localOffset = offset;
		auto range = step;

		if(localOffset >= step){
			localOffset -= step;
			range += step;
		}

		/*
		if(localOffset + range + xsensOrientations.size() > viconOrientations.size()){
			range = viconOrientations.size() - xsensOrientations.size() - localOffset;
		}

		IMU::OrientationFitting::fitOrientation(xsensOrientations.begin(),
			xsensOrientations.end(), viconOrientations.begin() + localOffset,
			range, rotation, offset);

		offset += localOffset;
		*/

		start = offset;
		offset = 0;
	}

	//const IMU::OrientationFitting::Vec3 cr = osg::QuatUtils::quaterionToEuler
	//	<IMU::OrientationFitting::Quat, IMU::OrientationFitting::Vec3>(tmpRotation);

	for(auto it = xsensOrientations.begin(); it != xsensOrientations.end(); ++it){
		
		auto o = tmpRotation * (*it);

		IMU::Vec3 e = osg::QuatUtils::quaterionToEuler
			<IMU::Quat, IMU::Vec3>(o);

		modifiedData.push_back(e);

		/*
		IMU::OrientationFitting::Vec3 r = cr + osg::QuatUtils::quaterionToEuler
			<IMU::OrientationFitting::Quat, IMU::OrientationFitting::Vec3>(*it);

		IMU::OrientationFitting::clampEuler(r);

		modifiedData.push_back(r);
		*/
	}

	rotation = tmpRotation;
}

void FitDataDock::onFit()
{
	stickDrawer_.setAttitude(IMU::Vec3::Zero());
	imuWidget_->dataTree->clear();

	std::vector<IMU::Quat> xsensOrientations;
	std::vector<IMU::Quat> viconOrientations;
	std::list<IMU::Vec3> fitData_;
	
	viconOrientations.reserve(viconData_->size());

	for(auto it = viconData_->begin(); it != viconData_->end(); ++it){
		auto euler = IMU::VICONDataSample::estimateOrientation(*it);
		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(euler);
		viconOrientations.push_back(q);
	}

	xsensOrientations.reserve(xsensData_->size());

	for(auto it = xsensData_->begin(); it != xsensData_->end(); ++it){

		IMU::Vec3 rad(
			osg::DegreesToRadians((*it).estimatedOrientationSample().x()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().y()),
			osg::DegreesToRadians((*it).estimatedOrientationSample().z())
			);

		IMU::Quat q = osg::QuatUtils::eulerToQuaternion
			<IMU::Quat, IMU::Vec3>(rad);
		xsensOrientations.push_back(q);
	}

	unsigned long offset = 0;
	IMU::Quat rotation(1.0, 0.0, 0.0, 0.0);
	lastStart_ = 0;
	fitData(xsensOrientations, viconOrientations, fitData_, rotation,
		offset, lastStart_,	fittingWidget_->stepSpinBox->value(),
		fittingWidget_->fitStepSpinBox->value());

	lastOffset_ = fittingWidget_->stepSpinBox->value();

	static const QString rText("x: %1\ty: %2\tz: %3\tw: %4\nroll: %5\tpitch: %6\tyaw: %7");

	IMU::Vec3 euler = osg::QuatUtils::quaterionToEuler
		<IMU::Quat, IMU::Vec3>(rotation);

	fittingWidget_->offsetLabel->setText(QString::number(offset));
	fittingWidget_->rotationLabel->setText(rText.arg(rotation.x()).arg(rotation.y()).arg(rotation.z()).arg(rotation.w()).arg(euler.x()).arg(euler.y()).arg(euler.z()));

	unsigned int idx = 0;

	for(auto it = fitData_.begin(); it != fitData_.end(); ++it, ++idx){
		QTreeWidgetItem * item = new QTreeWidgetItem;

		item->setText(0, QString::number(idx));
		item->setText(1, QString::number((*it).x()));
		item->setText(2, QString::number((*it).y()));
		item->setText(3, QString::number((*it).z()));

		QVariant q;
		q.setValue<IMU::Vec3>(*it);

		item->setData(0, Qt::UserRole, q);

		imuWidget_->dataTree->addTopLevelItem(item);
	}

	if(fitData_.empty() == false){
		imuWidget_->dataTree->setItemSelected(imuWidget_->dataTree->topLevelItem(0), true);
		currentSampleChanged(imuWidget_->dataTree->topLevelItem(0), nullptr);		
	}
}

void FitDataDock::currentSampleChanged(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
	if(current != nullptr){
		QVariant q = current->data(0, Qt::UserRole);
		auto s = q.value<IMU::Vec3>();

		stickDrawer_.setAttitude(s);
		imuWidget_->setLocalAttitude(s);
	}
}