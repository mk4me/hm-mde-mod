#include "MainWindow.h"
#include <QtCore/QTextCodec>
#include <QtGui/QApplication>
#include <IMU/Algorithms/OrientationFitting.h>
#include <Eigen/Eigen>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	//potrzebujemy polskich znaków
	//QTextCodec::setCodecForCStrings( QTextCodec::codecForName("Windows-1250") );
	// 

	IMU::Quat qA(0.5, 1.0, 0.0, 0.0);
	qA.normalize();

	IMU::Quat qB(0.1, 0.0, 1.0, 1.0);
	qB.normalize();

	IMU::Quat qAB = qB * qA.inverse();

	IMU::Quat qBA = qA * qB.inverse();

	IMU::Quat qA1 = qA * qAB;
	IMU::Quat qA2 = qAB * qA;


	Eigen::Matrix3d m_(Eigen::AngleAxisd(osg::PI_2, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

	Eigen::Vector3d o_ = m_.eulerAngles(0,1,2);

	Eigen::Matrix3d m;
	
	m(0,0) = 1.0;
	m(1,0) = 0.0;
	m(2,0) = 0.0;
	m(0,1) = 0.0;
	m(1,1) = 0.0;
	m(2,1) = -1.0;
	m(0,2) = 0.0;
	m(1,2) = 1.0;
	m(2,2) = 0.0;	

	Eigen::Vector3d o = m.eulerAngles(0,1,2);
	
	Eigen::Matrix3d mm;
	mm(0,0) = 1.0;
	mm(1,0) = 0.0;
	mm(2,0) = 0.0;
	mm(0,1) = 0.0;
	mm(1,1) = 0.0;
	mm(2,1) = 1.0;
	mm(0,2) = 0.0;
	mm(1,2) = -1.0;
	mm(2,2) = 0.0;	

	Eigen::Vector3d oo = m.eulerAngles(0,1,2);


	MainWindow w;
	w.show();
	return a.exec();
}

