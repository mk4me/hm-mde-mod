/********************************************************************
	created:  2013/06/02
	created:  2:6:2013   10:26
	filename: MainWindow.h
	author:   Przemys³aw Bartkowiak
	
	purpose:  Plik trzymajacy uchwyt do glownego okna.
				Z tego pliku beda sie rozchodzily sygnaly do innych
				okien poto aby je wlaczyc, obsluzyc i zamknac.
*********************************************************************/
#ifndef HEADER_GUARD__CORE__MAINWINDOW_H__
#define HEADER_GUARD__CORE__MAINWINDOW_H__

#include <QtGui/QMainWindow>
#include <QtGui/QTreeWidget>
#include <QtGui/QProgressBar>
#include <QtGui/QLabel>

#include "ui_MainWindow.h"
#include "CustomQOSGWidget.h"
#include "3PointsTStick.h"
#include "PATTStick.h"

#include <IMU/Data/VICONDataSample.h>
#include <IMU/Data/XSENSDataSample.h>
#include <osgManipulator/TranslateAxisDragger>
#include <osg/Switch>


class MainWindow : public QMainWindow, public Ui::MainWindow
{
	Q_OBJECT

private:

	struct DockDescription
	{
		QDockWidget * dockWidget;
		QTreeWidget * treeWidget;
	};

public:
	MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0);
	virtual ~MainWindow();

private slots:

	void loadXSENSData();

	void startProcessing(const QString & message);

	void XSENSDataProcessing( const QString & fileName );

	void finishProcessing(const QString & message);

	void loadVICONData();

	void VICONDataProcessing( const QString & fileName );

	void onVICONDataChange(QTreeWidgetItem * current, QTreeWidgetItem * previous);
	void onXSENSDataChange(QTreeWidgetItem * current, QTreeWidgetItem * previous);

private:

	const bool parseVICONData(const QString & file, std::list<IMU::VICONDataSample> & data);
	const bool parseXSENSData(const QString & file, std::list<IMU::XSENSDataSample> & data);

	void refreshVICONData(QTreeWidget * tree);
	void refreshXSENSData(QTreeWidget * tree);

private:
	//! Przegl¹darka materia³ów
	CustomQOSGWidget * osgSceneWidget;
	//! Dock dla danych VICON
	DockDescription viconDock;
	//! Dock dla danych XSENS
	DockDescription xsensDock;
	//! Dane vicon
	std::list<IMU::VICONDataSample> viconData;
	//! Dane xsens
	std::list<IMU::XSENSDataSample> xsensData;
	//! Wizualizacja dla VICON
	_3PointsTStick viconVisualization;
	//! Wizualizacja dla XSENS
	PATTStick xsensVisualization;
	//! Osie uk³adu
	osg::ref_ptr<osgManipulator::TranslateAxisDragger> axis;
	//! G³ówny wêze³ sceny
	osg::ref_ptr<osg::Switch> rootNode;
	//! Progressbar
	QProgressBar * progressBar;
	//! Label dla aktualnie realizowanego zadania
	QLabel * progressLabel;
	//! Czy dane z VICON maj¹ byc pokazywane w ich uk³adzie wspó³rzednych
	//! czy przesuniête do zadanego punktu (marker M1)
	bool viconUseCustomPosition;
	//! Wybrana pozycja dla danych VICON
	IMU::VICONDataSample::Vec3 viconCustomPosition;
};

#endif // HEADER_GUARD__CORE__MAINWINDOW_H__