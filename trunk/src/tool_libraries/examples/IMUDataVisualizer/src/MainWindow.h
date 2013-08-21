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
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QToolButton>

#include "ui_MainWindow.h"
#include "CustomQOSGWidget.h"

#include "VICONDockWidget.h"
#include "XSENSDockWidget.h"

#include <IMU/Data/VICONDataSample.h>
#include <IMU/Data/XSENSDataSample.h>
#include <osgManipulator/TranslateAxisDragger>
#include <osg/Switch>

class MainWindow : public QMainWindow, public Ui::MainWindow
{
	Q_OBJECT

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

	void fitVICON_XSENS();

private:

	const bool parseVICONData(const QString & file, std::list<IMU::VICONDataSample> & data);
	const bool parseXSENSData(const QString & file, std::list<IMU::XSENSDataSample> & data);

private:
	//! Przegl¹darka materia³ów
	CustomQOSGWidget * osgSceneWidget;
	//! Osie uk³adu
	osg::ref_ptr<osgManipulator::TranslateAxisDragger> axis;
	//osgManipulator::TranslateAxisDragger * axis;
	//! G³ówny wêze³ sceny
	osg::ref_ptr<osg::Switch> rootNode;
	//osg::Switch * rootNode;
	//! Progressbar
	QProgressBar * progressBar;
	//! Label dla aktualnie realizowanego zadania
	QLabel * progressLabel;

	VICONDockWidget * viconDock;
	XSENSDockWidget * xsensDock;
};

#endif // HEADER_GUARD__CORE__MAINWINDOW_H__