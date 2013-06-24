#include "MainWindow.h"
#include <QtGui/QDockWidget>
#include <QtGui/QFileDialog>
#include <QtCore/QFile>
#include <QtGui/QMessageBox>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>
#include <QtGui/QFont>
#include <IMU/Parsers/XSENSDataReader.h>
#include <IMU/Parsers/VICONDataReader.h>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/OrbitManipulator>
#include <QtCore/QtCore>
#include <QtGui/QStatusBar>
#include <boost/bind.hpp>

Q_DECLARE_METATYPE(IMU::VICONDataSample)
Q_DECLARE_METATYPE(IMU::XSENSDataSample)

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags), osgSceneWidget(new CustomQOSGWidget),
	xsensVisualization(400, 1000),
	axis(new osgManipulator::TranslateAxisDragger),
	rootNode(new osg::Switch), progressBar(new QProgressBar),
	progressLabel(new QLabel), viconUseCustomPosition(true),
	viconCustomPosition(IMU::VICONDataSample::Vec3::Zero())
{
	setupUi(this);

	// musze to ustawiæ aby klawisz ESC nie wy³¹cza³ renderowania sceny
	osgSceneWidget->setKeyEventSetsDone(0);
	// statystyki sceny
	osgSceneWidget->addEventHandler( new osgViewer::StatsHandler() );

	//ustawiam kolory ró¿d¿ek vicon i xsens
	xsensVisualization.setConnectionsColor(Qt::green);
	viconVisualization.setConnectionsColor(Qt::green);

	xsensVisualization.setSpheresColor(Qt::red);
	viconVisualization.setSpheresColor(Qt::yellow);

	//manipulator sceny - mysz + klawiatura
	auto manipulator = new osgGA::OrbitManipulator;
	// Inicjalna pozycja kamery
	manipulator->setHomePosition(osg::Vec3(0.0, -100.0, 100.0), osg::Vec3(0, 0, 0), osg::Vec3(0,0,1));
	osgSceneWidget->setCameraManipulator(manipulator);

	//domyœlna geometria osi
	axis->setupDefaultGeometry();
	axis->setMatrix(osg::Matrix::scale(100.0, 100.0, 100.0));


	//dodajê wszystkie elementy sceny
	rootNode->addChild(axis);
	rootNode->addChild(xsensVisualization.asNode());
	rootNode->addChild(viconVisualization.asNode());
	rootNode->setAllChildrenOn();

	//ustawiamy scene
	osgSceneWidget->setSceneData(rootNode);	

	//ustawiam scene 3D jako centralny widget
	setCentralWidget(osgSceneWidget);

	//czcionka dla teksów w dokach
	QFont f("Times", 15, QFont::Bold);

	xsensDock.dockWidget = new QDockWidget;
	xsensDock.treeWidget = new QTreeWidget;
	QWidget * w = new QWidget;
	QVBoxLayout * l = new QVBoxLayout;
	QLabel * label = new QLabel("XSENS Data");
	label->setFont(f);
	l->addWidget(label);
	l->addWidget(xsensDock.treeWidget);
	w->setLayout(l);
	xsensDock.dockWidget->setWidget(w);
	xsensDock.treeWidget->setColumnCount(13);
	QStringList xHeaders;
	xHeaders << "ID" << "GyroX" << "GyroY" << "GyroZ" << "AccX" << "AccY" << "AccZ" << "MagX" << "MagY" << "MagZ" << "Roll" << "Pitch" << "Yaw";
	xsensDock.treeWidget->setHeaderLabels(xHeaders);	
	xsensDock.treeWidget->header()->resizeSections(QHeaderView::ResizeMode::ResizeToContents);


	viconDock.dockWidget = new QDockWidget;
	viconDock.treeWidget = new QTreeWidget;
	w = new QWidget;
	l = new QVBoxLayout;
	label = new QLabel("VICON Data");
	label->setFont(f);
	l->addWidget(label);
	l->addWidget(viconDock.treeWidget);
	w->setLayout(l);
	viconDock.dockWidget->setWidget(w);
	viconDock.treeWidget->setColumnCount(7);
	QStringList vHeaders;
	vHeaders << "ID" << "M1" << "M2" << "M3" << "M4" << "M5" << "M6";
	viconDock.treeWidget->setHeaderLabels(vHeaders);	
	viconDock.treeWidget->header()->resizeSections(QHeaderView::ResizeMode::ResizeToContents);

	addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, viconDock.dockWidget);
	addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, xsensDock.dockWidget);

	connect(viconDock.treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(onVICONDataChange(QTreeWidgetItem*,QTreeWidgetItem*)));
	connect(xsensDock.treeWidget, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(onXSENSDataChange(QTreeWidgetItem*,QTreeWidgetItem*)));	

	progressBar->setMinimum(0);
	progressBar->setMaximum(0);
	progressBar->setValue(0);
	progressBar->setTextVisible(false);
	progressBar->setVisible(false);
	QMainWindow::statusBar()->addPermanentWidget(progressBar, 0);	
}

MainWindow::~MainWindow()
{

}

void MainWindow::loadXSENSData()
{
	auto fileName = QFileDialog::getOpenFileName(this, tr("Otwórz plik z danymi XSENS"));

	if(fileName.isEmpty() == true){
		return;
	}

	if(QFile(fileName).exists() == false){
		QMessageBox::critical(this, tr("B³¹d otwarcia pliku"), tr("Podany plik:") + " " + fileName + " " + tr("nie istnieje"));
		return;
	}

	startProcessing(tr("Trwa ³adowanie danych XSENS..."));

	//QtConcurrent::run(boost::bind(&MainWindow::XSENSDataProcessing, this, fileName));
	XSENSDataProcessing(fileName);
}

void MainWindow::loadVICONData()
{
	auto fileName = QFileDialog::getOpenFileName(this, tr("Otwórz plik z danymi VICON"));

	if(fileName.isEmpty() == true){
		return;
	}

	if(QFile(fileName).exists() == false){
		QMessageBox::critical(this, tr("B³¹d otwarcia pliku"), tr("Podany plik:") + " " + fileName + " " + tr("nie istnieje"));
		return;
	}

	startProcessing(tr("Trwa ³adowanie danych VICON..."));

	//QtConcurrent::run(boost::bind(&MainWindow::VICONDataProcessing, this, fileName));
	VICONDataProcessing(fileName);
}

const bool MainWindow::parseVICONData(const QString & file, std::list<IMU::VICONDataSample> & data)
{
	bool ret = true;

	IMU::VICONDataReader viconReader(file.toStdString());
	IMU::VICONDataSample viconSample;
	bool read = true;
	while(read == true){

		auto status = viconReader.readNextSample(viconSample);

		if(status == IMU::UniversalDataReader<IMU::VICONDataSample>::RESULT_OK){
			data.push_back(viconSample);
		}else if(status == IMU::UniversalDataReader<IMU::VICONDataSample>::DATA_FINISHED){
			read = false;
		}else{
			read = false;
			ret = false;
		}		
	}

	return ret;
}

const bool MainWindow::parseXSENSData(const QString & file, std::list<IMU::XSENSDataSample> & data)
{
	bool ret = true;

	IMU::XSENSDataReader xsensReader(file.toStdString());
	IMU::XSENSDataSample xsensSample;
	bool read = true;
	while(read == true){

		auto status = xsensReader.readNextSample(xsensSample);

		if(status == IMU::UniversalDataReader<IMU::XSENSDataSample>::RESULT_OK){
			data.push_back(xsensSample);
		}else if(status == IMU::UniversalDataReader<IMU::XSENSDataSample>::DATA_FINISHED){
			read = false;
		}else{
			read = false;
			ret = false;
		}		
	}

	return ret;
}

inline QString posToQString(const IMU::VICONDataSample::Vec3 & position)
{
	std::stringstream ss;

	ss << "(" << position.x() << "; " << position.y() << "; " << position.z() << ")";

	return QString::fromStdString(ss.str());
}

void MainWindow::refreshVICONData(QTreeWidget * tree)
{	
	for(auto it = viconData.begin(); it != viconData.end(); ++it){
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
		tree->addTopLevelItem(item);
	}
}

void MainWindow::refreshXSENSData(QTreeWidget * tree)
{
	for(auto it = xsensData.begin(); it != xsensData.end(); ++it){
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
		tree->addTopLevelItem(item);
	}
}


void MainWindow::onVICONDataChange(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
	if(current != nullptr){
		QVariant q = current->data(0, Qt::UserRole);
		auto s = q.value<IMU::VICONDataSample>();
		_3PointsTStick::StickPositionType pos;
		pos[0] = s.positionM5();
		pos[1] = s.positionM6();
		pos[2] = s.positionM4();

		if(viconUseCustomPosition == true){
			for(unsigned int i = 0; i < 3; ++i){
				pos[i] -= s.positionM1() + viconCustomPosition;
			}
		}

		viconVisualization.setPosition(pos);
	}
}

void MainWindow::onXSENSDataChange(QTreeWidgetItem * current, QTreeWidgetItem * previous)
{
	if(current != nullptr){
		QVariant q = current->data(0, Qt::UserRole);
		auto s = q.value<IMU::XSENSDataSample>();

		xsensVisualization.setAttitude(IMU::XSENSDataSample::Vec3(
			osg::DegreesToRadians(s.estimatedOrientationSample().x()),
			osg::DegreesToRadians(s.estimatedOrientationSample().y()),
			osg::DegreesToRadians(s.estimatedOrientationSample().z())			
			));
	}
}

void MainWindow::XSENSDataProcessing( const QString & fileName )
{
	std::list<IMU::XSENSDataSample> tmpList;

	bool result = parseXSENSData(fileName, tmpList);

	if(result == false){
		if(QMessageBox::question(this, tr("Problem z parsowaniem danych XSENS"), tr("U¿yæ danych które uda³o siê sparsowaæ czy anulowaæ ³adowanie danych?"), QMessageBox::Discard, QMessageBox::Yes) == QMessageBox::Discard){
			finishProcessing(tr("Anulowano ³adowanie danych XSENS"));
			return;
		}
	}

	xsensData.swap(tmpList);

	refreshXSENSData(xsensDock.treeWidget);

	if(xsensData.empty() == false){
		onXSENSDataChange(xsensDock.treeWidget->topLevelItem(0), nullptr);		
		xsensDock.treeWidget->setItemSelected(xsensDock.treeWidget->topLevelItem(0), true);
	}

	finishProcessing(tr("Za³adowano dane XSENS"));	
}

void MainWindow::VICONDataProcessing( const QString & fileName )
{
	std::list<IMU::VICONDataSample> tmpList;
	bool result = parseVICONData(fileName, tmpList);

	if(result == false){
		if(QMessageBox::question(this, tr("Problem z parsowaniem danych VICON"), tr("U¿yæ danych które uda³o siê sparsowaæ czy anulowaæ ³adowanie danych?"), QMessageBox::Discard, QMessageBox::Yes) == QMessageBox::Discard){
			finishProcessing(tr("Anulowano ³adowanie danych VICON"));			
			return;
		}
	}

	viconData.swap(tmpList);

	refreshVICONData(viconDock.treeWidget);

	//ustawiam d³ugoœci i offsety ró¿d¿ki dla XSENS
	if(viconData.empty() == false){

		onVICONDataChange(viconDock.treeWidget->topLevelItem(0), nullptr);
		viconDock.treeWidget->setItemSelected(viconDock.treeWidget->topLevelItem(0), true);

		auto s = viconData.front();

		const double ll = (s.positionM1() - s.positionM4()).norm();
		const double sl = (s.positionM5() - s.positionM6()).norm();

		IMU::VICONDataSample::Vec3 half = s.positionM5() + (s.positionM6() - s.positionM5()) / 2.0;

		const double o = (s.positionM1() - half).norm();

		xsensVisualization.setLongLength(ll);
		xsensVisualization.setShortLength(sl);
		xsensVisualization.setOffset(o);
	}

	finishProcessing(tr("Za³adowano dane VICON"));	
}

void MainWindow::finishProcessing(const QString & message)
{
	QApplication::restoreOverrideCursor();
	progressBar->setVisible(false);
	QMainWindow::statusBar()->showMessage(message, 2000);
}

void MainWindow::startProcessing(const QString & message)
{
	QApplication::setOverrideCursor(Qt::WaitCursor);
	progressBar->setVisible(true);
	QMainWindow::statusBar()->showMessage(message);
}
