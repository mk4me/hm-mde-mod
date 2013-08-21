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
#include "FitDataDock.h"
#include <IMU/Algorithms/OrientationFitting.h>
#include <QuatUtils/QuatUtils.h>
#include <IMU/Data/XSENSDataSample.h>
#include "FittingWizzard.h"
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(IMU::Quat)

Q_DECLARE_METATYPE(IMU::VICONDataSample)
Q_DECLARE_METATYPE(IMU::XSENSDataSample)

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags), osgSceneWidget(new CustomQOSGWidget),
	axis(new osgManipulator::TranslateAxisDragger),
	rootNode(new osg::Switch), progressBar(new QProgressBar),
	progressLabel(new QLabel)
{
	setupUi(this);

	// musze to ustawiæ aby klawisz ESC nie wy³¹cza³ renderowania sceny
	osgSceneWidget->setKeyEventSetsDone(0);
	// statystyki sceny
	osgSceneWidget->addEventHandler( new osgViewer::StatsHandler() );

	//manipulator sceny - mysz + klawiatura
	auto manipulator = new osgGA::OrbitManipulator;
	// Inicjalna pozycja kamery
	manipulator->setHomePosition(osg::Vec3(0.0, -100.0, 100.0), osg::Vec3(0, 0, 0), osg::Vec3(0,0,1));
	osgSceneWidget->setCameraManipulator(manipulator);

	viconDock = new VICONDockWidget(rootNode, this);
	xsensDock = new XSENSDockWidget(rootNode, this);

	//domyœlna geometria osi
	axis->setupDefaultGeometry();
	axis->setMatrix(osg::Matrix::scale(100.0, 100.0, 100.0));

	//dodajê wszystkie elementy sceny
	rootNode->addChild(axis);
	rootNode->setAllChildrenOn();

	//ustawiamy scene
	osgSceneWidget->setSceneData(rootNode);	

	//ustawiam scene 3D jako centralny widget
	setCentralWidget(osgSceneWidget);

	addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, viconDock);
	addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, xsensDock);

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

	xsensDock->setWindowTitle(fileName);
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

	viconDock->setWindowTitle(fileName);
}

const bool MainWindow::parseVICONData(const QString & file, std::list<IMU::VICONDataSample> & data)
{
	bool ret = true;

	IMU::VICONDataReader viconReader(file.toStdString());
	IMU::VICONDataSample viconSample;
	bool read = true;
	while(read == true){

		auto status = viconReader.readNextSample(viconSample);

		if(status == IMU::UniversalDataReaderBase::RESULT_OK){
			data.push_back(viconSample);
		}else if(status == IMU::UniversalDataReaderBase::DATA_FINISHED){
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

		if(status == IMU::UniversalDataReaderBase::RESULT_OK){
			data.push_back(xsensSample);
		}else if(status == IMU::UniversalDataReaderBase::DATA_FINISHED){
			read = false;
		}else{
			read = false;
			ret = false;
		}		
	}

	return ret;
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

	xsensDock->setData(XSENSDockWidget::DataPtr(new XSENSDockWidget::Data(tmpList)));

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
	
	viconDock->setData(VICONDockWidget::DataPtr(new VICONDockWidget::Data(tmpList)));

	//ustawiam d³ugoœci i offsety ró¿d¿ki dla XSENS
	if(tmpList.empty() == false){

		auto s = tmpList.front();

		const double ll = (s.positionM1() - s.positionM4()).norm();
		const double sl = (s.positionM5() - s.positionM6()).norm();

		IMU::Vec3 half = s.positionM5() + (s.positionM6() - s.positionM5()) / 2.0;

		const double o = (s.positionM1() - half).norm();
		const double dist51 = (s.positionM5() - s.positionM1()).norm();
		const double dist16 = (s.positionM6() - s.positionM1()).norm();

		xsensDock->stick().setLongLength(ll);
		xsensDock->stick().setShortLength(sl);
		xsensDock->stick().setOffset( dist51 >= dist16 ? o : -o);

		viconDock->stick().setLongLength(ll);
		viconDock->stick().setShortLength(sl);
		viconDock->stick().setOffset( dist51 >= dist16 ? o : -o);		
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

void MainWindow::fitVICON_XSENS()
{
	
	if(viconDock->data() == nullptr || xsensDock->data() == nullptr || viconDock->data()->empty() == true || xsensDock->data()->empty() == true){
		QMessageBox::warning(this, tr("Dane niekompletne"), tr("Na potrzeby dopasowania potrzebne s¹ dane XSENS i VICON. Uzupe³nij dane i spróbuj ponownie"));
		return;
	}

	FittingWizard fw(xsensDock->data()->size(), viconDock->data()->size(), this);
	//FittingWizard fw(100, 100, this);
	int res = fw.exec();

	if(res == QDialog::Accepted){

		//krok pierwszy - dopasowujemy sygna³y ze wzglêdu na offset, przycinamy je
		int xsensOffset = fw.offset();
		
		if(fw.offsetAutodetection() == true){
			
			//krok pierwszy - zamieniam oba sygna³y z reprezentacji orientacji na lokalrne rotacje

			bool lengthsOk = true;
			std::vector<IMU::Quat> viconRotations;
			std::vector<IMU::Quat> xsensRotations;

			//VICON
			{

				std::vector<IMU::Quat> viconOrientations;
				viconOrientations.reserve(viconDock->data()->size());
				viconRotations.reserve(viconDock->data()->size() - 1);

				//wyci¹gam orientacje w formie kwaternionów
				for(auto it = viconDock->data()->begin(); it != viconDock->data()->end(); ++it){
					viconOrientations.push_back( osg::QuatUtils::eulerToQuaternion
						<IMU::Quat, IMU::Vec3>(IMU::VICONDataSample::estimateOrientation(*it)));

				}

				//zamieniam orientacje na lokalne rotacje
				osg::QuatUtils::convertOrientationsToRotations(viconRotations,
					viconOrientations.begin(), viconOrientations.end());
			}

			//XSENS (czujnik)
			{
				std::vector<IMU::Quat> xsensOrientations;
				xsensOrientations.reserve(xsensDock->data()->size());
				xsensRotations.reserve(xsensDock->data()->size() - 1);

				//wyci¹gam orientacje jako kwaterniony
				for(auto it = xsensDock->data()->begin(); it != xsensDock->data()->end(); ++it){
					xsensOrientations.push_back(osg::QuatUtils::eulerToQuaternion
						<IMU::Quat, IMU::Vec3>((*it).estimatedOrientationSample()));
				}

				//konwertuje do lokalnych rotacji
				osg::QuatUtils::convertOrientationsToRotations(xsensRotations,
					xsensOrientations.begin(), xsensOrientations.end());
			}


			//sprawdzam który sygna³ jest d³u¿szy - zak³adam ¿e VICON, ale jeœli XSENS
			//to zamieniam je wtedy i pamiêtam ¿eby uzyskany offset zamieniæ na ujemny
			if(xsensRotations.size() > viconRotations.size()){
				lengthsOk = false;
				std::swap(xsensRotations, viconRotations);
			}

			//krok drugi - porównujê sugna³y w reprezentacji lokalnych rotacji
			{				
				// !!! WA¯NE !!!
				// porównujemy xsens do vicon!!
				double minAvgError = std::numeric_limits<double>::max();

				auto xS = xsensRotations.begin();
				auto xE = xS;
				std::advance(xE, fw.minimalMatchDistance());

				auto fxS = xS;
				std::advance(fxS, xsensRotations.size() - fw.minimalMatchDistance());

				do{

					const auto l = std::distance(xS, xE);
					const auto ll = std::distance(xS, xsensRotations.end());

					auto locDestStart = viconRotations.begin();
					auto locDestEnd = viconRotations.end();
					std::advance(locDestEnd, -l);

					while(locDestStart != locDestEnd){

						const auto cxS = xS;
						const double error = osg::QuatUtils::quatsDifference(cxS, xsensRotations.end(), locDestStart, viconRotations.end()).first / (double)ll;

						if(error < minAvgError){
							minAvgError = error;
							xsensOffset = std::distance(viconRotations.begin(), locDestStart) - std::distance(xsensRotations.begin(), xS);
						}

						++locDestStart;						
					}

					if(xS == fxS && xE == xsensRotations.end()){
						break;
					}

					//aktualizujemy porównywany zakres
					if(xE != xsensRotations.end()){
						++xE;
					}else{
						++xS;
					}

				}while(true);
			}

			if(lengthsOk == false){
				xsensOffset = -xsensOffset;
			}
		}

		//klonuje przyciête sygna³y
		XSENSDockWidget::DataPtr xD;
		XSENSDockWidget::DataPtr mxD;
		VICONDockWidget::DataPtr vD;

		{		
			int vS = std::max(xsensOffset, 0);
			int vE = std::min(viconDock->data()->size()-1, xsensDock->data()->size() + xsensOffset - 1);

			int xS = std::max(-xsensOffset, 0);
			int xE = std::min(xsensDock->data()->size()-1, viconDock->data()->size()-1);

			auto itVS = viconDock->data()->begin();
			std::advance(itVS, vS);
			auto itVE = viconDock->data()->begin();
			std::advance(itVE, vE);


			auto itXS = xsensDock->data()->begin();
			std::advance(itXS, xS);
			auto itXE = xsensDock->data()->begin();
			std::advance(itXE, xE);

			xD.reset(new XSENSDockWidget::Data(itXS, itXE));
			mxD.reset(new XSENSDockWidget::Data(itXS, itXE));
			vD.reset(new VICONDockWidget::Data(itVS, itVE));

		}

		//znajdujemy rotacjê pomiêdzy pomiarami czujnika i vicon
		//mam dany jeden obrót - uk³ad odniesienia czujnika do uk³adu odniesienia vicon
		// albo lokalne u³o¿enie czujnika na ró¿d¿ce
		// to wystarczy by wyznaczyæ drugi obrót i przemianowaæ dane orientacji z czujnika
		// do systemu vicon

		auto q = fw.orientation();

		std::vector<IMU::Quat> viconOrientations;
		std::vector<IMU::Quat> xsensOrientations;
		std::vector<IMU::Quat> diffRotations;

		//orientacje przyciêtych sygna³ów
		{	
			viconOrientations.reserve(vD->size());		

			//wyci¹gam orientacje w formie kwaternionów
			for(auto it = vD->begin(); it != vD->end(); ++it){
				viconOrientations.push_back( osg::QuatUtils::eulerToQuaternion
					<IMU::Quat, IMU::Vec3>(IMU::VICONDataSample::estimateOrientation(*it)));

			}
		
			xsensOrientations.reserve(xD->size());		

			//wyci¹gam orientacje jako kwaterniony
			for(auto it = xD->begin(); it != xD->end(); ++it){
				xsensOrientations.push_back(osg::QuatUtils::eulerToQuaternion
					<IMU::Quat, IMU::Vec3>((*it).estimatedOrientationSample()));
			}
		}

		//sygna³y powinny mieæ tak¹ sam¹ d³ugoœæ
		assert(xD->size() == vD->size());

		/*
		diffRotations.reserve(xD->size());

		//mam ró¿nicê pomiêdzy sygna³em vicona i xsensa
		for(unsigned int i = 0; i < xD->size() - 1; ++i){
			diffRotations.push_back(xsensOrientations[i].inverse() * viconOrientations[i]);
		}

		//teraz u¿ywamy rotacji któr¹ podano
		if(fw.inputOrientationType() == FittingWizard::GLOBAL_SENSOR_FRAME){
			auto qMissing = diffRotations[0] * q;
			auto invMissing = qMissing.inverse();
			//dopasowuje reszte

			int i = 0;

		}else{
			auto qMissing = q * diffRotations[0];
			auto invMissing = qMissing.inverse();
			//dopasowuje reszte

			int i = 0;
		}*/

		double minError = std::numeric_limits<double>::max();
		IMU::Quat totalRot(1.0, 0.0, 0.0, 0.0);

		for(unsigned int i = 0; i < viconOrientations.size() - 1; ++i){

			auto difR = viconOrientations[i] * xsensOrientations[i].inverse();

			std::vector<IMU::Quat> tmpXsens(xsensOrientations);

			for(auto it = tmpXsens.begin(); it != tmpXsens.end(); ++it){
				(*it) = difR * (*it);
			}

			double error = osg::QuatUtils::quatsDifference(viconOrientations.begin(),
				viconOrientations.end(), tmpXsens.begin(), tmpXsens.end()).first;

			if(error < minError){
				minError = error;
				totalRot = difR;
			}
		}

		for(auto it = mxD->begin(); it != mxD->end(); ++it){

			(*it).setEstimatedOrientationSample(totalRot * (*it).estimatedOrientationSample());

		}

		xsensDock->setCurrentData(xD);
		viconDock->setCurrentData(vD);





		//prezentujemy wyniki

		/*
		auto fDock = new FitDataDock(rootNode, this);
		fDock->setData(viconDock->windowTitle(), viconDock->data(),
			xsensDock->windowTitle(), xsensDock->data());
		
		addDockWidget(Qt::RightDockWidgetArea, fDock);
		*/

		auto dock = new XSENSDockWidget(rootNode, this);
		dock->setWindowTitle("Fit data");
		dock->setData(mxD);

		addDockWidget(Qt::RightDockWidgetArea, dock);
	}
}