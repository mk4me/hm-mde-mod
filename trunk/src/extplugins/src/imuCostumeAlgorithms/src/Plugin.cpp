#include "Plugin.h"
#include <corelib/IPlugin.h>
#include <corelib/IDataHierarchyManagerReader.h>
#include <corelib/ISourceManager.h>
#include <plugins/imuCostume/IIMUDataSource.h>
#include <plugins/imuCostume/IIMUOrientationEstimationAlgorithm.h>
#include <plugins/imuCostume/IMUCostumeCalibrationAlgorithm.h>
#include <plugins/imuCostume/IMUCostumeMotionEstimationAlgorithm.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include "filter_lib\lib_main.h"

#include "CQuatIO.h"

class DummyCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-0000000A563F}");

public:
	DummyCalibrationAlgorithm() {}
	virtual ~DummyCalibrationAlgorithm(){}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeCalibrationAlgorithm * create() const override { return new DummyCalibrationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "DummyCalibrationAlgorithm"; }

	//! Resets algo internal state
	virtual void reset() override { SensorsAdjustemnts().swap(sa); }

	//! Returns max number (n) of steps that algorithm might require to calibrate costume, 0 means no upper limit
	virtual unsigned int maxCalibrationSteps() const override { return 1; }

	//! \param skeleton	Kalibrowany szkielet
	//! \param mapping Mapowanie sensorów do szkieletu
	//! \param sensorsAdjustment Wstêpne ustawienie sensorów - pozwala zadaæ stan pocz¹tkowy bardziej zbli¿ony do rzeczywistoœci
	//! \param calibrationData Dane kalibracyjne
	virtual void initialize(kinematic::SkeletonConstPtr skeleton, const IMU::SensorsMapping & mapping,
		const SensorsAdjustemnts  & sensorsAdjustment = SensorsAdjustemnts()) override
	{
		sa = sensorsAdjustment;
		if (sa.empty() == true){
			for (const auto & m : mapping)
			{
				SensorAdjustment s;
				s.offset = osg::Vec3d(0, 0, 0);
				s.rotation = osg::Quat(0, 0, 0, 1);
				sa.insert(SensorsAdjustemnts::value_type(m.get_left(), s));
			}
		}
	}

	//! Calculates orientation from sensor fusion
	/*!
	\param data Dane z IMU
	\param inDeltaT Czas od poprzedniej ramki danych
	\return Returns if calibration successful and sufficient.
	*/
	virtual bool calibrate(const IMU::SensorsData & data, const double inDeltaT) override
	{
		return true;
	}

	//! \return Dane kalibracyjne szkieletu, poprawki dla sensorów
	virtual SensorsAdjustemnts sensorsAdjustemnts() const override
	{
		return sa;
	}

private:

	SensorsAdjustemnts sa;
};

class DummyMotionEstimationAlgorithm : public IMU::IMUCostumeMotionEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-1000000A563F}")

public:
	DummyMotionEstimationAlgorithm() {}
	//! Destruktor wirtualny
	virtual ~DummyMotionEstimationAlgorithm() {}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeMotionEstimationAlgorithm * create() const override { return new DummyMotionEstimationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "DummyMotionEstimationAlgorithm"; }

	//! Resets algo internal state
	virtual void reset() override {}

	//! \param skeleton	Kalibrowany szkielet
	virtual void initialize(kinematic::SkeletonConstPtr skeleton,
		const IMU::IMUCostumeCalibrationAlgorithm::SensorsAdjustemnts & sensorsAdjustment,
		const IMU::SensorsMapping & sensorsMapping) override
	{
		this->skeleton = skeleton;
		this->sensorsAdjustment = sensorsAdjustment;
		this->sensorsMapping = sensorsMapping;
	}

	//! Calculates orientation from sensor fusion
	/*!
	\param data Dane z IMU
	\param inDeltaT Czas od poprzedniej ramki danych
	\return Returns Lokalne orientacje wszystkich jointów, bez end-sitów
	*/
	virtual MotionState estimate(const MotionState & motionState,
		const IMU::SensorsData & data, const double inDeltaT) override
	{
		return motionState;
	}

private:
	kinematic::SkeletonConstPtr skeleton;
	IMU::IMUCostumeCalibrationAlgorithm::SensorsAdjustemnts sensorsAdjustment;
	IMU::SensorsMapping sensorsMapping;
};

//! OSG visualizer
CQuatIO quatWriter(true);

//! Generic quaternion-based orientation filter - generates orientation as a quaternion using IMU sensor fusion
class DummyOrientationEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-2000000A563F}")

private:
	std::ofstream _myLogFile;
	std::ofstream _myRawFile;
	std::stringstream _ssMyLogFile;
	std::stringstream _ssMyRawFile;
	boost::posix_time::ptime _lastTick;
	//boost::mutex _estimateMutex;
	static unsigned int _IntID;	// TODO: remove me
	unsigned int _thisID;
	unsigned int _sampleNum;
	std::list<osg::Quat> _accum;
	std::unique_ptr<ImuFilters::IOrientationFilter> _internalFilterImpl;

public:
	DummyOrientationEstimationAlgorithm() : _internalFilterImpl(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_INSTANTENOUS_KALMAN))
		//_internalFilterImpl(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_AQKF_KALMAN))
	{
		//_internalFilterImpl.swap(std::move(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_INSTANTENOUS_KALMAN)));
		std::string logName = "DOEA_log_" + std::to_string(_IntID) + ".txt";
		std::string rawDataName = "RAW_log_" + std::to_string(_IntID) + ".txt";
		_myLogFile.open(logName, std::ios::trunc);
		_myRawFile.open(rawDataName, std::ios::trunc);
		_lastTick = boost::posix_time::microsec_clock::local_time();
		_sampleNum = 0;
		_thisID = ++_IntID;
	}

	//! Make it polymorphic
	virtual ~DummyOrientationEstimationAlgorithm() 
	{
		_myLogFile.close();
		_myRawFile.close();
	}

	//! \return Nowy algorytm estymacji
	virtual IIMUOrientationEstimationAlgorithm * create() const override 
	{
		return new DummyOrientationEstimationAlgorithm; 
	};

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override 
	{ 
		return "DummyOrientationEstimationAlgorithm(Inst)";
		//return "DummyOrientationEstimationAlgorithm(AQKf)"; 
	}

	//! Resets filter internal state
	virtual void reset() override
	{
		_internalFilterImpl->reset();
	}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override 
	{ 
		return _internalFilterImpl->approximateEstimationDelay();
	}

	//! Calculates orientation from sensor fusion
	/*!
		\param inAcc accelerometer vector from IMU
		\param inGyro gyroscope vector from IMU
		\param inMag magnetometer vector from IMU
		\param inDeltaT time between acquisitions in seconds [s] from IMU sensor
		\return Returns estimated orientation.
	*/
	virtual osg::Quat estimate(const osg::Vec3d& inAcc, const osg::Vec3d& inGyro,
		const osg::Vec3d& inMag, const double inDeltaT, const osg::Quat & orient) override
	{
		//boost::unique_lock<boost::mutex> superLock(_estimateMutex);

		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;

		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		//osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, inDeltaT);
		osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, myTime);
		quatWriter.SetQuat(retQ, _thisID);

		_ssMyRawFile << inAcc.x() << "\t" << inAcc.y() << "\t" << inAcc.z() << "\t" <<
					  inGyro.x() << "\t" << inGyro.y() << "\t" << inGyro.z() << "\t" <<
					  inMag.x() << "\t" << inMag.y() << "\t" << inMag.z() << "\t" << std::endl;

		//double accLen = inAcc.length();
		//double gyroLen = inGyro.length();
		//double magLen = inMag.length();
		//double sum = accLen + gyroLen + magLen;
		//do sprawdzenia:
		//-œrednia ruchoma na wektorach(ile próbek ? ) - œrednia sk³adowych - jak siê to ma do d³ugoœci ? (nie moge zdeformowac tych wektorow)
		//-sprawdzic dane przemka i moje - dlugosci wektorów (acc moze mieæ zmienne ale ok. 10 w spoczynku, ¿yro - co to? magnetometr - ko³o 1, bo kierunek?)
		//-plot w matlabie wartosci przema i moich z raw sprzetu  - np fukcja do kreslenia 3 skladowych - plot_vec

		// SMA
		//_accum.push_back(retQ);
		//if (_accum.size() > 5)
		//	_accum.pop_front();

		//osg::Quat startQ = retQ;
		//osg::Quat finalQ = osg::Quat(0.0, 0.0, 0.0, 1.0);
		//for each (osg::Quat tmpQ in _accum)
		//{
		//	finalQ = tmpQ * finalQ;
		//}
		//retQ.slerp(0.5, startQ, finalQ);

		//_myLogFile << "[ThreadID:]" << boost::this_thread::get_id() << "\t" << inDeltaT << "\t" << elapsedMicroSec.total_milliseconds() << std::endl;
		_ssMyLogFile << _sampleNum << "\t" << inDeltaT << "\t" << myTime << "\t" << 
			retQ.x() << "\t" << retQ.y() << "\t" << retQ.z() << "\t" <<  retQ.w() << std::endl;

		// Write log
		if ((_sampleNum % 200) == 0)
		{
			_myRawFile << _ssMyRawFile.str();
			_myLogFile << _ssMyLogFile.str();
			_ssMyRawFile.str("");
			_ssMyLogFile.str("");
			_myRawFile.flush();
			_myLogFile.flush();
		}

		++_sampleNum;

		return retQ;
	}
};

unsigned int DummyOrientationEstimationAlgorithm::_IntID = 0; // TODO: remove me
volatile bool PluginHelper::finish = false;
core::IDataManagerReader::ObjectObserverPtr PluginHelper::objectObserver = core::IDataManagerReader::ObjectObserverPtr();
core::Thread PluginHelper::streamQuerryingThread = core::Thread();
IMU::CostumeSkeletonMotionConstPtr PluginHelper::skeletonMotion = IMU::CostumeSkeletonMotionConstPtr();
utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver> PluginHelper::streamObserver = utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver>();

class MotionDataObserver : public core::IDataManagerReader::IObjectObserver
{
public:
	MotionDataObserver() {}
	virtual ~MotionDataObserver() {}

	virtual void observe(const core::IDataManagerReader::ChangeList & changes) override
	{
		//pobieram manager hierarchi danych do sprawdzenia czy typ danych mi odpowiada
		auto dhm = plugin::getDataHierachyManagerReader();
		//lece po wszystkich zmianach
		for (const auto & c : changes)
		{
			//czy zmiana to dodanie danych i czy typ pasuje
			if (c.modyfication == core::IDataManagerReader::ADD_OBJECT &&
				dhm->isTypeCompatible(c.type, typeid(IMU::CostumeSkeletonMotion)) == true){

				PluginHelper::skeletonMotion = c.currentValue->get();
				//PluginHelper::streamObserver.reset(new threadingUtils::ResetableStreamStatusObserver);
				//PluginHelper::skeletonMotion->stream->attachObserver(PluginHelper::streamObserver);
				//PluginHelper::streamQuerryingThread.run(&PluginHelper::run);
				break;
			}
		}
	}
};

kinematic::JointPtr createJoint(kinematic::JointPtr parent,
	const std::string & name, const osg::Vec3 & position,
	const osg::Quat & orientation = osg::Quat(0, 0, 0, 1))
{
	kinematic::JointData jd;
	jd.name = name;
	jd.orientation = orientation;
	jd.position = position;

	auto ret = kinematic::Joint::create(jd);	
	if (parent != nullptr){
		parent->children.push_back(ret);
	}

	return ret;
}

osg::Vec3 norm(const osg::Vec3 & in)
{
	return in / 183.0;
}

bool PluginHelper::init()
{
	bool ret = true;

	try{
		//streamQuerryingThread = plugin::getThreadPool()->get("imuCOstumeAlgorithms", "Costume Stream Processing");
		auto imuDS = core::querySource<IMU::IIMUDataSource>(plugin::getSourceManager());
		if (imuDS == nullptr){
			ret = false;
		}
		else{			
			imuDS->registerCostumeCalibrationAlgorithm(new DummyCalibrationAlgorithm);
			imuDS->registerMotionEstimationAlgorithm(new DummyMotionEstimationAlgorithm);
			imuDS->registerOrientationEstimationAlgorithm(new DummyOrientationEstimationAlgorithm);
			//szkielet
			auto dummySkeleton = utils::make_shared<kinematic::Skeleton>();
			dummySkeleton->name = "DummySkeleton";
			dummySkeleton->root = createJoint(kinematic::JointPtr(), "HumanoidRoot", norm(osg::Vec3(0, 0, 0)));

			//lewa noga
			auto j = createJoint(dummySkeleton->root, "l_hip", norm(osg::Vec3(16, 0, 0)));
			j = createJoint(j, "l_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "l_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "l_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//prawa noga
			j = createJoint(dummySkeleton->root, "r_hip", norm(osg::Vec3(-16, 0, 0)));
			j = createJoint(j, "r_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "r_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "r_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//w górê
			auto vt = createJoint(dummySkeleton->root, "vt1", norm(osg::Vec3(0, 0, 50)));
			// w lewo
			j = createJoint(vt, "l_shoulder", norm(osg::Vec3(23.5, 0, 0)));
			j = createJoint(j, "l_elbow", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "l_wrist", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "l_middle_distal_tip", norm(osg::Vec3(0, 0, -17)));
			//w prawo
			j = createJoint(vt, "r_shoulder", norm(osg::Vec3(-23.5, 0, 0)));
			j = createJoint(j, "r_elbow", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "r_wrist", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "r_middle_distal_tip", norm(osg::Vec3(0, 0, -17)));

			//g³owa
			j = createJoint(vt, "skullbase", norm(osg::Vec3(0, 0, 15)));
			j = createJoint(j, "skull_tip", norm(osg::Vec3(0, 0, 23)));

			imuDS->registerSkeletonModel(utils::make_shared<IMU::Skeleton>(core::UID::GenerateUniqueID("{D7801231-BACA-42C6-9A8E-2000000A563F}"), *dummySkeleton));
		}

		//objectObserver.reset(new MotionDataObserver);
		//plugin::getDataManagerReader()->addObserver(objectObserver);
	}
	catch (...){
		ret = false;
	}	

	return ret;
}

void PluginHelper::deinit()
{
	finish = true;
	if (streamQuerryingThread.joinable()){
		streamQuerryingThread.join();
		streamQuerryingThread = core::Thread();
	}

	if (objectObserver != nullptr){
		plugin::getDataManagerReader()->removeObserver(objectObserver);
		objectObserver.reset();
	}

	if (skeletonMotion != nullptr){

		if (streamObserver != nullptr){
			skeletonMotion->stream->detachObserver(streamObserver);
			streamObserver.reset();
		}

		skeletonMotion.reset();
	}
}

void PluginHelper::run()
{
	while (finish == false)
	{
		if (streamObserver->modified() == true)
		{
			//s¹ dane - pobieram
			IMU::MotionStream::value_type ms;
			skeletonMotion->stream->data(ms);

			//TODO - robimy coœ z danymi
			//ms.jointsOrientations
			if (ms.second.jointsOrientations.size() > 0)
			{
				osg::Quat superQuat = ms.second.jointsOrientations["HumanoidRoot"];

			}

		}
		else{
			//nie mam danych - czekam
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

CORE_EXT_PLUGIN_BEGIN("imuCostumeAlgorithms", core::UID::GenerateUniqueID("{3C0C0000-9351-46CC-A5FE-52AA182E1279}"), "en", \
	PluginHelper::init, nullptr, PluginHelper::deinit, \
	"vendorName", "vendorShortName", "vendorDescription", "vendorContact", \
	1, 0, 0);

CORE_PLUGIN_END;