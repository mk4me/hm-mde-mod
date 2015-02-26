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

#define FAKE_FILTER_OUTPUT // if defined, Q(XYZW) = matlab (1000), inst (0100), aqkf (0010), hw (0001)
//#define OUTPUT_MATLAB_TO_OSG // if defined, matlab filter starts to send data to an external visualizer
//#define ACCEPT_EXTERNAL_QUATS // if defined, motion estimator will accept external quaternions
#define OVERRIDE_CALIBRATION // if defined, raw quatenrions will be passed

#ifdef OUTPUT_MATLAB_TO_OSG
	#include "CQuatIO.h" // For external visualization only
	CQuatIO quatWriter(true);
#endif // OUTPUT_MATLAB_TO_OSG

#ifdef ACCEPT_EXTERNAL_QUATS
	#include "CQuatIO.h" // For external input only
	CQuatIO quatReader(false);
#endif // ACCEPT_EXTERNAL_QUATS


class InertialCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-0000000A563F}");

public:
	InertialCalibrationAlgorithm() {}
	virtual ~InertialCalibrationAlgorithm(){}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeCalibrationAlgorithm * create() const override { return new InertialCalibrationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "Inertial callibration algorithm"; }

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
#ifdef OVERRIDE_CALIBRATION
		// No callibration
		for (auto& keyValue : sa)
		{
			keyValue.second.offset = osg::Vec3d(0.0, 0.0, 0.0);
			//keyValue.second.rotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
			keyValue.second.rotation = osg::Quat(3.14 / 2, osg::Vec3d(1.0, 0.0, 0.0));
		}
		return true;
#else
		// Make copy of received data
		IMU::SensorsData myData = data;

		// Align with global coordinate system
		for (auto& keyValue : myData)
		{
			sa[keyValue.first].offset = osg::Vec3d(0.0, 0.0, 0.0);
			sa[keyValue.first].rotation = keyValue.second.orientation;//.inverse();
			_sensorCallibrated.insert(keyValue.first);
		}
		tu jest rozwalone - nei wiem czy faktycznie mam podawac lokalne poprawki do kalibracji? i czy mi nie neguj¹ kwaternionu kalibracyjnego
		// Aligned all sensors
		if (_sensorCallibrated.size() == sa.size())
			return true;
		else
			return false;
#endif // OVERRIDE_CALIBRATION
	}

	//! \return Dane kalibracyjne szkieletu, poprawki dla sensorów
	virtual SensorsAdjustemnts sensorsAdjustemnts() const override
	{
		return sa;
	}

private:

	SensorsAdjustemnts sa;
	std::set<imuCostume::Costume::SensorID> _sensorCallibrated;
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
		static double accTime = 0.0;
		
		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		accTime += myTime;

		MotionState newMotionState = motionState;

#ifdef ACCEPT_EXTERNAL_QUATS
		newMotionState.jointsOrientations["HumanoidRoot"] = quatReader.GetQuat(0);

		//lewa noga
		newMotionState.jointsOrientations["l_hip"] = quatReader.GetQuat(1);
		newMotionState.jointsOrientations["l_knee"] = quatReader.GetQuat(2);
		newMotionState.jointsOrientations["l_ankle"] = quatReader.GetQuat(3);
		newMotionState.jointsOrientations["l_forefoot_tip"] = quatReader.GetQuat(4);

		//prawa noga
		newMotionState.jointsOrientations["r_hip"] = quatReader.GetQuat(5);
		newMotionState.jointsOrientations["r_knee"] = quatReader.GetQuat(6);
		newMotionState.jointsOrientations["r_ankle"] = quatReader.GetQuat(7);
		newMotionState.jointsOrientations["r_forefoot_tip"] = quatReader.GetQuat(8);

		//w górê
		newMotionState.jointsOrientations["vt1"] = quatReader.GetQuat(9);
		// w lewo
		newMotionState.jointsOrientations["l_shoulder"] = quatReader.GetQuat(10);
		newMotionState.jointsOrientations["l_elbow"] = quatReader.GetQuat(11);
		newMotionState.jointsOrientations["l_wrist"] = quatReader.GetQuat(12);
		newMotionState.jointsOrientations["l_middle_distal_tip"] = quatReader.GetQuat(13);
		//w prawo
		newMotionState.jointsOrientations["r_shoulder"] = quatReader.GetQuat(14);
		newMotionState.jointsOrientations["r_elbow"] = quatReader.GetQuat(15);
		newMotionState.jointsOrientations["r_wrist"] = quatReader.GetQuat(16);
		newMotionState.jointsOrientations["r_middle_distal_tip"] = quatReader.GetQuat(17);

		//g³owa
		newMotionState.jointsOrientations["skullbase"] = quatReader.GetQuat(18);
		newMotionState.jointsOrientations["skull_tip"] = quatReader.GetQuat(19);
#else // !ACCEPT_EXTERNAL_QUATS
		//for (auto& keyVal : newMotionState.jointsOrientations)
		//{
		//	keyVal.second = osg::Quat(0.0, 0.0, 0.0, 1.0);
		//}
#endif // ACCEPT_EXTERNAL_QUATS

		return newMotionState;
	}

private:
	kinematic::SkeletonConstPtr skeleton;
	IMU::IMUCostumeCalibrationAlgorithm::SensorsAdjustemnts sensorsAdjustment;
	IMU::SensorsMapping sensorsMapping;
	boost::posix_time::ptime _lastTick;
};

//! Matlab dumper (fake filter) - generates orientation as a quaternion using IMU sensor fusion
class MatlabDumpEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-2000000A563F}")

private:
	std::ofstream _myLogFile;
	std::ofstream _myRawFile;
	std::stringstream _ssMyLogFile;
	std::stringstream _ssMyRawFile;
	boost::posix_time::ptime _lastTick;
	//boost::mutex _estimateMutex;
	static unsigned int _IntID;
	unsigned int _thisID;
	unsigned int _sampleNum;
	std::unique_ptr<ImuFilters::IOrientationFilter> _internalFilterImpl;

public:
	//! Simple constructor
	MatlabDumpEstimationAlgorithm() : _internalFilterImpl(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_INSTANTENOUS_KALMAN))
	{
		// Open files and initialize timer
		std::string logName = "DOEA_log_" + std::to_string(_IntID) + ".txt";
		std::string rawDataName = "RAW_log_" + std::to_string(_IntID) + ".txt";
		_myLogFile.open(logName, std::ios::trunc);
		_myRawFile.open(rawDataName, std::ios::trunc);
		_lastTick = boost::posix_time::microsec_clock::local_time();

		// Set object counter (unique id is not provided) and reset sample number
		_sampleNum = 0;
		_thisID = ++_IntID;
	}

	//! Make it polymorphic
	virtual ~MatlabDumpEstimationAlgorithm()
	{
		// Force close any opened files
		_myLogFile.close();
		_myRawFile.close();
	}

	//! Returns the actual implementation
	virtual IIMUOrientationEstimationAlgorithm * create() const override 
	{
		// Return implementation
		return new MatlabDumpEstimationAlgorithm;
	};

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override 
	{ 
		// Return the actual role
		return "Matlab Dumper (Instantenous Estimation Algorithm)";
	}

	//! Resets filter internal state
	virtual void reset() override
	{
		// Reset internal state
		_internalFilterImpl->reset();
	}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override 
	{ 
		// Returns required delay
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

		// Get real time
		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		// Perform actual estimation
		osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, myTime);
		
#ifdef OUTPUT_MATLAB_TO_OSG
		// External visualizer
		quatWriter.SetQuat(orient, _thisID);
		//quatWriter.SetQuat(retQ, _thisID);
#endif

		// Save Accelerometer (XYZ), Gyroscope (XYZ) and Magnetometer(XYZ)
		_ssMyRawFile << inAcc.x() << "\t" << inAcc.y() << "\t" << inAcc.z() << "\t" <<
					  inGyro.x() << "\t" << inGyro.y() << "\t" << inGyro.z() << "\t" <<
					  inMag.x() << "\t" << inMag.y() << "\t" << inMag.z() << "\t" << std::endl;

		//_myLogFile << "[ThreadID:]" << boost::this_thread::get_id() << "\t" << inDeltaT << "\t" << elapsedMicroSec.total_milliseconds() << std::endl;
		// Save sample number, both time bases and estimated quaternon
		_ssMyLogFile << _sampleNum << "\t" << inDeltaT << "\t" << myTime << "\t" << 
			retQ.x() << "\t" << retQ.y() << "\t" << retQ.z() << "\t" <<  retQ.w() << std::endl;

		// Write log (every 200 sample, so we won't generate I/O operations too often)
		if ((_sampleNum % 200) == 0)
		{
			_myRawFile << _ssMyRawFile.str();
			_myLogFile << _ssMyLogFile.str();
			_ssMyRawFile.str("");
			_ssMyLogFile.str("");
			_myRawFile.flush();
			_myLogFile.flush();
		}
		
		// Increment sample number
		++_sampleNum;

		// Return estimated quaternion
#ifdef FAKE_FILTER_OUTPUT
		return osg::Quat(1.0, 0.0, 0.0, 0.0);
#else // !FAKE_FILTER_OUTPUT
		return retQ;
#endif // FAKE_FILTER_OUTPUT
	}
};

//! Instantenous Kalman filter - generates orientation as a quaternion using IMU sensor fusion
class InstantenousKalmanEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-20AC8B76ACBF}")

private:
	//! Time basis
	boost::posix_time::ptime _lastTick;
	//! Internal filter implementation
	std::unique_ptr<ImuFilters::IOrientationFilter> _internalFilterImpl;

public:
	//! Simple constructor
	InstantenousKalmanEstimationAlgorithm() : _internalFilterImpl(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_INSTANTENOUS_KALMAN))
	{
		// Initialize timer
		_lastTick = boost::posix_time::microsec_clock::local_time();
	}

	//! Make it polymorphic
	virtual ~InstantenousKalmanEstimationAlgorithm()
	{
	}

	//! Returns the actual implementation
	virtual IIMUOrientationEstimationAlgorithm * create() const override
	{
		return new InstantenousKalmanEstimationAlgorithm;
	};

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override
	{
		return "Software Orientation Estimation (Instatenous Kalman Filter)"; 
	}

	//! Resets filter internal state
	virtual void reset() override
	{
		// Reset internal state
		_internalFilterImpl->reset();
	}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override
	{
		// Return number of samples required for this filter to run
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
		// Get real time
		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		// Actual estimation
		osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, myTime);
		
		// Return estimated value
#ifdef FAKE_FILTER_OUTPUT
		return osg::Quat(0.0, 1.0, 0.0, 0.0);
#else // !FAKE_FILTER_OUTPUT
		return retQ;
#endif
	}
};

//! Generic quaternion-based orientation filter - generates orientation as a quaternion using IMU sensor fusion
class AQKfKalmanEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-4F5E6346FEBF}")

private:
	//! Time basis
	boost::posix_time::ptime _lastTick;
	//! Internal filter implementation
	std::unique_ptr<ImuFilters::IOrientationFilter> _internalFilterImpl;

public:
	//! Simple constructor
	AQKfKalmanEstimationAlgorithm() : _internalFilterImpl(ImuFilters::createFilter(ImuFilters::IOrientationFilter::FT_AQKF_KALMAN))
	{
		// Initialize timer
		_lastTick = boost::posix_time::microsec_clock::local_time();
	}

	//! Make it polymorphic
	virtual ~AQKfKalmanEstimationAlgorithm()
	{
	}

	//! Returns the actual implementation
	virtual IIMUOrientationEstimationAlgorithm * create() const override
	{
		return new AQKfKalmanEstimationAlgorithm;
	};

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override
	{
		return "Software Orientation Estimation (AQKf Kalman Filter)";
	}

	//! Resets filter internal state
	virtual void reset() override
	{
		// Reset internal state
		_internalFilterImpl->reset();
	}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override
	{
		// Return number of samples required for this filter to run
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
		// Get real time
		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		// Actual estimation
		osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, myTime);

		// Return estimated value
#ifdef FAKE_FILTER_OUTPUT
		return osg::Quat(0.0, 0.0, 1.0, 0.0);
#else // !FAKE_FILTER_OUTPUT
		return retQ;
#endif // FAKE_FILTER_OUTPUT
	}
};

//! Generic quaternion-based orientation filter - generates orientation as a quaternion using IMU sensor fusion
class HardwareKalmanEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-12B41D2BD142}")

private:
	

public:
	//! Simple constructor
	HardwareKalmanEstimationAlgorithm() 
	{
	}

	//! Make it polymorphic
	virtual ~HardwareKalmanEstimationAlgorithm()
	{
	}

	//! Returns the actual implementation
	virtual IIMUOrientationEstimationAlgorithm * create() const override
	{
		return new HardwareKalmanEstimationAlgorithm;
	};

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override
	{
		return "Hardware Orientation Estimation (Simple Kalman Filter)";
	}

	//! Resets filter internal state
	virtual void reset() override
	{
		// Not needed
	}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override
	{
		// Not needed
		return 0;
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
		// Not needed - straigh through processing
#ifdef FAKE_FILTER_OUTPUT
		return osg::Quat(0.0, 0.0, 0.0, 1.0);
#else // !FAKE_FILTER_OUTPUT
		return orient;
#endif // FAKE_FILTER_OUTPUT
	}
};

// Helper ID for Matlab Dumper Estimation Algorithm
unsigned int MatlabDumpEstimationAlgorithm::_IntID = 0;
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
			// Register basic callibration algorithm
			imuDS->registerCostumeCalibrationAlgorithm(new InertialCalibrationAlgorithm);

			imuDS->registerMotionEstimationAlgorithm(new DummyMotionEstimationAlgorithm);
			
			// Register all orientation estimation algorithms
			imuDS->registerOrientationEstimationAlgorithm(new MatlabDumpEstimationAlgorithm);
			imuDS->registerOrientationEstimationAlgorithm(new InstantenousKalmanEstimationAlgorithm);
			imuDS->registerOrientationEstimationAlgorithm(new AQKfKalmanEstimationAlgorithm);
			imuDS->registerOrientationEstimationAlgorithm(new HardwareKalmanEstimationAlgorithm);

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