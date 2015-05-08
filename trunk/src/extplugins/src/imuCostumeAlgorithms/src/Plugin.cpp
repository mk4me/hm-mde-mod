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
#include "calibWidget.h"

//#define FAKE_FILTER_OUTPUT // if defined, Q(XYZW) = matlab (1000), inst (0100), aqkf (0010), hw (0001)
//#define OUTPUT_MATLAB_TO_OSG // if defined, matlab filter starts to send data to an external visualizer
//#define ACCEPT_EXTERNAL_QUATS // if defined, motion estimator will accept external quaternions
#define OVERRIDE_CALIBRATION // if defined, raw quatenrions will be passed
#define NORTH_FIX // if defined, new north detection (and automatic fix) is enabled

#ifdef OUTPUT_MATLAB_TO_OSG
	#include "CQuatIO.h" // For external visualization only
	CQuatIO quatWriter(true);
#endif // OUTPUT_MATLAB_TO_OSG

#ifdef ACCEPT_EXTERNAL_QUATS
	#include "CQuatIO.h" // For external input only
	CQuatIO quatReader(false);
#endif // ACCEPT_EXTERNAL_QUATS

//! Basic callibration algorithm using gravity vector and magnetometer
class InertialCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-0000000A563F}");

public:
	//! Simple constructor
	InertialCalibrationAlgorithm() {}

	virtual ~InertialCalibrationAlgorithm(){}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeCalibrationAlgorithm * create() const override { return new InertialCalibrationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "Inertial callibration algorithm"; }

	//! Resets algo internal state
	virtual void reset() override { SensorsDescriptions().swap(sa); }

	//! Returns max number (n) of steps that algorithm might require to calibrate costume, 0 means no upper limit
	virtual unsigned int maxCalibrationSteps() const override { return 1; }

	//! Calibration initialization method
	/*!
		\param skeleton	Kalibrowany szkielet
		\param mapping Mapowanie sensorów do szkieletu
		\param sensorsAdjustment Wstêpne ustawienie sensorów - pozwala zadaæ stan pocz¹tkowy bardziej zbli¿ony do rzeczywistoœci
	*/
	virtual void initialize(kinematic::SkeletonConstPtr skeleton,
		const SensorsDescriptions & sensorsDescription) override
	{
		sa = sensorsDescription;
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
			keyValue.second.rotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
			//keyValue.second.rotation = osg::Quat(3.14 / 2, osg::Vec3d(1.0, 0.0, 0.0));
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
		SensorsAdjustemnts ret;

		for (const auto & sd : sa)
		{
			ret.insert(SensorsAdjustemnts::value_type(sd.first, sd.second));
		}

		return ret;
	}
private:

	SensorsDescriptions sa;
	std::set<imuCostume::Costume::SensorID> _sensorCallibrated;
};

//! Naive motion estimation algorithm
class DummyMotionEstimationAlgorithm : public IMU::IMUCostumeMotionEstimationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-1000000A563F}")

public:
	//! Simple initialization
	DummyMotionEstimationAlgorithm() 
	{
		// Initialize local timer
		_lastTick = boost::posix_time::microsec_clock::local_time();
	}
	
	//! Destruktor wirtualny
	virtual ~DummyMotionEstimationAlgorithm() {}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeMotionEstimationAlgorithm * create() const override { return new DummyMotionEstimationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "DummyMotionEstimationAlgorithm"; }

	//! Resets algo internal state
	virtual void reset() override {}

	//! Initializes callibration algorithm
	/*! 
		\param skeleton	Callibrated skeleton
		\param sensorsAdjustment rotational/translational adjustements for every IMU sensor in global frame of reference
		\param sensorsMapping binding structure between IMU sensors' indices and skeleton segments
	*/
	virtual void initialize(kinematic::SkeletonConstPtr skeleton,
		const IMU::IMUCostumeCalibrationAlgorithm::SensorsDescriptions & sensorsDescription) override
	{
		this->skeleton = skeleton;		
		this->sensorsMapping = sensorsDescription;
		this->nodesMapping = kinematic::LinearizedSkeleton::Visitor::createNonLeafMappingKey(*skeleton, [](const kinematic::Skeleton::JointData & jointData) { return jointData.name(); });
	}

	//! Calculates orientation from sensor fusion
	/*!
		\param motionState wstêpnie wyestymowany szkielet
		\param data dane pobrane w aktualnej klatce z IMU
		\param inDeltaT Czas od poprzedniej ramki danych
		\return Returns Lokalne orientacje wszystkich jointów, bez end-sitów
	*/
	virtual kinematic::SkeletonState::RigidCompleteStateLocal estimate(const kinematic::SkeletonState::RigidCompleteStateLocal & motionState,
		const IMU::SensorsData & data, const double inDeltaT) override
	{
		static double accTime = 0.0;

		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		accTime += myTime;

		auto newMotionState = motionState;

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
		// "Vae victis!", Brennus, 390 BC
		// Update cache
		for (auto& keyVal : data)
		{
			_dataCache[keyVal.first] = keyVal.second.orientation;
		}

#ifdef NORTH_FIX 
		int intAccTime = (int)accTime;
		//osg::Quat testRotation = osg::Quat((intAccTime % 90) / 180.0 * 3.14159, osg::Vec3d(0.0, 0.0, 1.0));
		osg::Quat xRot = osg::Quat(osg::PI_4, osg::Vec3d(1.0, 0.0, 0.0));
		osg::Quat zRot = osg::Quat(osg::PI_2, osg::Vec3d(0.0, 0.0, 1.0));

		// xRot * zRot - global ref frame rotated by 90 deg around z, next rotation is around global y (local x)
		osg::Quat testRotation = zRot * xRot;

		newMotionState.orientations[0] = /*testRotation **/ _dataCache[8];//_dataCache[0];
#else
		newMotionState.orientations[0] = _dataCache[8];//_dataCache[0];
#endif

		//koncowy odcinek(lydka) swiruje, nogi sa zamienione a indeksy sie zgadzaja

		////lewa noga (zamieniona z praw¹)
		newMotionState.orientations[nodesMapping.right.at("r_hip")] = _dataCache[6] * _dataCache[8].inverse(); // do roota 
		newMotionState.orientations[nodesMapping.right.at("r_knee")] = _dataCache[10] * _dataCache[6].inverse(); // do biodra
		newMotionState.orientations[nodesMapping.right.at("l_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		newMotionState.orientations[nodesMapping.right.at("l_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0);

		////prawa noga (zamieniona z praw¹)
		newMotionState.orientations[nodesMapping.right.at("l_hip")] = _dataCache[7] * _dataCache[8].inverse(); // do roota
		newMotionState.orientations[nodesMapping.right.at("l_knee")] = _dataCache[9] * _dataCache[7].inverse(); // do biodra
		newMotionState.orientations[nodesMapping.right.at("r_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		newMotionState.orientations[nodesMapping.right.at("r_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0);

		////w górê
		newMotionState.orientations[nodesMapping.right.at("vt1")] = _dataCache[4] * _dataCache[8].inverse(); //osg::Quat(0.0, 0.0, 0.0, 1.0);
		
		//// w lewo (zamieniona z praw¹)
		newMotionState.orientations[nodesMapping.right.at("r_shoulder")] = _dataCache[0] * _dataCache[4].inverse(); // do pleców
		newMotionState.orientations[nodesMapping.right.at("r_elbow")] = _dataCache[3] * _dataCache[0].inverse(); // do ramienia
		newMotionState.orientations[nodesMapping.right.at("l_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		newMotionState.orientations[nodesMapping.right.at("l_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////w prawo (zamieniona z praw¹)
		newMotionState.orientations[nodesMapping.right.at("l_shoulder")] = _dataCache[5] * _dataCache[4].inverse(); // do pleców
		newMotionState.orientations[nodesMapping.right.at("l_elbow")] = _dataCache[11] * _dataCache[5].inverse(); // do ramienia
		newMotionState.orientations[nodesMapping.right.at("r_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		newMotionState.orientations[nodesMapping.right.at("r_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0);

		////g³owa
		newMotionState.orientations[nodesMapping.right.at("skullbase")] = _dataCache[2] * _dataCache[4].inverse(); // do pleców
		newMotionState.orientations[nodesMapping.right.at("skull_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
#endif // ACCEPT_EXTERNAL_QUATS

		return newMotionState;
	}

private:

	kinematic::SkeletonConstPtr skeleton;	
	IMU::IMUCostumeCalibrationAlgorithm::SensorsDescriptions sensorsMapping;
	boost::posix_time::ptime _lastTick;
	std::map<imuCostume::Costume::SensorID, osg::Quat> _dataCache;
	utils::LinearizedTree::Mapping<std::string> nodesMapping;
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
		\param orient raw quaternion from IMU
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
		\param orient raw quaternion from IMU
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
		\param orient raw quaternion from IMU
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
	//! Nasty hack
	osg::Quat _calibQuat;
	bool _callibrated;
	unsigned int _sensorID;

	static CalibWidget* _myQTWindow;
	static unsigned int _currSensorID;
	static bool _inCalibOrientation; // Are we facing West?

	//! Calculates magic fix angle
	double CalcFixAngle(const osg::Vec3d& inAcc, const osg::Vec3d& inMag)
	{
		osg::Vec3d corrMag(inMag.x(), inMag.y(), inMag.z());
		corrMag.normalize();

		osg::Vec3d accVec(inAcc.x(), inAcc.y(), inAcc.z());
		accVec.normalize();

		osg::Vec3d crossMagAcc = corrMag ^ accVec;
		crossMagAcc.normalize();

		//printf("MAGACCCROSS =[X: %.2f, Y: %.2f, Z: %.2f]\n", (float)crossMagAcc.x(), (float)crossMagAcc.y(), (float)crossMagAcc.z());
		double fixAngle = acos(crossMagAcc * osg::Vec3d(0.0, 1.0, 0.0)) * 180.0 / osg::PI;
		//printf("FIX angle = %.2f\n", fixAngle);
		return fixAngle;
	}

	//! Spawns Qt window if non existant
	void SpawnQtWindow()
	{
		// If does not exist
		if (!_myQTWindow)
		{
			_myQTWindow = new CalibWidget();
			_myQTWindow->setVisible(true);
			_myQTWindow->setAttribute(Qt::WA_DeleteOnClose);
			_myQTWindow->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
			_myQTWindow->show();
		}
	}

	//! Despawn Qt window
	void DeSpawnQtWindow()
	{
		// Static field
		if (_myQTWindow)
		{
			//_myQTWindow->close();
			//delete _myQTWindow;
			//_myQTWindow->hide();
			//_myQTWindow->setVisible(false);
			//_myQTWindow = NULL;
		}
	}

public:
	//! Simple constructor
	HardwareKalmanEstimationAlgorithm() : _callibrated(false), _sensorID(_currSensorID++) // Post increment - Set next id
	{
	}

	//! Make it polymorphic
	virtual ~HardwareKalmanEstimationAlgorithm()
	{
		//DeSpawnQtWindow(); // TODO: diff thread error
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
		\param orient raw quaternion from IMU
		\return Returns estimated orientation.
	*/
	virtual osg::Quat estimate(const osg::Vec3d& inAcc, const osg::Vec3d& inGyro,
		const osg::Vec3d& inMag, const double inDeltaT, const osg::Quat & orient) override
	{
		// This is root and we dont have calib orientation
		if ((_sensorID == (8 + 1)) && !_inCalibOrientation)
		{
			// Spawn Qt Window if needed
			SpawnQtWindow();

			// Set callib string
			double fixAngle = CalcFixAngle(inAcc, inMag);
			QString stringToSet = QString::number(fixAngle, 'f', 1);
			_myQTWindow->calibText->setText(stringToSet);

			// Once we get into right position
			if (fixAngle < 3.0)
			{
				_inCalibOrientation = true;
				DeSpawnQtWindow();
				//QApplication::beep();
				//boost::this_thread::sleep(boost::posix_time::seconds(2));
				return osg::Quat(0.0, 0.0, 0.0, 1.0);
			}
		}

		// Not callibrated and in the wrong calibration pose - return
		if (!_inCalibOrientation && !_callibrated)
			return osg::Quat(0.0, 0.0, 0.0, 1.0);

		// TODO: remove me
		// should be R IJK, getting IJK R
		osg::Quat superOrient(orient.y(), orient.z(), orient.w(), orient.x());

		// Not callibrated?
		if (!_callibrated)
		{
			_calibQuat = superOrient.inverse();

#ifdef NORTH_FIX
			//osg::Quat origQ = osg::Quat(q_orig.i, q_orig.j, q_orig.k, q_orig.r);
			//osg::Vec3d magVec = osg::Vec3d(data.mag_x, data.mag_y, data.mag_z);
			//magVec.normalize();
			//osg::Vec3d newMagVec = origQ.inverse() * magVec;
			
			osg::Vec3d normMag = inMag;
			normMag.normalize();
			osg::Vec3d magInGlobalSpace = superOrient.inverse() * normMag;
			double sum = magInGlobalSpace.x() + magInGlobalSpace.y() + magInGlobalSpace.z();
			// In my house: N is (X: 0.38, Y: 0.1, Z: -0,92) - orientation independent in global ref frame, which is logical
#endif

			_callibrated = true;
		}

		// Not needed - straigh through processing
#ifdef FAKE_FILTER_OUTPUT
		osg::Quat fakeQ = osg::Quat(3.14 / 4.0, osg::Vec3d(1.0, 0.0, 0.0)); //fakeQ(0.001, 0.002, 0.003, 0.998);
		return fakeQ;
#else // !FAKE_FILTER_OUTPUT
		osg::Quat gE = superOrient * _calibQuat;
		gE = gE.inverse();
		return gE; // Align my global orientation frame with osg global orientation frame
#endif // FAKE_FILTER_OUTPUT
	}
};

// Temp
CalibWidget* HardwareKalmanEstimationAlgorithm::_myQTWindow = NULL;
unsigned int HardwareKalmanEstimationAlgorithm::_currSensorID = 0;
bool HardwareKalmanEstimationAlgorithm::_inCalibOrientation = true;

// Helper ID for Matlab Dumper Estimation Algorithm
unsigned int MatlabDumpEstimationAlgorithm::_IntID = 0;
volatile bool PluginHelper::finish = false;
core::IDataManagerReader::ObjectObserverPtr PluginHelper::objectObserver = core::IDataManagerReader::ObjectObserverPtr();
core::Thread PluginHelper::streamQuerryingThread = core::Thread();
IMU::CostumeSkeletonMotionConstPtr PluginHelper::skeletonMotion = IMU::CostumeSkeletonMotionConstPtr();
utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver> PluginHelper::streamObserver = utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver>();

//! Observer object for IMU data stream
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

//! Helper function for defining skeleton
kinematic::Skeleton::JointPtr createJoint(kinematic::Skeleton::JointPtr parent,
	const std::string & name, const osg::Vec3 & position,
	const osg::Quat & orientation = osg::Quat(0, 0, 0, 1))
{
	auto ret = kinematic::Skeleton::Joint::create({ name, position, orientation });
	if (parent != nullptr){
		kinematic::Skeleton::Joint::add(parent, ret);
	}

	return ret;
}

//! Helper height normalization function
osg::Vec3 norm(const osg::Vec3 & in)
{
	return in / 183.0;
}

//! Sample plugin initialization
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
			imuDS->registerCostumeCalibrationAlgorithm(IMU::IMUCostumeCalibrationAlgorithmPtr(new InertialCalibrationAlgorithm));

			imuDS->registerMotionEstimationAlgorithm(IMU::IMUCostumeMotionEstimationAlgorithmPtr(new DummyMotionEstimationAlgorithm));
			
			// Register all orientation estimation algorithms
			imuDS->registerOrientationEstimationAlgorithm(IMU::IIMUOrientationEstimationAlgorithmPtr(new MatlabDumpEstimationAlgorithm));
			imuDS->registerOrientationEstimationAlgorithm(IMU::IIMUOrientationEstimationAlgorithmPtr(new InstantenousKalmanEstimationAlgorithm));
			imuDS->registerOrientationEstimationAlgorithm(IMU::IIMUOrientationEstimationAlgorithmPtr(new AQKfKalmanEstimationAlgorithm));
			imuDS->registerOrientationEstimationAlgorithm(IMU::IIMUOrientationEstimationAlgorithmPtr(new HardwareKalmanEstimationAlgorithm));

			//szkielet
			auto dummySkeleton = utils::make_shared<kinematic::Skeleton>(createJoint(kinematic::Skeleton::JointPtr(), "HumanoidRoot", norm(osg::Vec3(0, 0, 0))));
			//dummySkeleton->name = "DummySkeleton";

			//lewa noga
			auto j = createJoint(dummySkeleton->root(), "l_hip", norm(osg::Vec3(16, 0, 0)));
			j = createJoint(j, "l_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "l_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "l_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//prawa noga
			j = createJoint(dummySkeleton->root(), "r_hip", norm(osg::Vec3(-16, 0, 0)));
			j = createJoint(j, "r_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "r_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "r_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//w górê
			auto vt = createJoint(dummySkeleton->root(), "vt1", norm(osg::Vec3(0, 0, 50)));
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

			imuDS->registerSkeletonModel(utils::make_shared<IMU::Skeleton>(core::UID::GenerateUniqueID("{D7801231-BACA-42C6-9A8E-2000000A563F}"), "DummySkeleton", *dummySkeleton));
		}

		//objectObserver.reset(new MotionDataObserver);
		//plugin::getDataManagerReader()->addObserver(objectObserver);
	}
	catch (...){
		ret = false;
	}	

	return ret;
}

//! Sample plugin de-initialization
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

//! Main plugin function (worker thread)
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
			if (ms.second.orientations.empty() == false)
			{
				osg::Quat superQuat = ms.second.orientations[0];

			}

		}
		else{
			//nie mam danych - czekam
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

//! Plugin definition macro
CORE_EXT_PLUGIN_BEGIN("imuCostumeAlgorithms", core::UID::GenerateUniqueID("{3C0C0000-9351-46CC-A5FE-52AA182E1279}"), "en", \
	PluginHelper::init, nullptr, PluginHelper::deinit, \
	"vendorName", "vendorShortName", "vendorDescription", "vendorContact", \
	1, 0, 0);

CORE_PLUGIN_END;