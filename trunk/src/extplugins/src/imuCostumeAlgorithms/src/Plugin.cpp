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

//#define STATIC_TOPOLOGY // if defined, uses old version of the callibration (face (Y) to the (W)est)

#ifdef STATIC_TOPOLOGY
osg::Quat g_zFix = osg::Quat(0.0, 0.0, 0.0, 1.0);
#endif

//! Basic callibration algorithm using gravity vector and magnetometer
class InertialCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-0000000A563F}");

private:
	typedef std::map<imuCostume::Costume::SensorID, osg::Quat> RawSensorOrientations;

public:
	//! Simple constructor
	InertialCalibrationAlgorithm() : _calibStage(CalibWidget::CS_START)
	{}

	virtual ~InertialCalibrationAlgorithm(){}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeCalibrationAlgorithm * create() const override { return new InertialCalibrationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "Inertial callibration algorithm"; }

	//! Resets algo internal state
	virtual void reset() override { SensorsDescriptions().swap(sa); }

	//! Returns max number (n) of steps that algorithm might require to calibrate costume, 0 means no upper limit
	virtual unsigned int maxCalibrationSteps() const override { return 0; }

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
#ifdef STATIC_TOPOLOGY
		// No callibration
		for (auto& keyValue : sa)
		{
			keyValue.second.offset = osg::Vec3d(0.0, 0.0, 0.0);
			keyValue.second.preMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
			keyValue.second.postMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
			//keyValue.second.rotation = osg::Quat(3.14 / 2, osg::Vec3d(1.0, 0.0, 0.0));
		}
		return true;
#else // STATIC_TOPOLOGY
		// TODO: read from skeleton config - Root sensor ID
		imuCostume::Costume::SensorID rootSensorID = 8;
		auto sensorDataIter = data.find(rootSensorID);
		osg::Quat rootBowRotation, zFix;

		// No root data - no point to callibrate
		if (sensorDataIter == data.end())
			return false;

		// Parse proper stage
		switch (_calibStage)
		{
			// Callibration starting phase - just pass packets
		case CalibWidget::CS_START:
			// Waiting for operator to press bind button and for user to assume bind pose
			return false;

			// Read bind pose
		case CalibWidget::CS_BINDPOSE:
			// User is in the bind pose, we can aquire right quaternion now

			// Get initial orientations (used as initial callibration vectors)
			if (_calibBindStage.empty()) // so user wont change it, while going from bind to bow
				_calibBindStage = sdCloneOrientations(data);

			return false;

			// Read bow pose
		case CalibWidget::CS_BOWPOSE:
			// User is in the bow pose, we can aquire right quaternion now

			// Get orientations in bow pose (used to calculate Y vec and zFix)
			if (_calibBowStage.empty())
				_calibBowStage = sdCloneOrientations(data);

			// Calc zFix for globally callibrated sensor
			rootBowRotation = _calibBindStage[rootSensorID] * _calibBowStage[rootSensorID].inverse(); // Contains X rotation now
			zFix = GetZFix(rootBowRotation);

			// Create global calibration table
			// ...

			// Calculate callibration offets
			for (auto& keyValue : sa)
			{
				keyValue.second.offset = osg::Vec3d(0.0, 0.0, 0.0);
				
				keyValue.second.preMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0); 
				//keyValue.second.preMulRotation = osg::Quat(osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0));
				
				keyValue.second.postMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
				//keyValue.second.postMulRotation = osg::Quat(-osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0));  
			}

			// Callibration is finished, form will be killed now
			return true;

			// Unknown stage
		default:
			return false;
		}
		return false;
#endif // !STATIC_TOPOLOGY
	}

	//! \return Widget kalibracyjny (informacje o aktualnym stanie kalibracji, instrukcje dla usera)
	virtual QWidget* calibrationWidget() 
	{ 
		return new CalibWidget(_calibStage);
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
	CalibWidget::ECalibStage _calibStage;

	RawSensorOrientations _calibBindStage;
	RawSensorOrientations _calibBowStage;

	//! Extracts orientations from IMU data vector
	RawSensorOrientations sdCloneOrientations(const IMU::SensorsData& inData, bool invertAll = false) const
	{
		RawSensorOrientations retVec;

		if (invertAll)
		{
			// Extraction with inversion
			for (const auto & item : inData)
			{
				retVec.insert(RawSensorOrientations::value_type(item.first, item.second.orientation.inverse()));
			}
		}
		else
		{
			// Regular extraction
			for (const auto & item : inData)
			{
				retVec.insert(RawSensorOrientations::value_type(item.first, item.second.orientation));
			}
		}

		return retVec;
	}

	RawSensorOrientations sdApplyPostMult(const RawSensorOrientations& inData, const osg::Quat& inQuat) const
	{
		RawSensorOrientations retVec;

		for (const auto & item : inData)
		{
			retVec.insert(RawSensorOrientations::value_type(item.first, item.second * inQuat));
		}

		return retVec;
	}

	RawSensorOrientations sdApplyPreMult(const RawSensorOrientations& inData, const osg::Quat& inQuat) const
	{
		RawSensorOrientations retVec;

		for (const auto & item : inData)
		{
			retVec.insert(RawSensorOrientations::value_type(item.first, inQuat * item.second));
		}

		return retVec;
	}

	// Calculates zFix
	osg::Quat GetZFix(const osg::Quat& bowRotation) const
	{
		// Calc zFix
		// Cast on XY plane
		osg::Vec3d xVec = osg::Vec3d(bowRotation.x(), bowRotation.y(), 0.0);
		xVec.normalize();

		// We bow, so rotation is negative (from Z to Y, will give -X axis)
		xVec *= -1.0;

		// Calc fix value
		osg::Quat zFix;
		zFix.makeRotate(osg::Vec3d(1.0, 0.0, 0.0), xVec);

		// Return value
		return zFix;
	}
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
		for (int i = 0; i < 20; ++i)
			_dataCache[i] = osg::Quat(0.0, 0.0, 0.0, 1.0);
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
		//this->nodesMapping = kinematic::LinearizedSkeleton::Visitor::createNonLeafMappingKey(*skeleton, [](const kinematic::Skeleton::JointData & jointData) { return jointData.name(); });
		const auto order = kinematic::LinearizedSkeleton::Visitor::createNonLeafOrderKey(*skeleton, [](const kinematic::Skeleton::JointData & jointData) { return jointData.name(); });
		this->nodesMapping = utils::LinearizedTree::convert(order);
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
#ifdef STATIC_TOPOLOGY
		// We estimate entire skeleton in-place from raw corrected sensor readings
		auto newMotionState = motionState;

		// Update cache (I assume data comes incomplete, till proven wrong)
		for (auto& keyVal : data)
		{
			_dataCache[keyVal.first] = keyVal.second.orientation;
		}

		// Set root (always 0 index)
		newMotionState.data().orientations[0] = _dataCache[8];
		
		//koncowy odcinek(lydka) swiruje, nogi sa zamienione a indeksy sie zgadzaja

		////lewa noga (zamieniona z praw¹)
		newMotionState.data().orientations[nodesMapping.right.at("r_hip")] = _dataCache[6] * _dataCache[8].inverse(); // do roota 
		newMotionState.data().orientations[nodesMapping.right.at("r_knee")] = _dataCache[10] * _dataCache[6].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("l_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("l_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////prawa noga (zamieniona z praw¹)
		newMotionState.data().orientations[nodesMapping.right.at("l_hip")] = _dataCache[7] * _dataCache[8].inverse(); // do roota
		newMotionState.data().orientations[nodesMapping.right.at("l_knee")] = _dataCache[9] * _dataCache[7].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("r_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("r_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////w górê
		newMotionState.data().orientations[nodesMapping.right.at("vt1")] = _dataCache[4] * _dataCache[8].inverse(); //osg::Quat(0.0, 0.0, 0.0, 1.0);
		//
		////// w lewo (zamieniona z praw¹)
		newMotionState.data().orientations[nodesMapping.right.at("r_shoulder")] = _dataCache[0] * _dataCache[4].inverse(); // do pleców
		newMotionState.data().orientations[nodesMapping.right.at("r_elbow")] = _dataCache[3] * _dataCache[0].inverse(); // do ramienia
		newMotionState.data().orientations[nodesMapping.right.at("l_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("l_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash
		//////w prawo (zamieniona z praw¹)
		newMotionState.data().orientations[nodesMapping.right.at("l_shoulder")] = _dataCache[5] * _dataCache[4].inverse(); // do pleców
		newMotionState.data().orientations[nodesMapping.right.at("l_elbow")] = _dataCache[11] * _dataCache[5].inverse(); // do ramienia
		newMotionState.data().orientations[nodesMapping.right.at("r_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("r_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////g³owa
		newMotionState.data().orientations[nodesMapping.right.at("skullbase")] = _dataCache[2] * _dataCache[4].inverse(); // do pleców
		//newMotionState.orientations[nodesMapping.right.at("skull_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		// Return new motion state (we estimate here)
		return newMotionState;
#else // !STATIC TOPOLOGY
		// Not needed! //
		return motionState;
#endif // !STATIC_TOPOLOGY
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
		// Get real time
		boost::posix_time::ptime nowTick = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration elapsedMicroSec = nowTick - _lastTick;
		_lastTick = nowTick;
		double myTime = elapsedMicroSec.total_milliseconds() / 1000.0;

		// Perform actual estimation
		osg::Quat retQ = _internalFilterImpl->estimate(inAcc, inGyro, inMag, myTime);
		
		// Save Accelerometer (XYZ), Gyroscope (XYZ) and Magnetometer(XYZ)
		_ssMyRawFile << inAcc.x() << "\t" << inAcc.y() << "\t" << inAcc.z() << "\t" <<
					  inGyro.x() << "\t" << inGyro.y() << "\t" << inGyro.z() << "\t" <<
					  inMag.x() << "\t" << inMag.y() << "\t" << inMag.z() << "\t" << std::endl;

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

		return retQ;
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

#ifdef STATIC_TOPOLOGY
	//! Nasty hack
	osg::Quat _calibQuat;
	bool _callibrated;
	unsigned int _sensorID;

	static unsigned int _currSensorID;
#else
	osg::Quat _calibQuat;
	bool _callibrated;
#endif // STATIC_TOPOLOGY

public:
	//! Simple constructor
	HardwareKalmanEstimationAlgorithm() : _callibrated(false)
#ifdef STATIC_TOPOLOGY
		: _callibrated(false), _sensorID(_currSensorID++) // Post increment - Set next id
#endif // STATIC_TOPOLOGY
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
#ifdef STATIC_TOPOLOGY
		// Not callibrated?
		if (!_callibrated)
		{
			// My orientation becomes a reference (we have orientation corresponding 
			// to accelerometer and magnetometer in global coordinate frame)
			_calibQuat = orient;
			_callibrated = true;
		}

		// Requires actor to face (W)est - (N)orth will be on the right side (rotation around global X axis)
		osg::Quat retVec =_calibQuat * orient.inverse(); 

		// TODO: remove me?
		return g_zFix * retVec * g_zFix.inverse();

#else // !STATIC_TOPOLOGY
		// Pass through mode
		if (!_callibrated)
		{
			// My orientation becomes a reference (we have orientation corresponding 
			// to accelerometer and magnetometer in global coordinate frame)
			_calibQuat = orient;
			_callibrated = true;
		}

		// Requires actor to face (W)est - (N)orth will be on the right side (rotation around global X axis)
		osg::Quat retVec = _calibQuat * orient.inverse();
		return retVec;
		//return osg::Quat(osg::PI_4, osg::Vec3d(0.0, 1.0, 0.0));
		//return osg::Quat(0.0, 0.0, 0.0, 1.0);
#endif
	}
};

// Helper ID for Matlab Dumper Estimation Algorithm
#ifdef STATIC_TOPOLOGY
unsigned int HardwareKalmanEstimationAlgorithm::_currSensorID = 0;
#endif
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

#ifdef STATIC_TOPOLOGY
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
#else // !STATIC_TOPOLOGY
			//lewa noga
			auto j = createJoint(dummySkeleton->root(), "l_hip", norm(osg::Vec3(-16, 0, 0)));
			j = createJoint(j, "l_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "l_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "l_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//prawa noga
			j = createJoint(dummySkeleton->root(), "r_hip", norm(osg::Vec3(16, 0, 0)));
			j = createJoint(j, "r_knee", norm(osg::Vec3(0, 0, -50)));
			j = createJoint(j, "r_ankle", norm(osg::Vec3(0, 0, -45)));
			j = createJoint(j, "r_forefoot_tip", norm(osg::Vec3(0, 26.5, 0)));

			//w górê
			auto vt = createJoint(dummySkeleton->root(), "vt1", norm(osg::Vec3(0, 0, 50)));
			// w lewo
			j = createJoint(vt, "l_shoulder", norm(osg::Vec3(-23.5, 0, 0)));
			j = createJoint(j, "l_elbow", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "l_wrist", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "l_middle_distal_tip", norm(osg::Vec3(0, 0, -17)));
			//w prawo
			j = createJoint(vt, "r_shoulder", norm(osg::Vec3(23.5, 0, 0)));
			j = createJoint(j, "r_elbow", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "r_wrist", norm(osg::Vec3(0, 0, -30)));
			j = createJoint(j, "r_middle_distal_tip", norm(osg::Vec3(0, 0, -17)));

			//g³owa
			j = createJoint(vt, "skullbase", norm(osg::Vec3(0, 0, 15)));
			j = createJoint(j, "skull_tip", norm(osg::Vec3(0, 0, 23)));
#endif // !STATIC_TOPOLOGY

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
			if (ms.second.data().orientations.empty() == false)
			{
				osg::Quat superQuat = ms.second.data().orientations[0];

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