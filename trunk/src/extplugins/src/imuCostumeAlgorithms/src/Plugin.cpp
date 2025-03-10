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
IMU::IMUCostumeCalibrationAlgorithm::SensorsDescriptions g_sensorsAdjustements;

//! Basic callibration algorithm using gravity vector and magnetometer
class InertialCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
	UNIQUE_ID("{D7801231-BACA-42C6-9A8E-0000000A563F}");

private:
	//! My special types
	typedef std::map<imuCostume::Costume::SensorID, osg::Quat> RawSensorOrientations;

	//! Class checking if my root is accessing from every selected joint
	class CVisMyTree_Impl : public kinematic::LinearizedSkeleton::Visitor
	{
	private:
		//! Root pointer to be found
		kinematic::Skeleton::JointConstPtr _soCalledRootPtr;

		//! Is root always accessible
		bool& _isRootAccessible;

		//! Lookup list for allowed joints
		std::set<std::string> _jointInUse;

	public:
		//! Simple constructor (sets root pointer to find)
		CVisMyTree_Impl(kinematic::Skeleton::JointConstPtr soCalledRoot, bool& isRootAccessibleRef,
			const SensorsDescriptions& descList) : _soCalledRootPtr(soCalledRoot), _isRootAccessible(isRootAccessibleRef)
		{
			// Create allowed joints filter
			for (const auto& sensDesc : descList)
			{
				_jointInUse.insert(sensDesc.second.jointName);
			}
		}

		//! Function performing visiting
		void operator() (kinematic::Skeleton::JointConstPtr jointPtr)
		{
			// Cycle up helper variable
			kinematic::Skeleton::JointConstPtr currJointPtr = jointPtr;

			// Go up, to the root
			while (currJointPtr)
			{
				// Is that checked joint on the adjustement list?
				if (_jointInUse.find(jointPtr->value().name()) == _jointInUse.end())
					return; // Shold not be checked

				// Did we find root bone?
				if (currJointPtr == _soCalledRootPtr)
					return; // Flag should be untouched - default: true

				// Go up!
				currJointPtr = currJointPtr->parent();
			}

			// Something went wrong
			_isRootAccessible = false;
		}
	};

public:
	//! Global root bone name
	std::string ROOT_BONE_NAME;

	//! Simple constructor
	InertialCalibrationAlgorithm() : _calibStage(CalibWidget::CS_START), _calibWindow(nullptr)
	{}

	virtual ~InertialCalibrationAlgorithm(){}

	//! \return Nowy algorytm kalibracji
	virtual IMUCostumeCalibrationAlgorithm * create() const override { return new InertialCalibrationAlgorithm; }

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "Inertial callibration algorithm"; }

	//! Resets algo internal state
	virtual void reset() override { SensorsDescriptions().swap(_sensorAdjustements); }

	//! Returns max number (n) of steps that algorithm might require to calibrate costume, 0 means no upper limit
	virtual unsigned int maxCalibrationSteps() const override { return 0; }

	//! Calibration initialization method
	/*!
		\param skeleton	Kalibrowany szkielet
		\param mapping Mapowanie sensor�w do szkieletu
		\param sensorsAdjustment Wst�pne ustawienie sensor�w - pozwala zada� stan pocz�tkowy bardziej zbli�ony do rzeczywisto�ci
	*/
	virtual void initialize(kinematic::SkeletonConstPtr skeleton,
		const SensorsDescriptions & sensorsDescription) override
	{
		// Initialize mapping (required to find a root sensor)
		_skeleton = skeleton;
		const auto order = kinematic::LinearizedSkeleton::Visitor::createNonLeafOrderKey(*skeleton, [](const kinematic::Skeleton::JointData & jointData) {	return jointData.name(); });
		_nodesMapping = treeContainer::Linearization::convert(order);

		// Find Root Name
		ROOT_BONE_NAME = FindRootBoneName(sensorsDescription);
		if (ROOT_BONE_NAME.empty())
			throw std::exception("Can't find root bone");

		// Find Root Ptr
		auto skeletonPtrMap = _skeleton->joints(_skeleton->root());
		auto rootJointPtrIter = skeletonPtrMap.find(ROOT_BONE_NAME);
		if (rootJointPtrIter == skeletonPtrMap.end())
			throw std::exception("Can't find root bone ptr");

		// Initialize descriptions
		_sensorAdjustements = sensorsDescription;

		// Check if all the selected sensors connect to my root
		bool isRootAccessible = true;
		CVisMyTree_Impl visInstance(rootJointPtrIter->second, isRootAccessible, _sensorAdjustements);
		kinematic::LinearizedSkeleton::Visitor::visit(*_skeleton, visInstance);
		if (!isRootAccessible)
			throw std::exception("Topology inconsistent");
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
		// Read root sensor ID from skeleton config
		imuCostume::Costume::SensorID rootSensorID = FindRootSensorID(_sensorAdjustements);
		osg::Quat rootBowRotation, zFix;

		// No root data - no point to callibrate
		if (rootSensorID == std::numeric_limits<imuCostume::Costume::SensorID>::max())
			return false;

		// Always update cache - RAW orientations
		sdCloneAndUpdate(_stageCache, data, _sensorAdjustements, true);

		// Parse proper stage
		switch (_calibStage)
		{
		// Callibration starting phase - just pass packets
		case CalibWidget::CS_START:
			// Waiting for operator to press bind button and for user to assume bind pose
			_stageCache.clear();
			return false;

		// Read bind pose
		case CalibWidget::CS_BINDPOSE:
			// User is in the bind pose, we can aquire right quaternions now

			// Calibration quaternions are set - just return, so user wont change it, while going from bind to bow
			if (!_calibBindStage.empty())
				return false;
				
			// We have all readings
			if (_stageCache.size() == _sensorAdjustements.size())
			{
				// Stage snapshot
				_calibBindStage = _stageCache; // I NEED EXACT READINGS AND I INVERT 
												// READING IN ESTIMATE_SENSOR (to get raw quaternions from hardware)

				// Enagle bow button
				if (_calibWindow)
					_calibWindow->bowButton->setEnabled(true);

				// Clear cache
				_stageCache.clear();
			}

			return false;

		// Read bow pose
		case CalibWidget::CS_BOWPOSE:
			// User is in the bow pose, we can aquire right quaternion now

			// Wait till we have all sensors
			if (_stageCache.size() != _sensorAdjustements.size())
				return false;

			// Get orientations in bow pose (used to calculate Y vec and zFix)
			_calibBowStage = _stageCache;  // I NEED EXACT READINGS AND I INVERT 
											// READING IN ESTIMATE_SENSOR (to get raw quaternions from hardware)

			// Clear cache
			_stageCache.clear();

			// Calc zFix for globally callibrated sensor
			rootBowRotation = _calibBindStage[rootSensorID] * _calibBowStage[rootSensorID].inverse(); // Contains X rotation now
			zFix = GetZFix(rootBowRotation);

			// Clear sensor adjustements
			for (auto& saKeyVal : _sensorAdjustements)
			{
				saKeyVal.second.adjustment.offset = osg::Vec3d(0.0, 0.0, 0.0);
				saKeyVal.second.adjustment.preMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
				saKeyVal.second.adjustment.postMulRotation = osg::Quat(0.0, 0.0, 0.0, 1.0);
			}

			// Fill callibration table
			UTILS_ASSERT(_calibBindStage.size() == _sensorAdjustements.size());
			for (const auto& calBindItem : _calibBindStage)
			{
				// Get element reference
				auto & saRef = _sensorAdjustements[calBindItem.first];
				saRef.adjustment.preMulRotation = zFix * calBindItem.second; // Align face and calib sensor to global coordinate frame (with respect to accel (-Z) and global north (X))
				saRef.adjustment.postMulRotation = zFix.inverse();
			}

			// Callibration is finished, form will be killed now
			g_sensorsAdjustements = _sensorAdjustements;
			_calibWindow = nullptr;
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
		_calibWindow = new CalibWidget(_calibStage);
		return _calibWindow;
	}

	//! \return Dane kalibracyjne szkieletu, poprawki dla sensor�w
	virtual SensorsAdjustemnts sensorsAdjustemnts() const override
	{
		SensorsAdjustemnts ret;

		for (const auto & sd : _sensorAdjustements)
		{
			ret.insert({ sd.first, sd.second.adjustment });
		}

		return ret;
	}
private:

	SensorsDescriptions _sensorAdjustements;
	CalibWidget::ECalibStage _calibStage;

	RawSensorOrientations _stageCache;

	RawSensorOrientations _calibBindStage;
	RawSensorOrientations _calibBowStage;

	CalibWidget* _calibWindow;

	kinematic::SkeletonConstPtr _skeleton;
	treeContainer::Linearization::Mapping<std::string> _nodesMapping;

	//! Extracts orientations from IMU data vector
	RawSensorOrientations sdCloneOrientations(const IMU::SensorsData& inData, const SensorsDescriptions& inFilter,  bool invertAll = false) const
	{
		RawSensorOrientations retVec;

		if (invertAll)
		{
			// Extraction with inversion
			for (const auto& item : inData)
			{
				// Is it on the list?
				if (inFilter.find(item.first) != inFilter.end())
					retVec.insert(RawSensorOrientations::value_type(item.first, item.second.orientation.inverse()));
			}
		}
		else
		{
			// Regular extraction
			for (const auto& item : inData)
			{
				// Is it on the list?
				if (inFilter.find(item.first) != inFilter.end())
					retVec.insert(RawSensorOrientations::value_type(item.first, item.second.orientation));
			}
		}

		return retVec;
	}

	//! Extracts orientations from IMU data vector
	void sdCloneAndUpdate(RawSensorOrientations& outData, const IMU::SensorsData& inData, const SensorsDescriptions& inFilter, bool invertAll = false) const
	{
		if (invertAll)
		{
			// Extraction with inversion
			for (const auto& item : inData)
			{
				// Is it on the list?
				if (inFilter.find(item.first) != inFilter.end())
					outData[item.first] = item.second.orientation.inverse();
			}
		}
		else
		{
			// Regular extraction
			for (const auto& item : inData)
			{
				// Is it on the list?
				if (inFilter.find(item.first) != inFilter.end())
					outData[item.first] = item.second.orientation;
			}
		}
	}

	//! Applies multiplication vector times vector (post)
	RawSensorOrientations sdApplyPostMult(const RawSensorOrientations& inData, const osg::Quat& inQuat) const
	{
		RawSensorOrientations retVec;

		for (const auto& item : inData)
		{
			retVec.insert(RawSensorOrientations::value_type(item.first, item.second * inQuat));
		}

		return retVec;
	}

	//! Applies multiplication vector times vector (pre)
	RawSensorOrientations sdApplyPreMult(const RawSensorOrientations& inData, const osg::Quat& inQuat) const
	{
		RawSensorOrientations retVec;

		for (const auto& item : inData)
		{
			retVec.insert(RawSensorOrientations::value_type(item.first, inQuat * item.second));
		}

		return retVec;
	}

	//! Finds root sensor ID in sensor adjustements
	imuCostume::Costume::SensorID FindRootSensorID(const SensorsDescriptions& inSensorDesc) const
	{
		for (const auto& item : inSensorDesc)
		{
			if (item.second.jointName == ROOT_BONE_NAME)
				return item.first;
		}

		// Errorneous ID
		return std::numeric_limits<imuCostume::Costume::SensorID>::max();
	}

	//! Finds root sensor name using lowest ID
	std::string FindRootBoneName(const SensorsDescriptions& inSensorDesc) const
	{
		imuCostume::Costume::SensorID bestID = std::numeric_limits<imuCostume::Costume::SensorID>::max();
		kinematic::LinearizedSkeleton::NodeIDX jointID = std::numeric_limits<kinematic::LinearizedSkeleton::NodeIDX>::max();
		for (const auto& item : inSensorDesc)
		{
			// Better id - memorize it
			if (item.second.jointIdx < jointID)
			{
				bestID = item.first;
				jointID = item.second.jointIdx;
			}
		}

		// Not found
		if (bestID == std::numeric_limits<imuCostume::Costume::SensorID>::max())
			return "";
		else
			return inSensorDesc.at(bestID).jointName;
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
		this->nodesMapping = treeContainer::Linearization::convert(order);
	}

	//! Calculates orientation from sensor fusion
	/*!
		\param motionState wst�pnie wyestymowany szkielet
		\param data dane pobrane w aktualnej klatce z IMU
		\param inDeltaT Czas od poprzedniej ramki danych
		\return Returns Lokalne orientacje wszystkich joint�w, bez end-sit�w
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

		////lewa noga (zamieniona z praw�)
		newMotionState.data().orientations[nodesMapping.right.at("r_hip")] = _dataCache[6] * _dataCache[8].inverse(); // do roota 
		newMotionState.data().orientations[nodesMapping.right.at("r_knee")] = _dataCache[10] * _dataCache[6].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("l_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("l_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////prawa noga (zamieniona z praw�)
		newMotionState.data().orientations[nodesMapping.right.at("l_hip")] = _dataCache[7] * _dataCache[8].inverse(); // do roota
		newMotionState.data().orientations[nodesMapping.right.at("l_knee")] = _dataCache[9] * _dataCache[7].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("r_ankle")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("r_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////w g�r�
		newMotionState.data().orientations[nodesMapping.right.at("vt1")] = _dataCache[4] * _dataCache[8].inverse(); //osg::Quat(0.0, 0.0, 0.0, 1.0);
		//
		////// w lewo (zamieniona z praw�)
		newMotionState.data().orientations[nodesMapping.right.at("r_shoulder")] = _dataCache[0] * _dataCache[4].inverse(); // do plec�w
		newMotionState.data().orientations[nodesMapping.right.at("r_elbow")] = _dataCache[3] * _dataCache[0].inverse(); // do ramienia
		newMotionState.data().orientations[nodesMapping.right.at("l_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("l_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash
		//////w prawo (zamieniona z praw�)
		newMotionState.data().orientations[nodesMapping.right.at("l_shoulder")] = _dataCache[5] * _dataCache[4].inverse(); // do plec�w
		newMotionState.data().orientations[nodesMapping.right.at("l_elbow")] = _dataCache[11] * _dataCache[5].inverse(); // do ramienia
		newMotionState.data().orientations[nodesMapping.right.at("r_wrist")] = osg::Quat(0.0, 0.0, 0.0, 1.0);
		////newMotionState.orientations[nodesMapping.right.at("r_middle_distal_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////g�owa
		newMotionState.data().orientations[nodesMapping.right.at("skullbase")] = _dataCache[2] * _dataCache[4].inverse(); // do plec�w
		//newMotionState.orientations[nodesMapping.right.at("skull_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		// Return new motion state (we estimate here)
		return newMotionState;
#else // !STATIC TOPOLOGY
		// Not needed! - passthrough mode
		return motionState;


		// We estimate entire skeleton in-place from raw corrected sensor readings
		auto newMotionState = motionState;

		// Update cache (I assume data comes incomplete, till proven wrong)
		for (auto& keyVal : data)
		{
			_dataCache[keyVal.first] = g_sensorsAdjustements[keyVal.first].adjustment.preMulRotation * keyVal.second.orientation * g_sensorsAdjustements[keyVal.first].adjustment.postMulRotation;
		}

		// Set root (always 0 index)
		newMotionState.data().orientations[0] = _dataCache[8];

		////lewa noga
		newMotionState.data().orientations[nodesMapping.right.at("l_hip")] = _dataCache[6] * _dataCache[8].inverse(); // do roota 
		newMotionState.data().orientations[nodesMapping.right.at("l_knee")] = _dataCache[10] * _dataCache[6].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("l_ankle")] = _dataCache[16] * _dataCache[10].inverse(); // do �ydki
		////newMotionState.orientations[nodesMapping.right.at("l_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		//////prawa noga 
		newMotionState.data().orientations[nodesMapping.right.at("r_hip")] = _dataCache[7] * _dataCache[8].inverse(); // do roota
		newMotionState.data().orientations[nodesMapping.right.at("r_knee")] = _dataCache[9] * _dataCache[7].inverse(); // do biodra
		newMotionState.data().orientations[nodesMapping.right.at("r_ankle")] = _dataCache[15] * _dataCache[9].inverse(); // do �ydki
		////newMotionState.orientations[nodesMapping.right.at("r_forefoot_tip")] = osg::Quat(0.0, 0.0, 0.0, 1.0); // end effector crash

		return newMotionState;
#endif // !STATIC_TOPOLOGY
	}

private:

	kinematic::SkeletonConstPtr skeleton;
	IMU::IMUCostumeCalibrationAlgorithm::SensorsDescriptions sensorsMapping;
	boost::posix_time::ptime _lastTick;
	std::map<imuCostume::Costume::SensorID, osg::Quat> _dataCache;
	treeContainer::Linearization::Mapping<std::string> nodesMapping;
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
		// I invert readings here, because I don't have access to skeleton estimation
		// I patch it in callibration protocol
		return orient.inverse();
#endif
	}
};

// Helper ID for Matlab Dumper Estimation Algorithm
#ifdef STATIC_TOPOLOGY
unsigned int HardwareKalmanEstimationAlgorithm::_currSensorID = 0;
#endif
unsigned int MatlabDumpEstimationAlgorithm::_IntID = 0;
volatile bool PluginHelper::finish = false;
core::IDataManagerReader::ObserverPtr PluginHelper::objectObserver = core::IDataManagerReader::ObserverPtr();
core::Thread PluginHelper::streamQuerryingThread = core::Thread();
IMU::CostumeSkeletonMotionConstPtr PluginHelper::skeletonMotion = IMU::CostumeSkeletonMotionConstPtr();
utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver> PluginHelper::streamObserver = utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver>();

//! Observer object for IMU data stream
class MotionDataObserver : public core::IDataManagerReader::IObserver
{
public:
	MotionDataObserver() {}
	virtual ~MotionDataObserver() {}

	virtual void observe(const core::IDataManagerReader::ChangeList & changes) override
	{
		//pobieram manager hierarchi danych do sprawdzenia czy typ danych mi odpowiada
		auto dhm = plugin::getRegisteredDataTypesManagerReader();
		//lece po wszystkich zmianach
		for (const auto & c : changes)
		{
			//czy zmiana to dodanie danych i czy typ pasuje
			if (c.modyfication == core::IDataManagerReader::ADD_OBJECT &&
				dhm->isBase(c.type, typeid(IMU::CostumeSkeletonMotion)) == true){

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

			//w g�r�
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

			//g�owa
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

			//w g�r�
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

			//g�owa
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
			skeletonMotion->stream->detach(streamObserver);
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
			//s� dane - pobieram
			IMU::MotionStream::value_type ms;
			skeletonMotion->stream->data(ms);

			//TODO - robimy co� z danymi
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