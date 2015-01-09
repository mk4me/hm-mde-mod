#include "Plugin.h"
#include <corelib/IPlugin.h>
#include <corelib/IDataHierarchyManagerReader.h>
#include <corelib/ISourceManager.h>
#include <plugins/imuCostume/IIMUDataSource.h>
#include <plugins/imuCostume/IIMUOrientationEstimationAlgorithm.h>
#include <plugins/imuCostume/IMUCostumeCalibrationAlgorithm.h>
#include <plugins/imuCostume/IMUCostumeMotionEstimationAlgorithm.h>

class DummyCalibrationAlgorithm : public IMU::IMUCostumeCalibrationAlgorithm
{
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

//! Generic quaternion-based orientation filter - generates orientation as a quaternion using IMU sensor fusion
class DummyOrientationEstimationAlgorithm : public IMU::IIMUOrientationEstimationAlgorithm
{
public:
	DummyOrientationEstimationAlgorithm() {}
	//! Make it polymorphic
	virtual ~DummyOrientationEstimationAlgorithm() {}

	//! \return Nowy algorytm estymacji
	virtual IIMUOrientationEstimationAlgorithm * create() const override { return new DummyOrientationEstimationAlgorithm; };

	// Public Interface
	//! Returns internal filter name
	virtual std::string name() const override { return "DummyOrientationEstimationAlgorithm"; }

	//! Resets filter internal state
	virtual void reset() override {}

	//! Returns number (n) of frames that are probably unstable after filter create() / reset() - call estimate() at least (n) times
	virtual unsigned int approximateEstimationDelay() const override { return 0; }

	//! Calculates orientation from sensor fusion
	/*!
	\param inAcc accelerometer vector from IMU
	\param inGyro gyroscope vector from IMU
	\param inMag magnetometer vector from IMU
	\param inDeltaT time between acquisitions in seconds [s] from IMU sensor
	\return Returns estimated orientation.
	*/
	virtual osg::Quat estimate(const osg::Vec3d& inAcc,
		const osg::Vec3d& inGyro, const osg::Vec3d& inMag,
		const double inDeltaT) override
	{
		return osg::Quat(0, 0, 0, 1);
	}
};

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
				PluginHelper::streamObserver.reset(new threadingUtils::ResetableStreamStatusObserver);
				PluginHelper::skeletonMotion->stream->attachObserver(PluginHelper::streamObserver);
				PluginHelper::streamQuerryingThread.run(&PluginHelper::run);
				break;
			}
		}
	}
};

kinematic::JointPtr createJoint(kinematic::JointPtr parent,
	const std::string & name, const osg::Vec3 & position,
	const osg::Quat & orientation = osg::Quat(0, 0, 0, 1))
{
	auto ret = utils::make_shared<kinematic::Joint>();
	ret->name = name;
	ret->parent = parent;
	if (parent != nullptr){
		parent->children.push_back(ret);
	}	
	ret->position = position;
	ret->orientation = orientation;
	return ret;
}

bool PluginHelper::init()
{
	bool ret = true;

	static core::IDataManagerReader::ObjectObserverPtr objectObserver;

	try{
		streamQuerryingThread = plugin::getThreadPool()->get("imuCOstumeAlgorithms", "Costume Stream Processing");
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
			dummySkeleton->root = createJoint(kinematic::JointPtr(), "HumanoidRoot", osg::Vec3(0, 0, 0));

			//lewa noga
			auto j = createJoint(dummySkeleton->root, "l_hip", osg::Vec3(16, 0, 0));
			j = createJoint(j, "l_knee", osg::Vec3(0, -50, 0));
			j = createJoint(j, "l_ankle", osg::Vec3(0, -45, 0));
			j = createJoint(j, "l_forefoot_tip", osg::Vec3(0, 0, 26.5));

			//prawa noga
			j = createJoint(dummySkeleton->root, "r_hip", osg::Vec3(-16, 0, 0));
			j = createJoint(j, "r_knee", osg::Vec3(0, -50, 0));
			j = createJoint(j, "r_ankle", osg::Vec3(0, -45, 0));
			j = createJoint(j, "r_forefoot_tip", osg::Vec3(0, 0, 26.5));

			//w górê
			auto vt = createJoint(dummySkeleton->root, "vt1", osg::Vec3(50, 0, 0));
			// w lewo
			j = createJoint(vt, "l_shoulder", osg::Vec3(23.5, 0, 0));
			j = createJoint(j, "l_elbow", osg::Vec3(0, -30, 0));
			j = createJoint(j, "l_wrist", osg::Vec3(0, -30, 0));
			j = createJoint(j, "l_middle_distal_tip", osg::Vec3(0, -17, 0));
			//w prawo
			j = createJoint(vt, "r_shoulder", osg::Vec3(-23.5, 0, 0));
			j = createJoint(j, "r_elbow", osg::Vec3(0, -30, 0));
			j = createJoint(j, "r_wrist", osg::Vec3(0, -30, 0));
			j = createJoint(j, "r_middle_distal_tip", osg::Vec3(0, -17, 0));

			//g³owa
			j = createJoint(vt, "skullbase", osg::Vec3(0, 15, 0));
			j = createJoint(j, "skull_tip", osg::Vec3(0, 23, 0));

			imuDS->registerSkeletonModel(dummySkeleton);
		}
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
			IMU::SkeletonMotionState ms;
			skeletonMotion->stream->data(ms);

			//TODO - ronimy coœ z danymi
		}
		else{
			//nie mam danych - czekam
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

CORE_EXT_PLUGIN_BEGIN("imuCostumeAlgorithms", core::UID::GenerateUniqueID("{3C0C0000-9351-46CC-A5FE-52AA182E1279}"), "en", \
	PluginHelper::init, PluginHelper::deinit, \
	"vendorName", "vendorShortName", "vendorDescription", "vendorContact", \
	1, 0, 0);

CORE_PLUGIN_END;