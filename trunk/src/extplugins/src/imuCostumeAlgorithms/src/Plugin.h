#include <plugins/imuCostume/Streams.h>
#include <corelib/Thread.h>
#include <corelib/IDataManagerReader.h>

class MotionDataObserver;

//! Data aquisition plugin helper class
class PluginHelper
{
	friend class MotionDataObserver;

public:

	static bool init();
	static void deinit();

	static void run();

private:

	static core::IDataManagerReader::ObserverPtr objectObserver;
	static core::Thread streamQuerryingThread;
	static IMU::CostumeSkeletonMotionConstPtr skeletonMotion;
	static volatile bool finish;
	static utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver> streamObserver;
};