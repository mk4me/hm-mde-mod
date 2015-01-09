#include <plugins/imuCostume/Streams.h>
#include <corelib/ThreadPool.h>
#include <corelib/IDataManagerReader.h>

class MotionDataObserver;

class PluginHelper
{
	friend class MotionDataObserver;

public:

	static bool init();
	static void deinit();

	static void run();

private:

	static core::IDataManagerReader::ObjectObserverPtr objectObserver;
	static core::Thread streamQuerryingThread;
	static IMU::CostumeSkeletonMotionConstPtr skeletonMotion;
	static volatile bool finish;
	static utils::shared_ptr<threadingUtils::ResetableStreamStatusObserver> streamObserver;
};