/********************************************************************
    created:  2013/08/22
    created:  22:8:2013   11:34
    filename: SkeletonCompressorProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__SKELETONCOMPRESSORPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__SKELETONCOMPRESSORPROCESSOR_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>
#include <threading/SynchronizationPolicies.h>
#include <corelib/PluginCommon.h>
#include <corelib/DataAccessors.h>
#include <corelib/IJobManager.h>
#include <corelib/IJob.h>
#include <boost/bind.hpp>

template<class LS>
class SkeletonCompressor : public df::ProcessingNode, public df::IDFProcessor
{
public:

	SkeletonCompressor()
	{
		inPinA = new JointAnglesCollectionInputPin(this);
		outPinA = new JointAnglesCollectionOutputPin(this);
		addInputPin(inPinA);
		addOutputPin(outPinA);
	}

	~SkeletonCompressor()
	{

	}

	void process()
	{
		auto inSkeleton = inPinA->getValue();
		auto jobManager = plugin::getJobManager();

		const auto size = inSkeleton->getNumChannels();
		jobs = size;
		std::vector<kinematic::JointAngleChannelPtr> channels;
		channels.resize(size);

		const auto samplesSize = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(inSkeleton->getNumPointsPerChannel());
		QuatUtils::QuatLiftingCompressor::CompressionSettings settings;
		std::string postfix;
		{

			std::stringstream ss;			
			auto resolutionsCount = LiftingScheme::LiftingSchemeUtils::ilog2(samplesSize);

			const int toRemove = 3;

			int idx = 0;
			int res = resolutionsCount-1;

			while(idx++ < toRemove && res > 0){
				ss << res << "-";
				settings.insert(res--);
			}

			postfix = ss.str();
			postfix.erase(postfix.size()-1, 1);
		}

		sync.lock();

		for(unsigned int idx = 0; idx < size; ++idx){

			jobManager->addJob("VDF->SkeletonCompressor", "JointCompressionDecompression",
				utils::IRunnablePtr(new utils::FunctorRunnable(boost::bind(&SkeletonCompressor::processSingleJoint,
				this, inSkeleton, &channels, idx, settings, postfix, samplesSize))));

		}

		utils::ScopedLock<utils::StrictSyncPolicy> lock(sync);

		kinematic::JointAnglesCollectionPtr outSkeleton(new kinematic::JointAnglesCollection);
		outSkeleton->setConfigurationID(inSkeleton->getConfigurationID());
		outSkeleton->setLengthRatio(inSkeleton->getLengthRatio());
		auto rootPositions = inSkeleton->getRootPositions();
		rootPositions.resize(samplesSize);
		outSkeleton->setSkeletal(inSkeleton->getHAnimSkeleton(), rootPositions, channels);

		outPinA->setValue(outSkeleton);
	}

	void reset()
	{

	}

private:

	void processSingleJoint(kinematic::JointAnglesCollectionConstPtr inData,
		std::vector<kinematic::JointAngleChannelPtr> * outData, const int idx,
		const QuatUtils::QuatLiftingCompressor::CompressionSettings & compressionSettings,
		const std::string & postfix, const unsigned int size)
	{
		auto channel = inData->getChannel(idx);

		kinematic::JointAngleChannelPtr newChannel(new kinematic::JointAngleChannel(channel->getSamplesPerSecond()));
		newChannel->setName("Compressed channel: " + postfix );
		newChannel->setTimeBaseUnit(channel->getTimeBaseUnit());
		newChannel->setValueBaseUnit(channel->getValueBaseUnit());

		QuatUtils::QuatLiftingScheme::Data lsData;
		lsData.reserve(size);

		//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			lsData.push_back(channel->value(i));
		}
		
		LS ls;
		ls.forwardTrans(lsData, size);

		auto res = QuatUtils::QuatLiftingCompressor::compress(lsData, compressionSettings);

		lsData = QuatUtils::QuatLiftingCompressor::decompress(res);

		ls.inverseTrans(lsData, size);

		//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			newChannel->addPoint(lsData[i]);
		}

		(*outData)[idx] = newChannel;

		utils::ScopedLock<utils::StrictSyncPolicy> lock(stateSync);
		--jobs;
		if(jobs == 0){
			sync.unlock();
		}
	}

private:

	utils::StrictSyncPolicy sync;
	utils::StrictSyncPolicy stateSync;
	unsigned int jobs;

	JointAnglesCollectionOutputPin* outPinA;
	JointAnglesCollectionInputPin* inPinA;
};

class SkeletonSource : public df::SourceNode, public df::IDFSource
{
public:
	SkeletonSource()
	{
		outPinA = new JointAnglesCollectionOutputPin(this);
		addOutputPin(outPinA);
		used = false;
	}

	~SkeletonSource() {}


	virtual void reset()
	{
		used = false;
	}

	virtual const bool empty() const
	{
		return used;
	}

	virtual void produce()
	{
		utils::ConstObjectsList objects;

		plugin::getDataManagerReader()->getObjects(objects, typeid(kinematic::JointAnglesCollection), false);		

		if(objects.empty() == false){			
			outPinA->setWrapper(objects.front());
		}

		used = true;
	}

private:

	JointAnglesCollectionOutputPin * outPinA;	
	bool used;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__SKELETONCOMPRESSORPROCESSOR_H__
