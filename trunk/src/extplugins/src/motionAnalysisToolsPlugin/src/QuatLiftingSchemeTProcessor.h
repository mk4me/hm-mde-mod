/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   16:27
    filename: QuatLiftingSchemeTProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATLIFTINGSCHEMETPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATLIFTINGSCHEMETPROCESSOR_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>
#include <GeneralAlgorithms/LiftingScheme/LiftingSchemeT.h>
#include <QuatUtils/QuaternionInterpolators.h>

template<class Processor>
class QuaternionProcessorT : public df::ProcessingNode, public df::IDFProcessor
{	

public:

	QuaternionProcessorT()
	{
		inPinA = new JointAnglesInputPin(this);
		outPinA = new JointAnglesOutputPin(this);
		addInputPin(inPinA);
		addOutputPin(outPinA);
	}

	~QuaternionProcessorT() {}

	virtual void process()
	{
		Processor::process(inPinA, outPinA);
	}

	virtual void reset() {}

private:	
	JointAnglesOutputPin* outPinA;
	JointAnglesInputPin* inPinA;
};

template<bool forward, class LS>
class TransformProcessor
{
public:
	static void process(JointAnglesInputPin * in, JointAnglesOutputPin * out)
	{
		auto inQuatData = in->getValue();

		//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
		kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
		outQuatData->setName("Forward lifting scheme");
		outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
		outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

		QuatUtils::QuatLiftingScheme::Data lsData;
		const double size = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(inQuatData->size());

		//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			lsData.push_back(inQuatData->value(i));
		}

		LS ls;
		ls.forwardTrans(lsData, size);

		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			outQuatData->addPoint(lsData[i]);
		}

		//zapisujê zmodyfikowane dane do udostêpniena
		out->setValue(outQuatData);
	}
};

template<class LS>
class TransformProcessor<false, LS>
{
public:

	static void process(JointAnglesInputPin * in, JointAnglesOutputPin * out)
	{
		auto inQuatData = in->getValue();

		//kopiuje go - przygotowuje sobie porcje danych wyjsciowych
		kinematic::JointAngleChannelPtr outQuatData(new kinematic::JointAngleChannel(inQuatData->getSamplesPerSecond()));
		outQuatData->setName("Inverse lifting scheme");
		outQuatData->setTimeBaseUnit(inQuatData->getTimeBaseUnit());
		outQuatData->setValueBaseUnit(inQuatData->getValueBaseUnit());

		QuatUtils::QuatLiftingScheme::Data lsData;
		const double size = LiftingScheme::LiftingSchemeUtils::floorPowerOfTwo(inQuatData->size());

		//modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu	
		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			lsData.push_back(inQuatData->value(i));
		}

		LS ls;
		ls.inverseTrans(lsData, size);

		for(kinematic::JointAngleChannel::size_type i = 0; i < size; ++i){
			outQuatData->addPoint(lsData[i]);
		}

		//zapisujê zmodyfikowane dane do udostêpniena
		out->setValue(outQuatData);
	}
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATLIFTINGSCHEMETPROCESSOR_H__
