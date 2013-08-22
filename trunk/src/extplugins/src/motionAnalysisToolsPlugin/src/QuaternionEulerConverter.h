/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   15:12
    filename: QuaternionEulerConverter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEULERCONVERTER_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEULERCONVERTER_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class QuaternionToEulerConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	QuaternionToEulerConverter();
	~QuaternionToEulerConverter();

	virtual void process();
	virtual void reset();

private:
	VectorOutputPin* outPinA;
	JointAnglesInputPin* inPinA;
};

class EulerToQuaternionConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	EulerToQuaternionConverter();
	~EulerToQuaternionConverter();

	virtual void process();
	virtual void reset();

private:
	JointAnglesOutputPin* outPinA;
	VectorInputPin* inPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEULERCONVERTER_H__
