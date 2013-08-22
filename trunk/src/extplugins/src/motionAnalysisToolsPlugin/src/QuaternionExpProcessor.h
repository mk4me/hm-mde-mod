/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   15:46
    filename: QuaternionExpProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEXPPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEXPPROCESSOR_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class QuaternionExpProcessor: public df::ProcessingNode, public df::IDFProcessor
{	

public:

	QuaternionExpProcessor();
	~QuaternionExpProcessor();

	virtual void process();
	virtual void reset();

private:	
	JointAnglesOutputPin* outPinA;
	JointAnglesInputPin* inPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONEXPPROCESSOR_H__
