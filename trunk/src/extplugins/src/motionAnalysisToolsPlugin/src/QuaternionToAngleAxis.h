/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   23:32
    filename: QuaternionToAngleAxis.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONTOANGLEAXIS_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONTOANGLEAXIS_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class QuaternionToAngleAxisConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	QuaternionToAngleAxisConverter();
	~QuaternionToAngleAxisConverter();

	virtual void process();
	virtual void reset();

private:

	VectorOutputPin* outPinA;
	ScalarOutputPin* outPinB;
	JointAnglesInputPin* inPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONTOANGLEAXIS_H__
