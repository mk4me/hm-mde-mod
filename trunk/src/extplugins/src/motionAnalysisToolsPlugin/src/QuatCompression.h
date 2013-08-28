/********************************************************************
    created:  2013/08/25
    created:  25:8:2013   12:03
    filename: QuatCompression.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATCOMPRESSION_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATCOMPRESSION_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class MotionAnalysisTests : public df::ProcessingNode, public df::IDFProcessor
{
public:

	MotionAnalysisTests();
	~MotionAnalysisTests();

	virtual void process();
	virtual void reset();	

private:
	VectorInputPin* inPinA;
	ObjectsCollectionOutputPin* outPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATCOMPRESSION_H__
