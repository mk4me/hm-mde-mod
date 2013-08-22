/********************************************************************
    created:  2013/08/22
    created:  22:8:2013   8:34
    filename: QuaternionCompressorProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONCOMPRESSORPROCESSOR_H__
#define HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONCOMPRESSORPROCESSOR_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class QuaternionSignalCompressor : public df::ProcessingNode, public df::IDFProcessor
{
public:

	QuaternionSignalCompressor();
	~QuaternionSignalCompressor();

	virtual void process();
	virtual void reset();

private:
	QuatCompressedSignalOutputPin* outPinA;
	JointAnglesInputPin* inPinA;
};

class QuaternionSignalDecompressor : public df::ProcessingNode, public df::IDFProcessor
{
public:

	QuaternionSignalDecompressor();
	~QuaternionSignalDecompressor();

	virtual void process();
	virtual void reset();

private:
	JointAnglesOutputPin* outPinA;
	QuatCompressedSignalInputPin* inPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__QUATERNIONCOMPRESSORPROCESSOR_H__
