/********************************************************************
    created:  2013/08/21
    created:  21:8:2013   17:44
    filename: DegreesToRadians.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD_MOTION_ANALYSIS__DEGREESTORADIANS_H__
#define HEADER_GUARD_MOTION_ANALYSIS__DEGREESTORADIANS_H__

#include "Types.h"

#include <dflib/Node.h>
#include <dflib/IDFNode.h>

class EulerDegreesToRadiansConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	EulerDegreesToRadiansConverter();
	~EulerDegreesToRadiansConverter();

	virtual void process();
	virtual void reset();

private:
	VectorOutputPin* outPinA;
	VectorInputPin* inPinA;
};

class EulerRadiansToDegreesConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	EulerRadiansToDegreesConverter();
	~EulerRadiansToDegreesConverter();

	virtual void process();
	virtual void reset();

private:
	VectorOutputPin* outPinA;
	VectorInputPin* inPinA;
};

class ScalarDegreesToRadiansConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	ScalarDegreesToRadiansConverter();
	~ScalarDegreesToRadiansConverter();

	virtual void process();
	virtual void reset();

private:
	ScalarOutputPin* outPinA;
	ScalarInputPin* inPinA;
};

class ScalarRadiansToDegreesConverter : public df::ProcessingNode, public df::IDFProcessor
{
public:

	ScalarRadiansToDegreesConverter();
	~ScalarRadiansToDegreesConverter();

	virtual void process();
	virtual void reset();

private:
	ScalarOutputPin* outPinA;
	ScalarInputPin* inPinA;
};

#endif	//	HEADER_GUARD_MOTION_ANALYSIS__DEGREESTORADIANS_H__
