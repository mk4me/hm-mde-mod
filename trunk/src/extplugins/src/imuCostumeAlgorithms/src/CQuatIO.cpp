//
//
//  @ Project : IMUPoseControl
//  @ File Name : CQuatIO.h
//  @ Date : 2014-01-02
//  @ Author : Kamil Lebek
//
//

#include "CQuatIO.h"

const char* CQuatIO::_channelName = "QuatIOChann_v1.0"; // API version string
const unsigned int CQuatIO::_channelMaxMsg = 200; // Might need increasing if transmission stutters

//! Simple constructor
CQuatIO::CQuatIO(bool isWriter, unsigned int maxBoneInd) : _isWriter(isWriter)
{
	// Comm channel is spawned on demand
	// ...

	// Setup bone index and quat table
	if (maxBoneInd <= 0)
		maxBoneInd = 1;

	// Construct all vectors
	_quatVec.resize(maxBoneInd, osg::Quat(0.0, 0.0, 0.0, 1.0));
}

//! Simple destructor
CQuatIO::~CQuatIO()
{
	// If we were in writer mode - delete channel
	if (_isWriter && IsCommChannelUp())
	{
		// no-throw destructor guarantee
		try
		{
			boost::interprocess::message_queue::remove(_channelName);
		}
		catch(...)
		{
		}
	}
}

//! Returns quat with given index, identity quat otherwise
osg::Quat CQuatIO::GetQuat(unsigned int quatInd)
{
	// Out of bounds
	if (quatInd >= _quatVec.size())
		return osg::Quat(0.0, 0.0, 0.0, 1.0);

	// We might need to update internal list in reader's case
	if (!_isWriter)
		RefreshQuats();

	// Access internal quat buffer
	return _quatVec[quatInd];
}

//! Sets given quaternion and sends it via comm channel
bool CQuatIO::SetQuat(const osg::Quat& inQuat, unsigned int quatInd)
{
	// Out of bounds
	if (quatInd >= _quatVec.size())
		return false; 

	// Reader does not send
	if (!_isWriter)
		return false;

	// Try sending
	if (!SendQuat(inQuat, quatInd))
		return false;

	// Internal update after successful send
	_quatVec[quatInd] = inQuat;
	return true;
}

//! Parses all pending message and refreshes internal quaternion buffer
void CQuatIO::RefreshQuats()
{
	// Check if there's a connection
	SpawnCommChannel();
	if (!IsCommChannelUp())
		return;

	size_t tmpSize;
	unsigned int tmpPrio;
	SIndexedQuat tmpQuat(1.0, 0.0, 0.0, 0.0, 0);

	// Handle exceptions
	try
	{
		// Remove all messages
		while (_msgQueue->try_receive(&tmpQuat, sizeof(tmpQuat), tmpSize, tmpPrio))
		{
			// Wrong message size? Discard.
			if (tmpSize != sizeof(tmpQuat))
				continue;

			// Check index - out of range? Discard.
			if (tmpQuat._Ind >= _quatVec.size())
				continue;

			// All checks passed, writer quat to internal vector
			_quatVec[tmpQuat._Ind] = osg::Quat(tmpQuat._X, tmpQuat._Y, tmpQuat._Z, tmpQuat._W);
		}
	}
	catch (...)
	{
		// Something went WONG
	}
}

//! Sends quat over communication channel
bool CQuatIO::SendQuat(const osg::Quat& inQuat, unsigned int quatInd)
{
	// Check if there's a connection
	SpawnCommChannel();
	if (!IsCommChannelUp())
		return false;

	// Handle exceptions
	try
	{
		SIndexedQuat tmpQuat(inQuat.w(), inQuat.x(), inQuat.y(), inQuat.z(), quatInd);
		return _msgQueue->try_send(&tmpQuat, sizeof(tmpQuat), 0);
	}
	catch (...)
	{
		// Something went WONG
	}

	return false;
}

//! Brings communication channel up if needed
void CQuatIO::SpawnCommChannel()
{
	if (IsCommChannelUp())
		return;

	try
	{
		// Writer case
		if (_isWriter)
			_msgQueue.reset(new boost::interprocess::message_queue(boost::interprocess::open_or_create, _channelName, _channelMaxMsg, sizeof(SIndexedQuat)));
		else
			_msgQueue.reset(new boost::interprocess::message_queue(boost::interprocess::open_only, _channelName));
	}
	catch(...)
	{
	}
}