//
//
//  @ Project : IMUPoseControl
//  @ File Name : CQuatIO.h
//  @ Date : 2014-01-02
//  @ Author : Kamil Lebek
//
//

#if !defined(_IMU_CQUATIO_H)
#define _IMU_CQUATIO_H

// Quaternion definition
#include <osg/Quat>
// Message queue
#include <boost/interprocess/ipc/message_queue.hpp>

//! Class for sending and receiving indexed quaternions on local host
class CQuatIO
{
public:
	//! Simple constructor
	/*!
		\param isWriter object can assume writer role (sending orientations) or reader role (reading cached orientations)
		\param maxBoneInd number of supported indexed bones from 0 to (maxBoneInd - 1)
	*/
	CQuatIO(bool isWriter = false, unsigned int maxBoneInd = 50);

	//! Simple destructor
	virtual ~CQuatIO();

	//! Returns quat with given index, identity quat otherwise
	osg::Quat GetQuat(unsigned int quatInd);

	//! Sets given quaternion and sends it via comm channel
	bool SetQuat(const osg::Quat& inQuat, unsigned int quatInd);

private:
	//! Parses all pending message and refreshes internal quaternion buffer
	void RefreshQuats();

	//! Sends quat over communication channel
	bool SendQuat(const osg::Quat& inQuat, unsigned int quatInd);

	//! Communication channel checker
	bool IsCommChannelUp() const
	{
		return (_msgQueue) ? true : false;
	}

	//! Brings communication channel up if needed
	void SpawnCommChannel();

	//! Only writer object can create message queue - others will only read
	bool _isWriter;

	//! Message queue object
	std::unique_ptr<boost::interprocess::message_queue> _msgQueue;

	//! Local copy of all quaternions
	std::vector<osg::Quat> _quatVec;

	//! Internal channel name
	static const char* _channelName;

	//! Internal channel max number of cached messages
	static const unsigned int _channelMaxMsg;

	//! Indexed quaternion serialization object
	__declspec (align(1)) struct SIndexedQuat
	{
		explicit SIndexedQuat(double W, double X, double Y, double Z, unsigned int Ind) :
		_W(W), _X(X), _Y(Y), _Z(Z), _Ind(Ind)
		{}

		double _W, _X, _Y, _Z;
		unsigned int _Ind;
	};
};

#endif // _IMU_CQUATIO_H