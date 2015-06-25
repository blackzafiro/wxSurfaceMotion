//------------------------------------------------------------------------------
// <copyright file="wxMainWindow.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------
// Shared memory between threads.
//------------------------------------------------------------------------------

#pragma once

#include "wxRGBACanvas.h"
#include "KinectParams.h"
#include <boost/thread/condition.hpp>

class ProcessingStatus
{
public:
	typedef enum KinectStatus {
		DISCONNECTION_FAILED = -2,
		CONNECTION_FAILED = -1,
		DISCONNECTED = 0,
		CONNECTING,
		CONNECTED
	} KinectStatus;

private:
	// Kinect connected
	KinectStatus			m_kinectConnected;
	boost::condition		m_kinectConnectedCond;

	bool					m_requestDisconnect;

public:
	ProcessingStatus();
	void SetKinectConnected(KinectStatus kinectConnected);
	KinectStatus GetKinectConnected(void);
	boost::condition& GetCondition(void);
	void SetRequestDisconnect(bool request);
	bool GetRequestDisconnect(void);
};

class Blackboard
{
private:
	// Parameters

	/// <description>
	/// Only thread safe constructor and assingment operators are to be used with this object.
	/// </description>
	//KinectParams			m_currentKinectParams;

	/// <description>
	/// Object where the GUI can set user choices.
	/// </description>
	KinectParams			m_nextKinectParams;
	NuiParams				m_nextNuiParams;

	ProcessingStatus		m_processingStatus;
	SurfaceReconstruction	m_surfaceReconstruction;

public:
	Blackboard(void);
	~Blackboard(void);
	ProcessingStatus& GetProcessingStatus(void);

	/// <summary>
	/// Gives access to the parameters that can be set from the user interface.
	/// </summary>
	KinectParams& GetNextKinectParams(void);

	/// <summary>
	/// Gives access to the nui parameters that can be set from the user interface.
	/// </summary>
	NuiParams& GetNextNuiParams(void);

	/// <summary>
	/// Gives access to image's and volume's information.
	/// </summary>
	SurfaceReconstruction& GetSurfaceReconstruction(void);

};

