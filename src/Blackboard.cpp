#include "Blackboard.h"

/// ProcessingStatus

ProcessingStatus::ProcessingStatus() :
	m_kinectConnected(DISCONNECTED),
	m_requestDisconnect(false)
{
}

void ProcessingStatus::SetKinectConnected(KinectStatus kinectConnected){
	m_kinectConnected = kinectConnected;
	m_kinectConnectedCond.notify_all();
}

ProcessingStatus::KinectStatus ProcessingStatus::GetKinectConnected(void){
	return m_kinectConnected;
}

boost::condition& ProcessingStatus::GetCondition(void){
	return m_kinectConnectedCond;
}

void ProcessingStatus::SetRequestDisconnect(bool request){
	m_requestDisconnect = request;
}

bool ProcessingStatus::GetRequestDisconnect(void){
	return m_requestDisconnect;
}


/// Blackboard

Blackboard::Blackboard(void)
{
}

Blackboard::~Blackboard(void)
{
}

ProcessingStatus& Blackboard::GetProcessingStatus(void){
	return m_processingStatus;
}

/// <summary>
/// Gives access to the parameters that can be set from the user interface.
/// </summary>
KinectParams& Blackboard::GetNextKinectParams(void){
	return m_nextKinectParams;
}

/// <summary>
/// Gives access to the nui parameters that can be set from the user interface.
/// </summary>
NuiParams& Blackboard::GetNextNuiParams(void) {
	return m_nextNuiParams;
}

/// <summary>
/// Gives access to image's and volume's information.
/// </summary>
SurfaceReconstruction& Blackboard::GetSurfaceReconstruction(void){
	return m_surfaceReconstruction;
}
