//------------------------------------------------------------------------------
// <copyright file="KinectManager.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <wx/log.h>
#include "NuiErrorsStringInterface.h"

#include "NuiMatrix.h" // Includes windows.h and  NuiApi.h
#include "Blackboard.h"
#include <boost/thread/thread.hpp>

class KinectManager{

private:
	typedef enum ThreadState {
		NOTINITIALIZED,
		RUNNING,
		WAITING,
		TERMINATED
	} ThreadState;

	static const int            cMinTimestampDifferenceForFrameReSync = 17; // The minimum timestamp difference between depth and color (in ms) at which they are considered un-synchronized. 

	// Current Kinect
	INuiSensor*             m_pNuiSensor;
	int						m_sensorIndex; // Required by NuiFusion

	// Reference to the communication blackboard
	Blackboard&				m_rBlackboard;

	// Kinect thread.
	boost::thread			m_kinectThread;
	ThreadState				m_kinectThreadState;

	// Kinect events
	HANDLE                  m_hNextDepthFrameEvent;
	HANDLE                  m_pDepthStreamHandle;

	HANDLE                  m_hNextColorFrameEvent;
	HANDLE                  m_pColorStreamHandle;

	// Used to synchronize color and depth frames
	LONGLONG                    m_cLastDepthFrameTimeStamp;
    LONGLONG                    m_cLastColorFrameTimeStamp;

	/// <summary>
    /// Frames from the depth input.
    /// </summary>
    NUI_DEPTH_IMAGE_PIXEL*      m_pDepthImagePixelBuffer;
    int                         m_cDepthPixelBufferLength;
	/// <summary>
    /// Image for display.
    /// </summary>
    NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;


	/// <summary>
    /// Frames from the color input.
    /// </summary>
	int                         m_cColorCoordinateBufferLength;
    NUI_COLOR_IMAGE_POINT*      m_pColorCoordinatesBuffer;
	/// <summary>
    /// Color image for display.
    /// </summary>
    NUI_FUSION_IMAGE_FRAME*     m_pColorImage;
	/// <summary>
    /// Color image aligned with depth, used for volume integration.
    /// </summary>
	NUI_FUSION_IMAGE_FRAME*     m_pResampledColorImageDepthAligned;
	/// <summary>
    /// For mapping color to depth
    /// </summary>
	INuiCoordinateMapper*       m_pMapper;


	/// <summary>
    /// The Kinect Fusion reconstruction volume.
    /// </summary>
    INuiFusionColorReconstruction* m_pVolume;
	NUI_FUSION_IMAGE_FRAME*     m_pRaycastPointCloudImage;		// Intermediate result
	NUI_FUSION_IMAGE_FRAME*     m_pCapturedSurfaceColorImage;	// For display

	/// <summary>
    // The Kinect Fusion Camera Transform from world to kinect coordinates.
    /// </summary>
    Matrix4                     m_worldToCameraTransform;

	/// <summary>
    // The default Kinect Fusion World to Volume Transform.
    /// </summary>
    Matrix4                     m_defaultWorldToVolumeTransform;

	/// <summary>
    /// Camera Pose Finder.
    /// Depth and color capture resolutions are set to be equal.
    /// </summary>
    INuiFusionCameraPoseFinder* m_pCameraPoseFinder;

	/// <summary>
	/// To better capture movement, the volume will be reset frequently.
	/// This variable keeps count of how many frames have been processed
	/// without reseting.
	/// <summary>
	int							m_integrationFrame;

	/// <summary>
	/// Create the first connected Kinect found 
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					CreateFirstConnected(void);

	/// <summary>
	/// Initializes Kinect data reading.
	/// Must be called after a Kinect has been connected successfully.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					InitializeStreams(void);

	/// <summary>
	/// Initializes Nui Fusion images.
	/// Must be called after a Kinect has been connected successfully.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					InitializeNuiFusion(void);

	/// <summary>
	/// Creates/recreates the volume reconstruction.
	/// </summary>
	/// <param name="rKinectParams">Reference to the kinect params being setup.</param>
	/// <param name="rNuiParams">Reference to the nui kinect params being setup.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					RecreateVolume(KinectParams& rKinectParams, NuiParams& rNuiParams);

	/// <summary>
	/// Reset the reconstruction camera pose and clear the volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	/// <param name="rKinectParams">Reference to the kinect params being setup.</param>
	HRESULT					ResetReconstruction(KinectParams& rKinectParams, NuiParams& rNuiParams);

	/// <summary>
	/// Carries out all the analysis and surface reconstruction.
	/// </summary>
	void					ProcessingThread(void);
	
	/// <summary>
	/// Auxiliary method to re-create nui fussion images.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					AuxCreateNuiFrame(
		NUI_FUSION_IMAGE_TYPE frameType,
		unsigned int imageWidth,
		unsigned int imageHeight,
		NUI_FUSION_IMAGE_FRAME** ppImageFrame);

	/// <summary>
	/// Store a Kinect Fusion image to a frame buffer.
	/// Accepts Depth Float, and Color image types.
	/// </summary>
	/// <param name="imageFrame">The image frame to store.</param>
	/// <param name="buffer">The frame buffer.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT KinectManager::AuxStoreImageToFrameBuffer(
		const NUI_FUSION_IMAGE_FRAME* imageFrame,
		BYTE* buffer);

	/// <summary>
	/// Updates parameters for the following frame and sets flags for further
	/// required changes.
	/// <returns>Whether there were changes</returns>
	/// </summary>
	bool					UpdateParameters(void);

	/// <summary>
	/// Handle new depth and color data and perform Kinect Fusion processing
	/// </summary>
	HRESULT					ProcessStreams(void);

	/// <summary>
	/// Handle new depth and color data and perform Kinect Fusion processing
	/// </summary>
	HRESULT					ReconstructVolume(void);

	/// <summary>
	/// Get Extended depth data from Kinect to pixel buffer.
	/// </summary>
	/// <param name="imageFrame">The extended depth image frame to copy.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					AuxCopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame);

	/// <summary>
	/// Get Color data from Kinect to pixel buffer.
	/// </summary>
	/// <param name="imageFrame">The color image frame to copy.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					AuxCopyColor(NUI_IMAGE_FRAME &imageFrame);

	/// <summary>
	/// Adjust color to the same space as depth
	/// </summary>
	/// <returns>S_OK for success, or failure code</returns>
	HRESULT					MapColorToDepth();

	/// <summary>
	/// Get the next frames from Kinect, re-synchronizing depth with color.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	//HRESULT					GetColorSynchronizedKinectFrames(void);

	/// <summary>
	/// Get the next frames from Kinect.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					GetKinectFrames(void);

	/// <summary>
	/// Disconnects the Kinect and releases any initialized components.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT 				Disconnect(void);

public:
	/// <summary>
    /// Constructor
    /// </summary>
    KinectManager(Blackboard& blackboard);

	/// <summary>
    /// Destructor
    /// </summary>
    ~KinectManager();

	/// <summary>
	/// Create the first connected Kinect found in a new thread.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					LaunchKinectThread(void);

	/// <summary>
	/// Sets message for Kinnect thread to disconnect the device and finish thread.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT					RequestDisconnectKinectThread(void);

	/// <summary>
	/// Allows main thread to join the Kinect thread.
	/// </summary>
	void					join(void);

	/// <summary>
	/// Informs whether the Kinect thread is running.
	/// </summary>
	bool					isThreadRunning(void);
};