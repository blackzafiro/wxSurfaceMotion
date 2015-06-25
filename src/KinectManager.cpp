//------------------------------------------------------------------------------
// <copyright file="KinectManager.cpp" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
//     Inspired by KinectFusionExplorer-D2D of (c) Microsoft.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"

#include "KinectManager.h"

/// <summary>
/// Sets thread state to terminated and returns.
/// </summary>
#ifndef RETURN_FROM_THREAD
#define RETURN_FROM_THREAD() m_kinectThreadState = TERMINATED; return;
#endif

/// <summary>
/// Constructor
/// </summary>
KinectManager::KinectManager(Blackboard& blackboard) :
	m_pDepthImagePixelBuffer(NULL),
	m_pColorCoordinatesBuffer(NULL),
	m_pDepthFloatImage(NULL),
	m_pColorImage(NULL),
	m_pResampledColorImageDepthAligned(NULL),
	m_pNuiSensor(NULL),
	m_rBlackboard(blackboard), m_kinectThreadState(NOTINITIALIZED),

	m_pVolume(NULL),
	m_pCameraPoseFinder(NULL),
	m_pMapper(NULL),
	m_integrationFrame(0),
	m_pRaycastPointCloudImage(NULL),
	m_pCapturedSurfaceColorImage(NULL)
{ }

/// <summary>
/// Destructor
/// </summary>
KinectManager::~KinectManager(void)
{
	Disconnect();
}

/// <summary>
/// Disconnects the Kinect and releases any initialized components.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::Disconnect(void){
	if (m_rBlackboard.GetProcessingStatus().GetKinectConnected() == ProcessingStatus::DISCONNECTED) return S_OK;

	wxLogMessage("Disconnecting... current status %d", m_rBlackboard.GetProcessingStatus().GetKinectConnected());
	if (m_pNuiSensor) {
        m_pNuiSensor->NuiShutdown();
	}

	// Clean up Kinect Fusion
    SafeRelease(m_pVolume);
	SafeRelease(m_pMapper);

	// Clean up Kinect
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
    SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pColorImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImageDepthAligned);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pRaycastPointCloudImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pCapturedSurfaceColorImage);

	SafeRelease(m_pNuiSensor);
	m_rBlackboard.GetSurfaceReconstruction().FreeBuffers();

	// Clean up synchronization objects
    if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
    }
    if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextColorFrameEvent);
    }

	m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::DISCONNECTED);
	m_rBlackboard.GetProcessingStatus().SetRequestDisconnect(false);
	wxLogMessage("Disconnected. Current status %d", m_rBlackboard.GetProcessingStatus().GetKinectConnected());
	return S_OK;
}


/// <summary>
/// Sets message for Kinnect thread to disconnect the device and finish thread.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::RequestDisconnectKinectThread(void){
	ProcessingStatus::KinectStatus kStatus, oldStatus;
	oldStatus = m_rBlackboard.GetProcessingStatus().GetKinectConnected();

	// Request disconnection and thread finishing.
	m_rBlackboard.GetProcessingStatus().SetRequestDisconnect(true);

	time_t timer1, timer2;
	time(&timer1);
	do {
		kStatus = m_rBlackboard.GetProcessingStatus().GetKinectConnected();
		time(&timer2);
	} while(kStatus != ProcessingStatus::DISCONNECTION_FAILED && kStatus != ProcessingStatus::DISCONNECTED && (timer2 - timer1 < 15) );

	if( kStatus == ProcessingStatus::DISCONNECTED ){
		return S_OK;
	} else {
		// Why could this fail and in which status would the Kinect stay in?
		wxLogMessage("[KinectManager] Disconnection failed... current status %d", m_rBlackboard.GetProcessingStatus().GetKinectConnected());
		m_rBlackboard.GetProcessingStatus().SetKinectConnected(oldStatus);
		return E_FAIL;
	}
}

/// <summary>
/// Allows main thread to join the Kinect thread.
/// </summary>
void KinectManager::join(void) {
	m_kinectThread.join();
}

/// <summary>
/// Informs whether the Kinect thread is running.
/// </summary>
bool KinectManager::isThreadRunning(void) {
	return RUNNING == m_kinectThreadState;
}

/// <summary>
/// Create the first connected Kinect found in a new thread.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::LaunchKinectThread(void){
	m_kinectThread = boost::move(boost::thread(&KinectManager::ProcessingThread, this));
	ProcessingStatus::KinectStatus kStatus;
	do {
		kStatus = m_rBlackboard.GetProcessingStatus().GetKinectConnected();
	} while(kStatus != ProcessingStatus::CONNECTION_FAILED && kStatus != ProcessingStatus::CONNECTED);

	if( kStatus == ProcessingStatus::CONNECTED ){
		return S_OK;
	} else {
		m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::DISCONNECTED);
		return E_FAIL;
	}
}


/// <summary>
/// Carries out all the analysis and surface reconstruction.
/// </summary>
void KinectManager::ProcessingThread(void){
	m_kinectThreadState = RUNNING;

	/// Connect to a Kinect.
	HRESULT hr = this->CreateFirstConnected(); if( FAILED(hr) ) { wxLogError("NuiFusion sensor connection failed."); RETURN_FROM_THREAD() }
	
	/// Set initial parameters
	m_rBlackboard.GetSurfaceReconstruction().Initialize(m_rBlackboard.GetNextKinectParams(), m_rBlackboard.GetNextNuiParams());
	hr = InitializeStreams(); if( FAILED(hr) ) { wxLogError("Streams initialization failed."); RETURN_FROM_THREAD() } else { wxLogMessage("Streams initialized."); }
	hr = InitializeNuiFusion(); if( FAILED(hr) ) { wxLogError("NuiFusion initialization failed."); RETURN_FROM_THREAD() } else { wxLogMessage("NuiFusion initialized."); }

	ProcessingStatus& ps = m_rBlackboard.GetProcessingStatus();
	HANDLE handles[] = { m_hNextDepthFrameEvent, m_hNextColorFrameEvent };
	do {
        DWORD waitResult = WaitForMultipleObjects(ARRAYSIZE(handles), handles, TRUE, INFINITE);
		if (WAIT_OBJECT_0 == waitResult) {
			UpdateParameters();
			hr = ProcessStreams();
			if ( FAILED(hr) ) { wxLogError("Processing failed: %d \nNui sensor status: %d", hr, m_pNuiSensor->NuiStatus()); break; }

			hr = ReconstructVolume();
			if ( FAILED(hr) ) { wxLogError("Volume reconstruction failed: %d \nNui sensor status: %d", hr, m_pNuiSensor->NuiStatus()); }
		}
	} while ( SUCCEEDED(hr) && !ps.GetRequestDisconnect() );

	
	if ( ps.GetRequestDisconnect() ) { wxLogMessage("Disconnection requested"); }
	if ( FAILED( m_pNuiSensor->NuiStatus() ) ) { wxLogError("There is problem with the sensor"); }
	wxLogMessage("Finishing thread execution");

	Disconnect();

	m_kinectThreadState = TERMINATED;
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::CreateFirstConnected(void)
{
	m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::CONNECTING);
    INuiSensor * pNuiSensor;
	HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
    {
		wxLogMessage("[KinectManager] Nui sensor count failed.");
		m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::CONNECTION_FAILED);
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            m_pNuiSensor = pNuiSensor;
			m_sensorIndex = i;
			m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::CONNECTED);
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        SafeRelease(pNuiSensor);
    }

    if (NULL == m_pNuiSensor || FAILED(hr))
    {
		wxLogMessage("[KinectManager] Connection to Nui sensor failed.  Sensor count = %d\nNui will not detect a recently connected device.\nConnect a Kinect and restart the application.", iSensorCount);
		SafeRelease(m_pNuiSensor);
		m_rBlackboard.GetProcessingStatus().SetKinectConnected(ProcessingStatus::CONNECTION_FAILED);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Initializes Kinect data reading.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::InitializeStreams(void){
	assert(NULL != m_pNuiSensor);

	HRESULT hr;

	// Initialize the Kinect and specify that we'll be using depth and color
    hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);

	if( FAILED(hr) ){
		wxLogMessage("Nui initialization failed.");
		return E_FAIL;
	}

	/// Depth
	NUI_IMAGE_RESOLUTION iResolution = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams().GetImageResolution();
    // Create an event that will be signaled when depth data is available
    m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    // Open a depth image stream to receive depth frames
    hr = m_pNuiSensor->NuiImageStreamOpen(
        NUI_IMAGE_TYPE_DEPTH,
        iResolution,
        0,
        2,
        m_hNextDepthFrameEvent,
        &m_pDepthStreamHandle);

	if (SUCCEEDED(hr)) {
		// Create an event that will be signaled when color data is available
		m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		// Open a color image stream to receive color frames
        hr = m_pNuiSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_COLOR,
            iResolution,
            0,
            2,
            m_hNextColorFrameEvent,
            &m_pColorStreamHandle);

		if(SUCCEEDED(hr)) {
            // Create the coordinate mapper for converting color to depth space
            hr = m_pNuiSensor->NuiGetCoordinateMapper(&m_pMapper);
			if( FAILED(hr) ) wxLogError("Failed to create coordinate mapper");

			// TODO: near mode
			return hr;
		} else {
			// Reset the event to non-signaled state
			wxLogError("Failed to open color stream.");
			ResetEvent(m_hNextDepthFrameEvent);
			ResetEvent(m_hNextColorFrameEvent);
			return E_FAIL;
		}

	} else {
		// Reset the event to non-signaled state
		wxLogError("Failed to open depth stream.");
		ResetEvent(m_hNextDepthFrameEvent);
		return E_FAIL;
	}

}

/// <summary>
/// Initializes Nui Fusion images.
/// Must be called after a Kinect has been connected successfully.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::InitializeNuiFusion(void){
	HRESULT hr = S_OK;

	/// NUI FUSION

	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
    WCHAR description[MAX_PATH];
    WCHAR instancePath[MAX_PATH];

	// As we now create a color volume in addition to the depth volume, the memory requirement
    // for a given volume size has doubled. Here we return the total dedicated memory available.
    unsigned int _deviceMemory;

    if (FAILED(hr = NuiFusionGetDeviceInfo(
        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, // Critical, NuiFusion requires DirectX11
        m_sensorIndex, 
        &description[0], 
        ARRAYSIZE(description), 
        &instancePath[0],
        ARRAYSIZE(instancePath), 
        &_deviceMemory)))
    {
        if (hr ==  E_NUI_BADINDEX)
        {
            // This error code is returned either when the device index is out of range for the processor 
            // type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
            // for the device index in the parameters, this indicates that there is no DirectX11 capable 
            // device. The options for users in this case are to either install a DirectX11 capable device
            // (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
            // reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
            //SetStatusMessage(L"No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.");
			return E_FAIL;
        }
        return hr;
    }

	KinectParams kParams = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();

	DWORD width = kParams.GetWidth();
	DWORD height = kParams.GetHeight();
	// Frame generated from the depth input
	if (FAILED(hr = AuxCreateNuiFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, &m_pDepthFloatImage)))
    {
        return hr;
    }
	// Frame generated from the raw color input of Kinect
	if (FAILED(hr = AuxCreateNuiFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pColorImage)))
    {
        return hr;
    }
	// Frame generated from the raw color input of Kinect
	if (FAILED(hr = AuxCreateNuiFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, &m_pRaycastPointCloudImage)))
    {
        return hr;
    }
	// Frame generated from the color volume reconstruction
	if (FAILED(hr = AuxCreateNuiFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pCapturedSurfaceColorImage)))
    {
        return hr;
    }
	// Frame re-sampled from the color input of Kinect, aligned to depth - this will be the same size as the depth.
    if (FAILED(hr = AuxCreateNuiFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, &m_pResampledColorImageDepthAligned)))
    {
        return hr;
    }

	// Depth buffer to store from Kinect
	if (nullptr != m_pDepthImagePixelBuffer) {
        // If buffer length has changed, delete the old one.
		if (kParams.GetImagePixels() != m_cDepthPixelBufferLength) {
            SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
        }
    }
    if (nullptr == m_pDepthImagePixelBuffer) {
        // Depth pixel array to capture data from Kinect sensor
        m_pDepthImagePixelBuffer =
            new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[kParams.GetImagePixels()];

        if (nullptr == m_pDepthImagePixelBuffer) {
            wxLogMessage(L"Failed to initialize Kinect Fusion depth image pixel buffer.");
            return hr;
        }

        m_cDepthPixelBufferLength = kParams.GetImagePixels();
    }

	// Color buffer to store from Kinect
    if (nullptr != m_pColorCoordinatesBuffer) {
        // If buffer length has changed, delete the old one.
        if (kParams.GetImagePixels() != m_cColorCoordinateBufferLength) {
            SAFE_DELETE_ARRAY(m_pColorCoordinatesBuffer);
        }
    }
    if (nullptr == m_pColorCoordinatesBuffer) {
        // Color coordinate array to capture data from Kinect sensor and for color to depth mapping
        // Note: this must be the same size as the depth
        m_pColorCoordinatesBuffer = 
            new(std::nothrow) NUI_COLOR_IMAGE_POINT[kParams.GetImagePixels()];

        if (nullptr == m_pColorCoordinatesBuffer) {
            wxLogMessage(L"Failed to initialize Kinect Fusion color image coordinate buffers.");
            return hr;
        }

        m_cColorCoordinateBufferLength = kParams.GetImagePixels();
    }

	

	// Camera pose finder
	if (nullptr != m_pCameraPoseFinder) {
        SafeRelease(m_pCameraPoseFinder);
    }

    // Create the camera pose finder
    if (nullptr == m_pCameraPoseFinder)
    {
        if (FAILED(hr = NuiFusionCreateCameraPoseFinder(
            &NuiParams::CAMERA_POSE_FINDER_PARAMETERS,
            nullptr,
            &m_pCameraPoseFinder)))
        {
			wxLogError("Could not create camera pose finder.  Error %d", hr);
            return hr;
        }
    }

	// Create reconstruction volume
	NuiParams nParams = m_rBlackboard.GetSurfaceReconstruction().GetCurrentNuiParams();
	hr = RecreateVolume(kParams, nParams);

	return hr;
}

/// <summary>
/// Creates/recreates the volume reconstruction.
/// </summary>
/// <param name="rKinectParams">Reference to the kinect params being setup.</param>
/// <param name="rNuiParams">Reference to the nui kinect params being setup.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::RecreateVolume(KinectParams& rKinectParams, NuiParams& rNuiParams) {
	HRESULT hr = S_OK;

    // Clean up Kinect Fusion
    SafeRelease(m_pVolume);

    NuiMatrix::SetIdentityMatrix(m_worldToCameraTransform);

    // Create the Kinect Fusion Reconstruction Volume
    // Here we create a color volume, enabling optional color processing in the Integrate, ProcessFrame and CalculatePointCloud calls
    hr = NuiFusionCreateColorReconstruction(
		&rNuiParams.GetReconstructionParameters(),
		NuiParams::PROCESSOR_TYPE,	// GPU
		NuiParams::DEVICE_INDEX,												// Automatic selection, use if there is only one volume reconstruction
        &m_worldToCameraTransform,
        &m_pVolume);

    if (FAILED(hr)) {
        if (E_NUI_GPU_FAIL == hr) {
			wxLogError(L"Device not able to run Kinect Fusion, or error initializing.");
        }
        else if (E_NUI_GPU_OUTOFMEMORY == hr) {
			wxLogError(L"Device out of memory error initializing reconstruction - try a smaller reconstruction volume.");
        }
        else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != NuiParams::PROCESSOR_TYPE) {
			wxLogError(L"Failed to initialize Kinect Fusion reconstruction volume on device.");
        }
        else {
			wxLogError(L"Failed to initialize Kinect Fusion reconstruction volume on CPU.");
        }
        return hr;
    }
    else {
        // Save the default world to volume transformation to be used in ResetReconstruction
        hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
        if (FAILED(hr)) {
            wxLogError(L"Failed in call to GetCurrentWorldToVolumeTransform.");
            return hr;
        }

        // This call will set the world-volume transformation
		hr = ResetReconstruction(rKinectParams, rNuiParams);
        if (FAILED(hr)) {
            return hr;
        }

        // Map X axis to blue channel, Y axis to green channel and Z axis to red channel,
        // normalizing each to the range [0, 1].
		/* // Used to shade point cloud
        SetIdentityMatrix(m_worldToBGRTransform);
        m_worldToBGRTransform.M11 = m_paramsCurrent.m_reconstructionParams.voxelsPerMeter / m_paramsCurrent.m_reconstructionParams.voxelCountX;
        m_worldToBGRTransform.M22 = m_paramsCurrent.m_reconstructionParams.voxelsPerMeter / m_paramsCurrent.m_reconstructionParams.voxelCountY;
        m_worldToBGRTransform.M33 = m_paramsCurrent.m_reconstructionParams.voxelsPerMeter / m_paramsCurrent.m_reconstructionParams.voxelCountZ;
        m_worldToBGRTransform.M41 = 0.5f;
        m_worldToBGRTransform.M42 = 0.5f;
        m_worldToBGRTransform.M44 = 1.0f;
		*/

		wxLogMessage(L"Reconstruction has been reset.");
    }

    return hr;
}

/// <summary>
/// Reset the reconstruction camera pose and clear the volume.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::ResetReconstruction(KinectParams& rKinectParams, NuiParams& rNuiParams) {
	if (nullptr == m_pVolume) { return E_FAIL; }

    HRESULT hr = S_OK;

    NuiMatrix::SetIdentityMatrix(m_worldToCameraTransform);

    // Translate the world origin away from the reconstruction volume location by an amount equal
    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
    // If set false, the default world origin is set to the center of the front face of the 
    // volume, which has the effect of locating the volume directly in front of the initial camera
    // position with the +Z axis into the volume along the initial camera direction of view.
    Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

    // Translate the volume in the Z axis by the minDepthThreshold distance
    float minDist = (rKinectParams.GetMinDepthThreshold() < rKinectParams.GetMaxDepthThreshold()) ? rKinectParams.GetMinDepthThreshold() : rKinectParams.GetMaxDepthThreshold();
	worldToVolumeTransform.M43 -= (minDist * rNuiParams.GetVoxelsPerMeter());

    hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);

	if (SUCCEEDED(hr)) {
        m_pCameraPoseFinder->ResetCameraPoseFinder();
    }

    return hr;
}

/// <summary>
/// Auxiliary method to re-create nui fussion images.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::AuxCreateNuiFrame(
    NUI_FUSION_IMAGE_TYPE frameType,
    unsigned int imageWidth,
    unsigned int imageHeight,
    NUI_FUSION_IMAGE_FRAME** ppImageFrame)
{
    HRESULT hr = S_OK;
    
    if (nullptr != *ppImageFrame)
    {
        // If image size or type has changed, release the old one.
        if ((*ppImageFrame)->width != imageWidth ||
            (*ppImageFrame)->height != imageHeight ||
            (*ppImageFrame)->imageType != frameType)
        {
            static_cast<void>(NuiFusionReleaseImageFrame(*ppImageFrame));
            *ppImageFrame = nullptr;
        }
    }

    // Create a new frame as needed.
    if (nullptr == *ppImageFrame)
    {
        hr = NuiFusionCreateImageFrame(
            frameType,
            imageWidth,
            imageHeight,
            nullptr,
            ppImageFrame);

        if (FAILED(hr))
        {
            std::cerr << (L"Failed to initialize Kinect Fusion image.") << std::endl;
        }
    }

    return hr;
}

/// <summary>
/// Handle new depth and color data and perform Kinect Fusion processing
/// </summary>
HRESULT KinectManager::ProcessStreams(void) {
	HRESULT hr = S_OK;

	// Get the next frames from Kinect
	hr = GetKinectFrames();
	if (FAILED(hr)) {
		wxLogError("Failed to get depth and color frames");
		return hr;
	}

	// Copy frames for display
	SurfaceReconstruction& surface = m_rBlackboard.GetSurfaceReconstruction();
	hr = surface.StoreImagesToFrameBuffer(m_pDepthFloatImage, m_pColorImage);

	if ( FAILED(hr) ) {
		wxLogError("Failed to store image to BYTE* buffer.");
	}
	return hr;
};

/// <summary>
/// Updates parameters for the following frame and sets flags for further
/// required changes.
/// <returns>Whether there were changes</returns>
/// </summary>
bool KinectManager::UpdateParameters(void) {
	KinectParams nextParams = m_rBlackboard.GetNextKinectParams();
	KinectParams oldParams = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();

	NuiParams nextNuiParams = m_rBlackboard.GetNextNuiParams();
	NuiParams oldNuiParams = m_rBlackboard.GetSurfaceReconstruction().GetCurrentNuiParams();
	if (nextParams.Equals(oldParams) && nextNuiParams.Equals(oldNuiParams)) return false;

	if (nextParams.GetImageResolution() != oldParams.GetImageResolution()) {
		wxLogError("Can't change resolution while sensor is connected.  Disconnect first.");
		return false;
	}

	m_rBlackboard.GetSurfaceReconstruction().UpdateParameters(nextParams, nextNuiParams);

	if (oldNuiParams.GetVoxelsPerMeter() != nextNuiParams.GetVoxelsPerMeter() ||
			oldNuiParams.GetVoxelCountX() != nextNuiParams.GetVoxelCountX() ||
			oldNuiParams.GetVoxelCountY() != nextNuiParams.GetVoxelCountY() ||
			oldNuiParams.GetVoxelCountZ() != nextNuiParams.GetVoxelCountZ()) {
				// if processorType or deviceIndex could change this should be tested as well
				HRESULT hr = RecreateVolume(nextParams, nextNuiParams);
				if( FAILED(hr) ) { return true; }
				wxLogMessage("Volume recreated.  %f voxels per meter, voxels %d in X, %d in Y and %d in Z .",
					nextNuiParams.GetVoxelsPerMeter(),
					nextNuiParams.GetVoxelCountX(), nextNuiParams.GetVoxelCountY(), nextNuiParams.GetVoxelCountZ());
	}
	return true;
}

/// <summary>
/// Get Extended depth data
/// </summary>
/// <param name="imageFrame">The extended depth image frame to copy.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::AuxCopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame) {
    HRESULT hr = S_OK;

    if (nullptr == m_pDepthImagePixelBuffer) {
        wxLogError(L"Error depth image pixel buffer is nullptr.");
        return E_FAIL;
    }

    INuiFrameTexture *extendedDepthTex = nullptr;

    // Extract the extended depth in NUI_DEPTH_IMAGE_PIXEL format from the frame
    BOOL nearModeOperational;
    hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
        m_pDepthStreamHandle,
        &imageFrame,
        &nearModeOperational,
        &extendedDepthTex);

    if (FAILED(hr)) {
        wxLogError(L"Error getting extended depth texture.");
        return hr;
    }

    NUI_LOCKED_RECT extendedDepthLockedRect;

    // Lock the frame data to access the un-clamped NUI_DEPTH_IMAGE_PIXELs
    hr = extendedDepthTex->LockRect(0, &extendedDepthLockedRect, nullptr, 0);

    if (FAILED(hr) || extendedDepthLockedRect.Pitch == 0) {
        wxLogError(L"Error getting extended depth texture pixels.");
        return hr;
    }

    // Copy the depth pixels so we can return the image frame
    errno_t err = memcpy_s(
        m_pDepthImagePixelBuffer,
        m_cDepthPixelBufferLength * sizeof(NUI_DEPTH_IMAGE_PIXEL),
        extendedDepthLockedRect.pBits,
        extendedDepthTex->BufferLen());
	//memcpy(m_pDepthImagePixelBuffer, extendedDepthLockedRect.pBits, extendedDepthTex->BufferLen());

    extendedDepthTex->UnlockRect(0);
	
    if (0 != err) {
        wxLogError(L"Error copying extended depth texture pixels.");
        return hr;
    }

	
	KinectParams kp = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();
	// Convert the pixels describing extended depth as unsigned short type in millimeters to depth
    // as floating point type in meters.
	hr = m_pVolume->DepthToDepthFloatFrame(
		m_pDepthImagePixelBuffer,
		kp.GetImagePixels() * sizeof(NUI_DEPTH_IMAGE_PIXEL),
		m_pDepthFloatImage,
		kp.GetMinDepthThreshold(),
		kp.GetMaxDepthThreshold(),
		false);

	if ( FAILED(hr) ) {
		wxLogError(L"Error getting depth float frame %s.  Parameters: %p, %d, %d, %p, %f, %f",
			NuiErrorsStringInterface::NuiFusionDepthToDepthFloatFrameError(hr),
			m_pDepthImagePixelBuffer,
			kp.GetWidth(),
			kp.GetHeight(),
            m_pDepthFloatImage,
			kp.GetMinDepthThreshold(),
			kp.GetMaxDepthThreshold());
        return hr;
    }

    return hr;
}


/// <summary>
/// Get Color data
/// </summary>
/// <param name="imageFrame">The color image frame to copy.
/// It uses m_pColorCoordinatesBuffer internally.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectManager::AuxCopyColor(NUI_IMAGE_FRAME &imageFrame) {
    HRESULT hr = S_OK;

    if (nullptr == m_pColorImage) {
        wxLogMessage(L"Error copying color texture pixels.");
        return E_FAIL;
    }

    INuiFrameTexture *srcColorTex = imageFrame.pFrameTexture;
    INuiFrameTexture *destColorTex = m_pColorImage->pFrameTexture;

    if (nullptr == srcColorTex || nullptr == destColorTex)
    {
        return E_NOINTERFACE;
    }

    // Lock the frame data to access the color pixels
    NUI_LOCKED_RECT srcLockedRect;

    hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

    if (FAILED(hr) || srcLockedRect.Pitch == 0) {
        wxLogMessage(L"Error getting color texture pixels.");
        return E_NOINTERFACE;
    }

    // Lock the frame data to access the color pixels
    NUI_LOCKED_RECT destLockedRect;

    hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

    if (FAILED(hr) || destLockedRect.Pitch == 0) {
        srcColorTex->UnlockRect(0);
        wxLogMessage(L"Error copying color texture pixels.");
        return E_NOINTERFACE;
    }

    // Copy the color pixels so we can return the image frame
	
    errno_t err = memcpy_s(
        destLockedRect.pBits, 
		m_cColorCoordinateBufferLength * m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams().BytesPerPixel,
        srcLockedRect.pBits,
        srcLockedRect.size);
	
	
	// Copy and mirror the color pixels so we can return the image frame
	/*
	KinectParams kp = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();
	int pixelSize = kp.BytesPerPixel;
	DWORD columns = kp.GetWidth();
	DWORD rows = kp.GetHeight();
	byte* rowStart = destLockedRect.pBits;
	byte* rowEnd;
	DWORD colIndex, rowIndex;
	for (int i = 0; i < rows; ++i) {
		colIndex = i * columns * pixelSize;
		rowStart = destLockedRect.pBits + colIndex;
		rowEnd = srcLockedRect.pBits + colIndex + (columns - 1) * pixelSize;
		for(int j = 0; j < columns; ++j) {
			for(int c = 0; c < pixelSize; ++c){
				*rowStart = *rowEnd; 
				++rowStart;
				++rowEnd;
			}
			rowEnd -= (2 * pixelSize);
		}
	}
	*/
    srcColorTex->UnlockRect(0);
    destColorTex->UnlockRect(0);

	
    if (0 != err) {
        wxLogMessage(L"Error copying color texture pixels.");
        hr = E_FAIL;
    }
	

    return hr;
}

/// <summary>
/// Adjust color to the same space as depth
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
HRESULT	KinectManager::MapColorToDepth() {
	/// Align color and depth

	HRESULT hr = S_OK;

	// Get the coordinates to convert color to depth space
	KinectParams kParams = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();
	hr = m_pMapper->MapDepthFrameToColorFrame(
		kParams.GetImageResolution(),
		kParams.GetImagePixels(), 
        m_pDepthImagePixelBuffer, 
        NUI_IMAGE_TYPE_COLOR, 
        kParams.GetImageResolution(), 
        kParams.GetImagePixels(),   // the color coordinates that get set are the same array size as the depth image
        m_pColorCoordinatesBuffer );
	if( FAILED(hr) ) wxLogError("Mapping depth to color is not working");

	INuiFrameTexture *srcColorTex = m_pColorImage->pFrameTexture;
    INuiFrameTexture *destColorTex = m_pResampledColorImageDepthAligned->pFrameTexture;

	if (nullptr == srcColorTex || nullptr == destColorTex) {
        wxLogError(L"Error accessing color textures.");
        return E_NOINTERFACE;
    }

    // Lock the source color frame
    NUI_LOCKED_RECT srcLockedRect;

    // Lock the frame data to access the color pixels
    hr = srcColorTex->LockRect(0, &srcLockedRect, nullptr, 0);

    if (FAILED(hr) || srcLockedRect.Pitch == 0) {
        wxLogError(L"Error accessing color texture pixels.");
        return  E_FAIL;
    }

    // Lock the destination color frame
    NUI_LOCKED_RECT destLockedRect;

    // Lock the frame data to access the color pixels
    hr = destColorTex->LockRect(0, &destLockedRect, nullptr, 0);

    if (FAILED(hr) || destLockedRect.Pitch == 0) {
        srcColorTex->UnlockRect(0);
        wxLogError(L"Error accessing color texture pixels.");
        return  E_FAIL;
    }

	int *rawColorData = reinterpret_cast<int*>(srcLockedRect.pBits);
    int *colorDataInDepthFrame = reinterpret_cast<int*>(destLockedRect.pBits);
	KinectParams kp = m_rBlackboard.GetSurfaceReconstruction().GetCurrentKinectParams();
	
	for(int y = 0; y < kp.GetHeight(); ++y) {
		unsigned int destIndex = y * kp.GetWidth();

        // Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
        // to give a viewpoint as though from behind the Kinect looking forward by default.
        unsigned int flippedDestIndex = destIndex + (kp.GetWidth() - 1);

        for (int x = 0; x < kp.GetWidth(); ++x, ++destIndex, --flippedDestIndex)
		//for (int x = 0; x < kp.GetWidth(); ++x, ++destIndex)
        {
            // calculate index into depth array
            int colorInDepthX = m_pColorCoordinatesBuffer[destIndex].x;
            int colorInDepthY = m_pColorCoordinatesBuffer[destIndex].y;

            // make sure the depth pixel maps to a valid point in color space
            if ( colorInDepthX >= 0 && colorInDepthX < kp.GetWidth() 
                && colorInDepthY >= 0 && colorInDepthY < kp.GetHeight()
                && m_pDepthImagePixelBuffer[destIndex].depth != 0)
            {
                // Calculate index into color array- this will perform a horizontal flip as well
                unsigned int sourceColorIndex = colorInDepthX + (colorInDepthY * kp.GetWidth());

                // Copy color pixel
                colorDataInDepthFrame[flippedDestIndex] = rawColorData[sourceColorIndex];
				//colorDataInDepthFrame[destIndex] = rawColorData[sourceColorIndex];
            }
            else
            {
                colorDataInDepthFrame[flippedDestIndex] = 0;
				//colorDataInDepthFrame[destIndex] = 0;
            }
        }
    }
	
	return hr;
}

/// <summary>
/// Get the next frames from Kinect.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT	KinectManager::GetKinectFrames(void) {
	NUI_IMAGE_FRAME imageFrame;
	LONGLONG currentDepthFrameTime = 0;
    LONGLONG currentColorFrameTime = 0;

	// Since we wait for the depth frame event to be signaled we should not need to wait.
	const int timeout = 0;

	////////////////////////////////////////////////////////
    // Get an extended depth frame from Kinect

    HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, timeout, &imageFrame);

    if (FAILED(hr))
    {
        wxLogMessage(L"[KinectManager] Kinect depth stream NuiImageStreamGetNextFrame call failed.");
        return hr;
    }

    hr = AuxCopyExtendedDepth(imageFrame);

    currentDepthFrameTime = imageFrame.liTimeStamp.QuadPart;

    // Release the Kinect depth camera frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

    if (FAILED(hr))
    {
        wxLogMessage(L"[KinectManager] Kinect depth stream NuiImageStreamReleaseFrame call failed.");
        return hr;
    }

    ////////////////////////////////////////////////////////
    // Get a color frame from Kinect

    currentColorFrameTime = m_cLastColorFrameTimeStamp;

    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, timeout, &imageFrame);
    if (FAILED(hr))
    {
        // This shouldn't happen since for the color frame event to be signaled before calling this function.
		wxLogMessage(L"[KinectManager] Kinect color stream NuiImageStreamReleaseFrame call failed.");
		return hr;
    }
    else
    {
        hr = AuxCopyColor(imageFrame);

        currentColorFrameTime = imageFrame.liTimeStamp.QuadPart;

        // Release the Kinect camera frame
        m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

        if (FAILED(hr))
        {
            wxLogMessage(L"[KinectManager] Kinect color stream NuiImageStreamReleaseFrame call failed.");
        }
    }

	m_cLastDepthFrameTimeStamp = currentDepthFrameTime;
    m_cLastColorFrameTimeStamp = currentColorFrameTime;

	return hr;
}







/// <summary>
/// Handle new depth and color data and perform Kinect Fusion processing
/// </summary>
HRESULT	KinectManager::ReconstructVolume(void) {
	HRESULT hr = S_OK;

	SurfaceReconstruction& surface = m_rBlackboard.GetSurfaceReconstruction();
	KinectParams kParams = surface.GetCurrentKinectParams();
	NuiParams nuiParams = surface.GetCurrentNuiParams();

	/// Reset
	/*
	// Reset volume

	Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;
	// Translate the volume in the Z axis by the minDepthThreshold distance
	float minDist = (kParams.GetMinDepthThreshold() < kParams.GetMaxDepthThreshold()) ?
		kParams.GetMinDepthThreshold() : kParams.GetMaxDepthThreshold();
	worldToVolumeTransform.M43 -= (minDist * nuiParams.GetVoxelsPerMeter());
    hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	*/
	// Reset camera pose finder and volume
	if (m_integrationFrame >= nuiParams.GetFramesBeforeReset()) {
		wxLogMessage("Reseting after %d frames", m_integrationFrame);
		ResetReconstruction(kParams, nuiParams);
		m_integrationFrame = 0;
	} else ++m_integrationFrame;

	MapColorToDepth();

	/// Integrate
	// Integrate the depth and color data into the volume from the calculated camera pose
    hr = m_pVolume->IntegrateFrame(
            m_pDepthFloatImage,
			m_pResampledColorImageDepthAligned,
			nuiParams.GetMaxIntegrationWeight(),
            NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES,
            &m_worldToCameraTransform);
	if (FAILED(hr)) { wxLogError(L"Kinect Fusion IntegrateFrame call failed."); }
	
	/// Render
	hr = m_pVolume->CalculatePointCloud(
		m_pRaycastPointCloudImage,
		m_pCapturedSurfaceColorImage,
		&m_worldToCameraTransform); // Render from the sensor's point of view
	if (FAILED(hr)) { wxLogError(L"Kinect Fusion CalculatePointCloud call failed."); }

	// Copy frames for display
	hr = surface.StoreReconstructionImageToFrameBuffer(m_pCapturedSurfaceColorImage);

	if ( FAILED(hr) ) {
		wxLogError("Failed to store reconstruction image to BYTE* buffer.");
	}
	return hr;
}