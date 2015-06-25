//------------------------------------------------------------------------------
// <copyright file="KinectParams.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <wx/log.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/lock_algorithms.hpp>		// boost::lock(mtx...)
//#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

class KinectParams {
public:
	// Number of bytes per pixel (applies to both depth float and int-per-pixel color and raycast images)
    static const int            BytesPerPixel = 4;

	/// <summary>
	/// Ranges for depth threshold values.
	/// </summary>
	static const float			MinimumMinDepthThreshold;
	static const float			MaximumMinDepthThreshold;
	static const float			MinimumMaxDepthThreshold;
	static const float			MaximumMaxDepthThreshold;

private:
	static const NUI_IMAGE_RESOLUTION DEFAULT_IMAGE_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

	typedef boost::mutex::scoped_lock scoped_lock;
	typedef boost::lock_guard<boost::mutex> scoped_guard;

	/// <summary>
    /// Lock to be used when thread safe access to this object is required.
    /// </summary>
	boost::mutex				m_mutexParameters;

	/// <summary>
    /// Depth and color image resolution and size
    /// </summary>
    NUI_IMAGE_RESOLUTION        m_imageResolution;
    int                         m_cWidth;
    int                         m_cHeight;
    int                         m_cImagePixels;

	/// <summary>
    /// Used when transforming depth from millimeters in the nui pixel buffer to meters in a nui image frame.
    /// </summary>
    float                       m_fMinDepthThreshold;
    float                       m_fMaxDepthThreshold;

	/// <summary>
    /// Not thread safe function to assign imageResolution and derived parameters.
    /// </summary>
	void UnsafeSetImageResolution(NUI_IMAGE_RESOLUTION imageResolution);
public:
	/// <summary>
	/// Constructor
	/// </summary>
	KinectParams();

	/// <summary>
	/// Thread safe copy constructor
	/// </summary>
	KinectParams(KinectParams& other);

	/// <summary>
	/// Thread safe asignment function
	/// </summary>
	KinectParams& operator=(KinectParams& other);

	/// <summary>
	/// Thread safe comparison function
	/// </summary>
	bool Equals(KinectParams& other);

	/// <summary>
    /// Thread safe function to assign imageResolution and derived parameters.
    /// </summary>
	void SetImageResolution(NUI_IMAGE_RESOLUTION imageResolution);
	NUI_IMAGE_RESOLUTION GetImageResolution(void);
	DWORD GetWidth(void);
	DWORD GetHeight(void);
	DWORD GetImagePixels(void);
	float GetMinDepthThreshold(void);
	float GetMaxDepthThreshold(void);

	/// <summary>
	/// Meant to be used from the GUI.
	/// </summary>
	void SetMinDepthThreshold(float minDepthThreshold);
	void SetMaxDepthThreshold(float maxDepthThreshold);
};

typedef struct NumberStringOption {
		int index;
		UINT value;
		wxString string;

		NumberStringOption(int index, UINT value, wxString& string);
	} NumberStringOption;
inline int NSOvalueToIndex(NumberStringOption* arr, int arrLength, UINT value) {
	for (int i = 0; i< arrLength; ++i) {
		if(arr[i].value == value) return arr[i].index;
	}
	return -1;
}

class NuiParams {
public:
	static int const VOXELS_PER_METER_OPTIONS_LENGTH = 6;
	static NumberStringOption voxelsPerMeterOptions[];

	static int const VOXELS_COUNT_OPTIONS_LENGTH = 5;
	static NumberStringOption voxelsCountOptions[];

    static int const DEVICE_INDEX = -1;    // automatically choose device index for processing

	// This parameter sets whether GPU or CPU processing is used. Note that the CPU will likely be 
    // too slow for real-time processing.
	static NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE const PROCESSOR_TYPE = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;

	/// <summary>
    /// Camera pose finder configuration parameters
    /// </summary>
	static const NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS CAMERA_POSE_FINDER_PARAMETERS;

private:
	typedef boost::mutex::scoped_lock scoped_lock;
	typedef boost::lock_guard<boost::mutex> scoped_guard;

	/// <summary>
    /// Lock to be used when thread safe access to this object is required.
    /// </summary>
	boost::mutex				m_mutexParameters;

	NUI_FUSION_RECONSTRUCTION_PARAMETERS	m_reconstructionParams;

	/// <summary>
	/// Integration parameters.  Integration weight.
	/// </summary>
	unsigned short              m_cMaxIntegrationWeight;

	/// <summary>
	/// Frames before reset of volume reconstruction.
	/// </summary>
	int							m_framesBeforeReset;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	NuiParams();

	/// <summary>
	/// Thread safe copy constructor
	/// </summary>
	NuiParams(NuiParams& other);

	/// <summary>
	/// Thread safe asignment function
	/// </summary>
	NuiParams& operator=(NuiParams& other);

	/// <summary>
	/// Thread safe comparison function
	/// </summary>
	bool Equals(NuiParams& other);

	float GetVoxelsPerMeter();
	UINT GetVoxelCountX();
	UINT GetVoxelCountY();
	UINT GetVoxelCountZ();
	void SetVoxelsPerMeter(float vpm);
	void SetVoxelCountX(UINT vx);
	void SetVoxelCountY(UINT vy);
	void SetVoxelCountZ(UINT vz);

	int GetFramesBeforeReset();
	void SetFramesBeforeReset(int frames);

	unsigned short GetMaxIntegrationWeight();

	/// <sumary>
	/// Thread safe function that returns a consistent copy of the parameters.
	/// </sumary>
	NUI_FUSION_RECONSTRUCTION_PARAMETERS GetReconstructionParameters();
};

class SurfaceReconstruction {
private:
	typedef boost::lock_guard<boost::recursive_mutex> scoped_guard;

	/// <description>
	/// Buffer for display of volume reconstruction rendered image.
	/// </description>
	BYTE*						m_pReconstructionRGBX;
	//BYTE*						m_pTrackingDataRGBX;
	BYTE*						m_pDepthRGBX;
	BYTE*						m_pColorRGBX;

	// Count of bytes in each frame buffer
    unsigned long				m_cbImageSize;

	/// <description>
	/// Flag set when new frames are ready to be displayed.
	/// </description>
	bool						m_frameReady;

	/// <description>
	/// Flag set when new reconstruction frames are ready to be displayed.
	/// </description>
	bool						m_reconstructionFrameReady;

	/// <summary>
    /// Lock to be used when thread safe access to the buffers and buffer info
	/// in this object is required.
    /// </summary>
	boost::recursive_mutex				m_mutexSurfaceBuffers;

	// Parameters used for the data currently withold.
	KinectParams				m_currentKinectParams;

	// Nui Fusion Parameters used for the data currently withold.
	NuiParams				m_currentNuiParams;

	/// <summary>
	/// Thread unsafe version to frees the buffers.
	/// </summary>
	void UnsafeFreeBuffers(void);

	/// <summary>
	/// Store a Kinect Fusion image to a frame buffer.
	/// Accepts Depth Float, and Color image types.
	/// </summary>
	/// <param name="imageFrame">The image frame to store.</param>
	/// <param name="buffer">The frame buffer.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT AuxUnsafeStoreImageToFrameBuffer(
		const NUI_FUSION_IMAGE_FRAME* imageFrame,
		BYTE* buffer);
public:
	/// <summary>
	/// Constructor
	/// </summary>
	SurfaceReconstruction(void);

	/// <summary>
	/// Destructor
	/// </summary>
	~SurfaceReconstruction(void);

	/// <summary>
	/// Initializes/reinitializes each of the frame buffers to the given image size.
	/// </summary>
	/// <param name="rKinectParams">Number of pixels to allocate in each frame buffer uses thread safe copy constructor.</param>
	HRESULT Initialize(KinectParams rKinectParams, NuiParams nuiParams);

	/// <summary>
	/// Updates parameters and reinitializes if required.
	/// </summary>
	/// <param name="rKinectParams">Number of pixels to allocate in each frame buffer uses thread safe copy constructor.</param>
	HRESULT UpdateParameters(KinectParams kinectParams, NuiParams nuiParams);

	/// <summary>
	/// Thread safe funtion to free the buffers.
	/// </summary>
	void FreeBuffers(void);

	/// <summary>
	/// Buffers' size in bytes.
	/// </summary>
	unsigned long GetImageBytesSize(void);

	/// <summary>
	/// Gives access to a copy of the parameters currently being used by the processing stream.
	/// </summary>
	KinectParams GetCurrentKinectParams(void);

	/// <summary>
	/// Gives access to a copy of the NUI parameters currently being used by the processing stream.
	/// </summary>
	NuiParams GetCurrentNuiParams(void);

	/// <summary>
	/// Thread safe store a Kinect Fusion images to depth and color buffers.
	/// Accepts Depth Float, and Color image types.
	/// </summary>
	/// <param name="depthImageFrame">The depth image frame to store.</param>
	/// <param name="depthColorFrame">The color image frame to store.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT StoreImagesToFrameBuffer(
		const NUI_FUSION_IMAGE_FRAME* depthImageFrame,
		const NUI_FUSION_IMAGE_FRAME* depthColorFrame);

	/// <summary>
	/// Thread safe store a Kinect Fusion images to depth and color buffers.
	/// Accepts Depth Float, and Color image types.
	/// </summary>
	/// <param name="depthColorFrame">The color image frame to store.</param>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT StoreReconstructionImageToFrameBuffer(
		const NUI_FUSION_IMAGE_FRAME* depthColorFrame);

	//void SetFrameReady(bool request);
	//bool GetFrameReady(void);

	/// <summary>
	/// Thread safe access to the depth buffer.
	/// </summary>
	BYTE* GetDepthBuffer(void);

	/// <summary>
	/// Thread safe access to the color buffer.
	/// </summary>
	BYTE* GetColorBuffer(void);

	/// <summary>
	/// Thread safe access to the color reconstruction buffer.
	/// </summary>
	BYTE* GetColorReconstructionBuffer(void);

	void lockBuffersFrame(void);
	void unlockBuffersFrame(void);

	/// <summary>
	/// Indicates whether there is new frame data ready to render.
	/// </summary>
	bool GetFrameReady(void);

	/// <summary>
	/// Indicates whether there is new frame reconstruction data ready to render.
	/// </summary>
	bool GetReconstructionFrameReady(void);

	/// <summary>
	/// Sets whether there is new frame data ready to render.
	/// </summary>
	void SetFrameReady(bool ready);

	/// <summary>
	/// Sets whether there is new frame reconstruction data ready to render.
	/// </summary>
	void SetReconstructionFrameReady(bool ready);
};