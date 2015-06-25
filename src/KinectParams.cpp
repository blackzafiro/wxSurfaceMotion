//-----------------------------------------------------------------------------
// <copyright file="KinectParams.cpp" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//-----------------------------------------------------------------------------

#include "stdafx.h"

#include "KinectParams.h"


// ----------------------------------------------------------------------------
// KinectParams
// ----------------------------------------------------------------------------

const float KinectParams::MinimumMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH; // Currently, this is the minimum
const float KinectParams::MaximumMinDepthThreshold = 8.0f;
const float KinectParams::MinimumMaxDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;
const float KinectParams::MaximumMaxDepthThreshold = 8.0f;

/// <summary>
/// Constructor
/// </summary>
KinectParams::KinectParams() :
    m_imageResolution(DEFAULT_IMAGE_RESOLUTION),
	m_fMinDepthThreshold(NUI_FUSION_DEFAULT_MINIMUM_DEPTH),
    m_fMaxDepthThreshold(NUI_FUSION_DEFAULT_MAXIMUM_DEPTH)
{
	scoped_guard paramsGuard(m_mutexParameters);

	UnsafeSetImageResolution(DEFAULT_IMAGE_RESOLUTION);
    DWORD width = 0, height = 0;
    
}

/// <summary>
/// Thread safe copy constructor
/// </summary>
KinectParams::KinectParams(KinectParams& other) {
	scoped_guard paramsGuard(other.m_mutexParameters);

	m_imageResolution = other.m_imageResolution;
	m_cWidth = other.m_cWidth;
	m_cHeight = other.m_cHeight;
	m_cImagePixels = other.m_cImagePixels;

	m_fMinDepthThreshold = other.m_fMinDepthThreshold;
	m_fMaxDepthThreshold = other.m_fMaxDepthThreshold;
}

/// <summary>
/// Thread safe asignment function
/// </summary>
KinectParams& KinectParams::operator=(KinectParams& other) {
	if(this != &other) {
		boost::lock(m_mutexParameters, other.m_mutexParameters);
		scoped_lock paramsLockOther(other.m_mutexParameters, boost::adopt_lock);
		scoped_lock paramsLockThis(m_mutexParameters, boost::adopt_lock);

		m_imageResolution = other.m_imageResolution;
		m_cWidth = other.m_cWidth;
		m_cHeight = other.m_cHeight;
		m_cImagePixels = other.m_cImagePixels;

		m_fMinDepthThreshold = other.m_fMinDepthThreshold;
		m_fMaxDepthThreshold = other.m_fMaxDepthThreshold;
	}
	return *this;
}

/// <summary>
/// Thread safe comparison function
/// </summary>
bool KinectParams::Equals(KinectParams& other) {
	boost::lock(m_mutexParameters, other.m_mutexParameters);
	scoped_lock paramsLockOther(other.m_mutexParameters, boost::adopt_lock);
	scoped_lock paramsLockThis(m_mutexParameters, boost::adopt_lock);

	return (
		m_imageResolution == other.m_imageResolution &&
		m_cWidth == other.m_cWidth &&
		m_cHeight == other.m_cHeight &&
		m_cImagePixels == other.m_cImagePixels &&

		m_fMinDepthThreshold == other.m_fMinDepthThreshold &&
		m_fMaxDepthThreshold == other.m_fMaxDepthThreshold
		);
}

/// <summary>
/// Thread safe function to assign imageResolution and derived parameters.
/// </summary>
void KinectParams::SetImageResolution(NUI_IMAGE_RESOLUTION imageResolution) {
	scoped_guard paramsGuard(m_mutexParameters);

	UnsafeSetImageResolution( imageResolution);
}

/// <summary>
/// Not thread safe function to assign imageResolution and derived parameters.
/// </summary>
void KinectParams::UnsafeSetImageResolution(NUI_IMAGE_RESOLUTION imageResolution) {
	// Get the depth frame size from the NUI_IMAGE_RESOLUTION enum.
    // You can use NUI_IMAGE_RESOLUTION_640x480 or NUI_IMAGE_RESOLUTION_320x240 in this sample.
    // Smaller resolutions will be faster in per-frame computations, but show less detail in reconstructions.
	DWORD width = 0, height = 0;
	m_imageResolution = imageResolution;
	NuiImageResolutionToSize(m_imageResolution, width, height);
    m_cWidth = width;
    m_cHeight = height;
    m_cImagePixels = m_cWidth*m_cHeight;
}

NUI_IMAGE_RESOLUTION KinectParams::GetImageResolution(void) { return m_imageResolution; }
DWORD KinectParams::GetWidth(void) { return m_cWidth; }
DWORD KinectParams::GetHeight(void) { return m_cHeight; }
DWORD KinectParams::GetImagePixels(void) { return m_cImagePixels; }
float KinectParams::GetMinDepthThreshold(void) { return m_fMinDepthThreshold; }
float KinectParams::GetMaxDepthThreshold(void) { return m_fMaxDepthThreshold; }

/// <summary>
/// Meant to be used from the GUI.
/// </summary>
void KinectParams::SetMinDepthThreshold(float minDepthThreshold) {
	m_fMinDepthThreshold = minDepthThreshold;
}
void KinectParams::SetMaxDepthThreshold(float maxDepthThreshold) {
	m_fMaxDepthThreshold = maxDepthThreshold;
}


// ----------------------------------------------------------------------------
// NuiParams
// ----------------------------------------------------------------------------

NumberStringOption::NumberStringOption(int index, UINT value, wxString& string) :
	index(index), value(value), string(string)
{
}

NumberStringOption NuiParams::voxelsPerMeterOptions[VOXELS_PER_METER_OPTIONS_LENGTH] = {
	NumberStringOption(0, 128, wxString("128")),
	NumberStringOption(1, 256, wxString("256")),
	NumberStringOption(2, 384, wxString("384")),
	NumberStringOption(3, 512, wxString("512")),
	NumberStringOption(4, 640, wxString("640")),
	NumberStringOption(5, 768, wxString("768")),
};

NumberStringOption NuiParams::voxelsCountOptions[VOXELS_COUNT_OPTIONS_LENGTH] = {
	NumberStringOption(0, 128, wxString("128")),
	NumberStringOption(1, 256, wxString("256")),
	NumberStringOption(2, 384, wxString("384")),
	NumberStringOption(3, 512, wxString("512")),
	NumberStringOption(4, 640, wxString("640")),
};

/// <summary>
/// Camera pose finder configuration parameters
/// </summary>
const NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS NuiParams::CAMERA_POSE_FINDER_PARAMETERS = {
	NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_FEATURE_LOCATIONS_PER_FRAME_COUNT,
	NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_POSE_HISTORY_COUNT,
	NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_MAX_DEPTH_THRESHOLD
};

/// <summary>
/// Constructor
/// </summary>
NuiParams::NuiParams() :
	m_cMaxIntegrationWeight(NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT),
	m_framesBeforeReset(4)
{
	scoped_guard paramsGuard(m_mutexParameters);

	// Define a cubic Kinect Fusion reconstruction volume, with the sensor at the center of the
    // front face and the volume directly in front of sensor.
    m_reconstructionParams.voxelsPerMeter = 256;    // 1000mm / 256vpm = ~3.9mm/voxel
    m_reconstructionParams.voxelCountX = 512;       // 512 / 256vpm = 2m wide reconstruction
    m_reconstructionParams.voxelCountY = 384;       // Memory = 512*384*512 * 4bytes per voxel
    m_reconstructionParams.voxelCountZ = 512;       // This will require a GPU with at least 512MB
}

/// <summary>
/// Thread safe copy constructor
/// </summary>
NuiParams::NuiParams(NuiParams& other) {
	scoped_guard paramsGuard(other.m_mutexParameters);

	m_reconstructionParams = other.m_reconstructionParams;
	m_cMaxIntegrationWeight = other.m_cMaxIntegrationWeight;
	m_framesBeforeReset = other.m_framesBeforeReset;
}

/// <summary>
/// Thread safe asignment function
/// </summary>
NuiParams& NuiParams::operator=(NuiParams& other) {
	if(this != &other) {
		boost::lock(m_mutexParameters, other.m_mutexParameters);
		scoped_lock paramsLockOther(other.m_mutexParameters, boost::adopt_lock);
		scoped_lock paramsLockThis(m_mutexParameters, boost::adopt_lock);

		m_reconstructionParams = other.m_reconstructionParams;
		m_cMaxIntegrationWeight = other.m_cMaxIntegrationWeight;
		m_framesBeforeReset = other.m_framesBeforeReset;
	}
	return *this;
}

bool operator==(const NUI_FUSION_RECONSTRUCTION_PARAMETERS& left, const NUI_FUSION_RECONSTRUCTION_PARAMETERS& right) {
	return (
		left.voxelCountX == right.voxelCountX &&
		left.voxelCountY == right.voxelCountY &&
		left.voxelCountZ == right.voxelCountZ &&
		left.voxelsPerMeter == right.voxelsPerMeter
		);
}

/// <summary>
/// Thread safe comparison function
/// </summary>
bool NuiParams::Equals(NuiParams& other) {
	boost::lock(m_mutexParameters, other.m_mutexParameters);
	scoped_lock paramsLockOther(other.m_mutexParameters, boost::adopt_lock);
	scoped_lock paramsLockThis(m_mutexParameters, boost::adopt_lock);

	return (
		m_reconstructionParams == other.m_reconstructionParams &&
		m_cMaxIntegrationWeight == other.m_cMaxIntegrationWeight &&
		m_framesBeforeReset == other.m_framesBeforeReset
		);
}

float NuiParams::GetVoxelsPerMeter() {
	return m_reconstructionParams.voxelsPerMeter;
}

UINT NuiParams::GetVoxelCountX() {
	return m_reconstructionParams.voxelCountX;
}
UINT NuiParams::GetVoxelCountY() {
	return m_reconstructionParams.voxelCountY;
}
UINT NuiParams::GetVoxelCountZ() {
	return m_reconstructionParams.voxelCountZ;
}

void NuiParams::SetVoxelsPerMeter(float vpm) {
	m_reconstructionParams.voxelsPerMeter = vpm;
}

void NuiParams::SetVoxelCountX(UINT vx) {
	m_reconstructionParams.voxelCountX = vx;
}
void NuiParams::SetVoxelCountY(UINT vy) {
	m_reconstructionParams.voxelCountY = vy;
}
void NuiParams::SetVoxelCountZ(UINT vz) {
	m_reconstructionParams.voxelCountZ = vz;
}

int NuiParams::GetFramesBeforeReset() {
	return m_framesBeforeReset;
}

void NuiParams::SetFramesBeforeReset(int frames) {
	m_framesBeforeReset = frames;
}

unsigned short NuiParams::GetMaxIntegrationWeight() {
	return m_cMaxIntegrationWeight;
}

/// <sumary>
/// Thread safe function that returns a consistent copy of the parameters.
/// </sumary>
NUI_FUSION_RECONSTRUCTION_PARAMETERS NuiParams::GetReconstructionParameters() {
	scoped_guard paramsGuard(m_mutexParameters);

	return m_reconstructionParams;
}

// ---------------------------------------------------------------------------
// SurfaceReconstruction
// ---------------------------------------------------------------------------

/// <summary>
/// Constructor
/// </summary>
SurfaceReconstruction::SurfaceReconstruction(void) :
	m_pDepthRGBX(NULL),
	m_pColorRGBX(NULL),
	m_pReconstructionRGBX(NULL),
	m_cbImageSize(-1),
	m_frameReady(false),
	m_reconstructionFrameReady(false)
{
}

/// <summary>
/// Destructor
/// </summary>
SurfaceReconstruction::~SurfaceReconstruction(void) {
	FreeBuffers();
}

/// <summary>
/// Initializes/reinitializes each of the frame buffers to the given image size.
/// </summary>
/// <param name="rKinectParams">Number of pixels to allocate in each frame buffer  uses thread safe copy constructor.</param>
HRESULT SurfaceReconstruction::Initialize(KinectParams rKinectParams, NuiParams nuiParams) {
	HRESULT hr = S_OK;

	if (m_currentKinectParams.GetImagePixels() != rKinectParams.GetImagePixels() || NULL == m_pDepthRGBX || NULL == m_pColorRGBX) {
		m_mutexSurfaceBuffers.lock();

		// If bufferes where already initialized, free them.
		UnsafeFreeBuffers();

		ULONG cbImageSize = rKinectParams.GetImagePixels() * KinectParams::BytesPerPixel;
		m_cbImageSize = cbImageSize;

        m_pDepthRGBX = new(std::nothrow) BYTE[cbImageSize];
		m_pColorRGBX = new(std::nothrow) BYTE[cbImageSize];
		m_pReconstructionRGBX = new(std::nothrow) BYTE[cbImageSize];


        if (nullptr != m_pDepthRGBX ||
            nullptr != m_pColorRGBX ||
			nullptr != m_pReconstructionRGBX)
        {
			memset(m_pDepthRGBX,0,cbImageSize);
			memset(m_pColorRGBX,0,cbImageSize);
			memset(m_pReconstructionRGBX,0,cbImageSize);
        }
        else
        {
            FreeBuffers();
            hr = E_OUTOFMEMORY;
        }

		m_mutexSurfaceBuffers.unlock();
	}

	// Using thread safe assignmente operator.
	m_currentKinectParams = rKinectParams;
	m_currentNuiParams = nuiParams;

	return hr;
}

/// <summary>
/// Updates parameters and reinitializes if required.
/// </summary>
/// <param name="rKinectParams">Number of pixels to allocate in each frame buffer uses thread safe copy constructor.</param>
HRESULT SurfaceReconstruction::UpdateParameters(KinectParams rKinectParams, NuiParams nuiParams) {
	// Reinitialize and assign new parameters.
	return Initialize(rKinectParams, nuiParams);
}

/// <summary>
/// Thread unsafe version to frees the buffers.
/// </summary>
void SurfaceReconstruction::UnsafeFreeBuffers() {
    SAFE_DELETE_ARRAY(m_pReconstructionRGBX);
	//SAFE_DELETE_ARRAY(m_pTrackingDataRGBX);
    SAFE_DELETE_ARRAY(m_pDepthRGBX);
    SAFE_DELETE_ARRAY(m_pColorRGBX);

	m_cbImageSize = 0;
	m_frameReady = false;
	m_reconstructionFrameReady = false;
}

/// <summary>
/// Frees the buffers.
/// </summary>
void SurfaceReconstruction::FreeBuffers() {
    scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	UnsafeFreeBuffers();
}

/// <summary>
/// Buffers' size in bytes.
/// </summary>
unsigned long SurfaceReconstruction::GetImageBytesSize(void) {
	return m_cbImageSize;
}

/// <summary>
/// Gives access to a copy of the parameters currently being used by the processing stream.
/// </summary>
KinectParams SurfaceReconstruction::GetCurrentKinectParams(void){
	return m_currentKinectParams;
}

/// <summary>
/// Gives access to a copy of the NUI parameters currently being used by the processing stream.
/// </summary>
NuiParams SurfaceReconstruction::GetCurrentNuiParams(void){
	return m_currentNuiParams;
}

/// <summary>
/// Thread safe store a Kinect Fusion images to depth and color buffers.
/// Accepts Depth Float, and Color image types.
/// </summary>
/// <param name="imageFrame">The depth image frame to store.</param>
/// <param name="imageFrame">The color image frame to store.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT SurfaceReconstruction::StoreImagesToFrameBuffer(
	const NUI_FUSION_IMAGE_FRAME* depthImageFrame,
	const NUI_FUSION_IMAGE_FRAME* colorImageFrame) {
		scoped_guard paramsGuard(m_mutexSurfaceBuffers);

		HRESULT hr;

		hr = AuxUnsafeStoreImageToFrameBuffer(depthImageFrame, m_pDepthRGBX);
		if ( FAILED(hr) ) {
			wxLogError("Failed to store depth nui image to depth buffer");
		}

		hr = AuxUnsafeStoreImageToFrameBuffer(colorImageFrame, m_pColorRGBX);
		if ( FAILED(hr) ) {
			wxLogError("Failed to store color nui image to color buffer");
		}

		m_frameReady = true;

		return hr;
}

/// <summary>
/// Thread safe store a Kinect Fusion images to depth and color buffers.
/// Accepts Depth Float, and Color image types.
/// </summary>
/// <param name="depthColorFrame">The color image frame to store.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT SurfaceReconstruction::StoreReconstructionImageToFrameBuffer(
	const NUI_FUSION_IMAGE_FRAME* depthColorFrame) {
		scoped_guard paramsGuard(m_mutexSurfaceBuffers);

		HRESULT hr;
		hr = AuxUnsafeStoreImageToFrameBuffer(depthColorFrame, m_pReconstructionRGBX);
		if ( FAILED(hr) ) {
			wxLogError("Failed to store color reconstruction nui image to color buffer");
		}

		m_reconstructionFrameReady = true;

		return hr;
}

/// <summary>
/// Store a Kinect Fusion image to a frame buffer.
/// Accepts Depth Float, and Color image types.
/// </summary>
/// <param name="imageFrame">The image frame to store.</param>
/// <param name="buffer">The frame buffer.</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT SurfaceReconstruction::AuxUnsafeStoreImageToFrameBuffer(
    const NUI_FUSION_IMAGE_FRAME* imageFrame,
    BYTE* buffer) {

    HRESULT hr = S_OK;

    if (nullptr == imageFrame || nullptr == imageFrame->pFrameTexture || nullptr == buffer)
    {
		wxLogError("Null arguments.");
        return E_INVALIDARG;
    }

    if (NUI_FUSION_IMAGE_TYPE_COLOR != imageFrame->imageType &&
        NUI_FUSION_IMAGE_TYPE_FLOAT != imageFrame->imageType)
    {
		wxLogError("Neither color nor float.  Image type %d" , imageFrame->imageType);
        return E_INVALIDARG;
    }

    if (0 == imageFrame->width || 0 == imageFrame->height)
    {
		wxLogError("Width or height is cero.");
        return E_NOINTERFACE;
    }

    INuiFrameTexture *imageFrameTexture = imageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        // Convert from floating point depth if required
        if (NUI_FUSION_IMAGE_TYPE_FLOAT == imageFrame->imageType)
        {
            // Depth ranges set here for better visualization, and map to black at 0 and white at 4m
			
            const FLOAT range = 4.0f;
            const FLOAT oneOverRange = (1.0f / range) * 256.0f;
            const FLOAT minRange = 0.0f;

            const float *pFloatBuffer = reinterpret_cast<float *>(LockedRect.pBits);

			for (unsigned int y = 0; y < imageFrame->height; ++y) {
                unsigned int* pColorRow = reinterpret_cast<unsigned int*>(reinterpret_cast<unsigned char*>(buffer) + (y * LockedRect.Pitch));
                const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer) + (y * LockedRect.Pitch));

                for (unsigned int x = 0; x < imageFrame->width; ++x)
                {
                    float depth = pFloatRow[x];

                    // Note: Using conditionals in this loop could degrade performance.
                    // Consider using a lookup table instead when writing production code.
                    BYTE intensity = (depth >= minRange) ?
                        static_cast<BYTE>( (int)((depth - minRange) * oneOverRange) % 256 ) :
                        0; // % 256 to enable it to wrap around after the max range

                    pColorRow[x] = (255 << 24) | (intensity << 16) | (intensity << 8 ) | intensity;
                }
			}
        }
        else	// already in 4 bytes per int (RGBA/BGRA) format
        {

            BYTE * pBuffer = (BYTE *)LockedRect.pBits;

			memcpy(buffer, pBuffer, m_cbImageSize );
        }
    }
    else
    {
		wxLogError("[SurfaceReconstruction] No interface.");
        return E_NOINTERFACE;
    }

    // We're done with the texture so unlock it
    imageFrameTexture->UnlockRect(0);

    return hr;
}

/// <summary>
/// Access to the depth buffer (locks the whole object -- *useless, is a pointer, call lockBuffersFrame first).
/// </summary>
BYTE* SurfaceReconstruction::GetDepthBuffer(void) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	return m_pDepthRGBX;
}

/// <summary>
/// Access to the color buffer (locks the whole object -- *useless, is a pointer, call lockBuffersFrame first).
/// </summary>
BYTE* SurfaceReconstruction::GetColorBuffer(void) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	return m_pColorRGBX;
}

/// <summary>
/// Thread safe access to the color reconstruction buffer.
/// </summary>
BYTE* SurfaceReconstruction::GetColorReconstructionBuffer(void) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	return m_pReconstructionRGBX;
}

/// <summary>
/// Indicates whether there is new frame data ready to render.
/// </summary>
bool SurfaceReconstruction::GetFrameReady(void) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	return m_frameReady;
}

/// <summary>
/// Indicates whether there is new frame reconstruction data ready to render.
/// </summary>
bool SurfaceReconstruction::GetReconstructionFrameReady(void) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	return m_reconstructionFrameReady;
}

/// <summary>
/// Sets whether there is new frame data ready to render.
/// </summary>
void SurfaceReconstruction::SetFrameReady(bool ready) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	m_frameReady = ready;
}

/// <summary>
/// Sets whether there is new frame reconstruction data ready to render.
/// </summary>
void SurfaceReconstruction::SetReconstructionFrameReady(bool ready) {
	scoped_guard paramsGuard(m_mutexSurfaceBuffers);

	m_reconstructionFrameReady = ready;
}

void SurfaceReconstruction::lockBuffersFrame(void) {
	m_mutexSurfaceBuffers.lock();
}

void SurfaceReconstruction::unlockBuffersFrame(void) {
	m_mutexSurfaceBuffers.unlock();
}

