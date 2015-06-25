//------------------------------------------------------------------------------
// <copyright file="wxRGBACanvas.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "windows.h"
#include <wx/panel.h>
#include <wx/rawbmp.h>

#include <boost/thread/mutex.hpp>

class wxRGBACanvas : public wxPanel {
	typedef wxPixelData<wxBitmap, wxAlphaPixelFormat> PixelData;

	typedef boost::mutex::scoped_lock scoped_lock;

private:
	// Mutex for the bitmap
	boost::mutex			m_bitmapMutex;

	// Placeholder for the image data to display
	wxImage*				  m_pImage;

	// Required by the wxBufferedDC to draw on screen and avoid flickering
	// due to cleaning and drawing.
	wxBitmap*               m_pScreenBitmap;

	// Format information
    UINT                    m_sourceHeight;
    UINT                    m_sourceWidth;
    LONG                    m_sourceStride;

public:
	/// <summary>
    /// Constructor
    /// </summary>
	/// <param name="parent">container window</param>
    /// <param name="id">id of the container window</param>
    /// <param name="pos">position of the left corner</param>
    /// <param name="size">size of the canvas = (cDepthWidth, cDepthHeight)</param>
	wxRGBACanvas( wxWindow *parent, wxWindowID id, const wxPoint &pos, const wxSize &size, const wxSize &imageSize );

	/// <summary>
	/// Destructor
	/// </summary>
	~wxRGBACanvas();

    /// <summary>
    /// Draws a 32 bit per pixel image of previously specified width, height, and stride.
    /// </summary>
    /// <param name="pImage">image data in RGBX format</param>
    /// <param name="cbImage">size of image data in bytes</param>
	/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
    HRESULT DrawBGRX(BYTE* pImage, unsigned long cbImage);

	/// <summary>
    /// Draws a 32 bit per pixel image of previously specified width, height, and stride.
    /// </summary>
    /// <param name="pImage">image data in BGR format</param>
    /// <param name="cbImage">size of image data in bytes</param>
	/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
    HRESULT DrawBGR(BYTE* pImage, unsigned long cbImage, bool flip = false);

	/// <summary>
    /// Draws a 32 bit per pixel image of previously specified width, height, and stride.
    /// </summary>
    /// <param name="pImage">image data in RGB format</param>
    /// <param name="cbImage">size of image data in bytes</param>
	/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
    HRESULT DrawRGB(BYTE* pImage, unsigned long cbImage);
};
