//------------------------------------------------------------------------------
// <copyright file="wxDepthCanvas.cpp" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#include "wxRGBACanvas.h"
#include <wx/dcbuffer.h>
#include <wx/log.h>


/// <summary>
/// Constructor
/// </summary>
/// <param name="parent">container window</param>
/// <param name="id">id of the container window</param>
/// <param name="pos">position of the left corner</param>
/// <param name="size">size of the canvas = (cDepthWidth, cDepthHeight)</param>
wxRGBACanvas::wxRGBACanvas( wxWindow *parent, wxWindowID id, const wxPoint &pos, const wxSize &size, const wxSize &imageSize ) :
	m_pImage(NULL),
	m_pScreenBitmap(NULL),
	wxPanel( parent, id, pos, size, wxSUNKEN_BORDER ) {
		SetBackgroundStyle(wxBG_STYLE_PAINT);
		m_sourceHeight = imageSize.GetHeight();
		m_sourceWidth = imageSize.GetWidth();
		m_sourceStride = m_sourceWidth * sizeof(long);

		// For performance reasons 32 bits is better even if alpha channel is not used.
		//m_pBitmap = new wxBitmap(m_sourceWidth, m_sourceHeight, 32);
		m_pScreenBitmap = new wxBitmap(m_sourceWidth, m_sourceHeight, 32);
}


/// <summary>
/// Destructor
/// </summary>
wxRGBACanvas::~wxRGBACanvas(){
	if (m_pScreenBitmap != NULL) {
		delete m_pScreenBitmap;
	}
}


/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride.
/// It follows the indications in wx/rawbmp.h according to the discussion in
/// https://forums.wxwidgets.org/viewtopic.php?t=8729&p=39768
/// </summary>
/// <param name="pImage">image data in RGBX format, it must be locked before calling this function.</param>
/// <param name="cbImage">size of image data in bytes, it must be locked before calling this function.</param>
/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
HRESULT wxRGBACanvas::DrawBGRX(BYTE* pImage, unsigned long cbImage){

	// Only one thread can draw on m_pBitmap
	scoped_lock lock(m_bitmapMutex);

	// Incorrectly sized image data passed in.
    if ( cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4) ) {
		wxLogError("[wxRGBACanvas] image size mismatch, %d requested.", cbImage);
        return E_INVALIDARG;
    }

	m_pImage = new wxImage(m_sourceWidth, m_sourceHeight);
	m_pImage->InitAlpha();
	// Requesting access to pixel data for copying the image to the canvas.
	wxImagePixelData data(*m_pImage);
    if ( !data ) {
        wxLogError(_T("[wxRGBACanvas] Failed to gain raw access to bitmap data"));
		return E_ACCESSDENIED;
    }

	wxImagePixelData::Iterator p(data);
	const BYTE* s = pImage;

	for ( unsigned int y = 0, i = 0; y < m_sourceHeight; ++y ) {
		wxImagePixelData::Iterator rowStart = p;

        for ( unsigned int x = 0; x < m_sourceWidth; ++x, ++p, i += 4 )
        {
            p.Red() = s[i+2]; // s[i];
            p.Green() = s[i+1];
            p.Blue() = s[i]; // s[i+2];
			p.Alpha() = s[i+3];
        }

        p = rowStart;
        p.OffsetY(data, 1);
    }

	m_pImage->Rescale(this->GetSize().GetWidth(), this->GetSize().GetHeight());

	wxBitmap pBitmap(*m_pImage, 32);

	// Stack object used to paint on screen.
	wxClientDC dc(this);
	wxBufferedDC bdc(&dc, *m_pScreenBitmap);
	bdc.SetBackground(*wxBLACK_BRUSH);
	bdc.Clear();
	bdc.DrawBitmap(pBitmap, 0, 0);

	// Request updating the displayed image.
	this->Update();

	delete m_pImage;
	return true;
}


/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride.
/// It follows the indications in wx/rawbmp.h according to the discussion in
/// https://forums.wxwidgets.org/viewtopic.php?t=8729&p=39768
/// </summary>
/// <param name="pImage">image data in BGR format, it must be locked before calling this function.</param>
/// <param name="cbImage">size of image data in bytes, it must be locked before calling this function.</param>
/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
HRESULT wxRGBACanvas::DrawBGR(BYTE* pImage, unsigned long cbImage, bool flip){

	// Only one thread can draw on m_pBitmap
	scoped_lock lock(m_bitmapMutex);

	// Incorrectly sized image data passed in.
    if ( cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4) ) {
		wxLogError("[wxRGBACanvas] image size mismatch, %d requested.", cbImage);
        return E_INVALIDARG;
    }

	m_pImage = new wxImage(m_sourceWidth, m_sourceHeight);
	m_pImage->InitAlpha();
	// Requesting access to pixel data for copying the image to the canvas.
	wxImagePixelData data(*m_pImage);
    if ( !data ) {
        wxLogError(_T("[wxRGBACanvas] Failed to gain raw access to bitmap data"));
		return E_ACCESSDENIED;
    }

	wxImagePixelData::Iterator p(data);
	const BYTE* s = pImage;

	for ( unsigned int y = 0, i = 0; y < m_sourceHeight; ++y ) {
		wxImagePixelData::Iterator rowStart = p;

        for ( unsigned int x = 0; x < m_sourceWidth; ++x, ++p, i += 4 )
        {
            p.Red() = s[i+2]; // s[i];
            p.Green() = s[i+1];
            p.Blue() = s[i]; // s[i+2];
			p.Alpha() = 255;
        }

        p = rowStart;
        p.OffsetY(data, 1);
    }

	m_pImage->Rescale(this->GetSize().GetWidth(), this->GetSize().GetHeight());

	wxImage mirrored;
	if (flip) {
		mirrored  = m_pImage->Mirror();
		delete m_pImage;
		m_pImage = &mirrored;
	}

	wxBitmap pBitmap(*m_pImage, 32);

	// Stack object used to paint on screen.
	wxClientDC dc(this);
	wxBufferedDC bdc(&dc, *m_pScreenBitmap);
	bdc.SetBackground(*wxBLACK_BRUSH);
	bdc.Clear();
	bdc.DrawBitmap(pBitmap, 0, 0);

	// Request updating the displayed image.
	this->Update();

	if(!flip) delete m_pImage;
	return true;
}


/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride.
/// It follows the indications in wx/rawbmp.h according to the discussion in
/// https://forums.wxwidgets.org/viewtopic.php?t=8729&p=39768
/// </summary>
/// <param name="pImage">image data in RGB format, it must be locked before calling this function.</param>
/// <param name="cbImage">size of image data in bytes, it must be locked before calling this function.</param>
/// <returns>indicates success or failure (<code>E_INVALIDARG</code>)</returns>
HRESULT wxRGBACanvas::DrawRGB(BYTE* pImage, unsigned long cbImage){

	// Only one thread can draw on m_pBitmap
	scoped_lock lock(m_bitmapMutex);

	// Incorrectly sized image data passed in.
    if ( cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4) ) {
		wxLogError("image size mismatch, %d requested.", cbImage);
        return E_INVALIDARG;
    }

	m_pImage = new wxImage(m_sourceWidth, m_sourceHeight);
	m_pImage->InitAlpha();
	// Requesting access to pixel data for copying the image to the canvas.
	wxImagePixelData data(*m_pImage);
    if ( !data ) {
        wxLogError(_T("Failed to gain raw access to bitmap data"));
		return E_ACCESSDENIED;
    }

	wxImagePixelData::Iterator p(data);
	const BYTE* s = pImage;

	for ( unsigned int y = 0, i = 0; y < m_sourceHeight; ++y ) {
		wxImagePixelData::Iterator rowStart = p;

        for ( unsigned int x = 0; x < m_sourceWidth; ++x, ++p, i += 4 )
        {
            p.Red() = s[i];
            p.Green() = s[i+1];
            p.Blue() = s[i+2];
			p.Alpha() = 255;
        }

        p = rowStart;
        p.OffsetY(data, 1);
    }

	m_pImage->Rescale(this->GetSize().GetWidth(), this->GetSize().GetHeight());

	wxBitmap pBitmap(*m_pImage, 32);

	// Stack object used to paint on screen.
	wxClientDC dc(this);
	wxBufferedDC bdc(&dc, *m_pScreenBitmap);
	bdc.SetBackground(*wxBLACK_BRUSH);
	bdc.Clear();
	bdc.DrawBitmap(pBitmap, 0, 0);

	// Request updating the displayed image.
	this->Update();

	delete m_pImage;
	return true;
}