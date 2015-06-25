//------------------------------------------------------------------------------
// <copyright file="SurfaceMotionGUI.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//
// Coded and tested with wxWdigets v3.0 and KinectSDK v1.8.
//
// Code from DepthBasics of (c) Microsoft has been incorporated to control access
// to the Kinect in the KinectManager files.
//
// The contents of this project are for general information and illustrative
// purposes only and without comercial interest.
// This software is supplied "as is" without any warranties or support.
//
// wxWidgets was installed as recommended in:
// https://github.com/T-Rex/wxKinectHelper
// However, this code does not use wxKinect Helper.
//------------------------------------------------------------------------------

#pragma once

// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wxprec.h>

#ifdef __BORLANDC__
#pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include <wx/spinctrl.h>
#include "kinect.xpm"
#include "wxRGBACanvas.h"
#include "wxExtra/wxSliderFloatCombo.h"
#include "wxExtra/wxLogConsole.h"
#include "KinectManager.h"
#include "Blackboard.h"

class MainFrame;

//-----------------------------------------------------------------------------
// MyApp
//----------------------------------------------------------------------------

class SurfaceMotionApp: public wxApp{

private:
	MainFrame*			frame;

public:

	/// <summary>
	/// Initialize the application.  Find and connect to a Kinect and show the main
	/// window.
	/// </summary>
    virtual bool OnInit() override; //wxOVERRIDE;

};

// ----------------------------------------------------------------------------
// MainFrame
// ----------------------------------------------------------------------------

class MainFrame: public wxFrame {

private:
	KinectManager*          m_pKinectManager;
	Blackboard				m_blackboard;

	// Timer to make update calls
	wxTimer*                m_pTimer;

	/// <summary>
	/// Requests the creation of a new thread and initialization of the Kinect.
	/// </summary>
    void OnConnectKinect(wxCommandEvent& event);

	/// <summary>
	/// Requests the end of the Kinect thread and disconnect the device.
	/// </summary>
    void OnDisconnectKinect(wxCommandEvent& event);

	/// <summary>
	/// Sets the resolution for data processing.
	/// </summary>
	void OnResolutionChoice(wxCommandEvent& event);

	/// <summary>
	/// Changes the minimum depth threshold.
	/// </summary>
	void OnMinThresholdChange(wxCommandEvent& event);

	/// <summary>
	/// Changes the maximum depth threshold.
	/// </summary>
	void OnMaxThresholdChange(wxCommandEvent& event);

	/// <summary>
	/// Sets the resolution for data reconstruction.
	/// </summary>
	void OnVoxelsPerMeterChoice(wxCommandEvent& event);

	/// <summary>
	/// Sets the number of voxels in the X axis for data reconstruction.
	/// </summary>
	void OnVoxelsInXChoice(wxCommandEvent& event);

	/// <summary>
	/// Sets the number of voxels in the Y axis for data reconstruction.
	/// </summary>
	void OnVoxelsInYChoice(wxCommandEvent& event);

	/// <summary>
	/// Sets the number of voxels in the Z axis for data reconstruction.
	/// </summary>
	void OnVoxelsInZChoice(wxCommandEvent& event);

	/// <summary>
	/// Sets the number of frames before volume reconstruction is reset.
	/// </summary>
	void OnFramesBeforeResetSpin(wxSpinEvent& event);

	/// <summary>
	/// Close the application on exit.
	/// </summary>
    void OnExit(wxCommandEvent& event);

	/// <summary>
	/// What this app is.
	/// </summary>
    void OnAbout(wxCommandEvent& event);

	/// <summary>
	/// Check if there is new frame data and display it on the screen.
	/// </summary>
	/// <param name="event">unused timer event object</param>
	void OnTimerEvent(wxTimerEvent &event);

    wxDECLARE_EVENT_TABLE();

public:
	/// <summary>
    /// Constructor
    /// </summary>
	/// <param name="title">screen title</param>
	/// <param name="pos">position of the frame within the screen</param>
	/// <param name="size">frame size</param>
    MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size);

	/// <summary>
    /// Destructor
    /// </summary>
	~MainFrame();

private:
	SurfaceMotionApp*				m_pMainApp;

	wxMenu*							m_pMenuKinect;
	wxCheckBox*						m_pCheckCaptureColor;
	wxCheckBox*						m_pCheckNearMode;
	wxChoice*						m_pChoiceNuiImageResolution;
	wxExtra::wxSliderFloatCombo*	m_pSliderMinDepthThreshold;
	wxExtra::wxSliderFloatCombo*	m_pSliderMaxDepthThreshold;

	wxChoice*						m_pChoiceVoxelsPerMeter;
	wxChoice*						m_pChoiceVoxelCountX;
	wxChoice*						m_pChoiceVoxelCountY;
	wxChoice*						m_pChoiceVoxelCountZ;
	wxSpinCtrl*						m_pSpinFramesBeforeReset;

	wxRGBACanvas*					m_pDepthCanvas;
	wxRGBACanvas*					m_pColorCanvas;
	wxRGBACanvas*					m_pColorReconstructionCanvas;
};

// Custom id events.
enum
{
    ID_ConnectKinect = 1,
	ID_DisconnectKinect,
	ID_ImageResolution,
	ID_MinDepthThreshold,
	ID_MaxDepthThreshold,

	ID_VoxelsPerMeter,
	ID_CheckVoxelCountX,
	ID_CheckVoxelCountY,
	ID_CheckVoxelCountZ,
	ID_SpinFramesBeforeReset
};

wxIMPLEMENT_APP(SurfaceMotionApp);