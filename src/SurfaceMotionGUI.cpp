//------------------------------------------------------------------------------
// <copyright file="SurfaceMotionGUI.cpp" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//-----------------------------------------------------------------------------

#include "SurfaceMotionGUI.h"

wxBEGIN_EVENT_TABLE(MainFrame, wxFrame)
	EVT_SPINCTRL(ID_SpinFramesBeforeReset, MainFrame::OnFramesBeforeResetSpin)
	EVT_CHOICE(ID_CheckVoxelCountZ, MainFrame::OnVoxelsInZChoice)
	EVT_CHOICE(ID_CheckVoxelCountY, MainFrame::OnVoxelsInYChoice)
	EVT_CHOICE(ID_CheckVoxelCountX, MainFrame::OnVoxelsInXChoice)
	EVT_CHOICE(ID_VoxelsPerMeter, MainFrame::OnVoxelsPerMeterChoice)
	EVT_COMMAND(ID_MinDepthThreshold, wxExtra::EVT_SLIDER_FLOAT_COMBO, MainFrame::OnMinThresholdChange)
	EVT_COMMAND(ID_MaxDepthThreshold, wxExtra::EVT_SLIDER_FLOAT_COMBO, MainFrame::OnMaxThresholdChange)
	EVT_CHOICE(ID_ImageResolution, MainFrame::OnResolutionChoice)
    EVT_MENU(ID_ConnectKinect,   MainFrame::OnConnectKinect)
	EVT_MENU(ID_DisconnectKinect,   MainFrame::OnDisconnectKinect)
    EVT_MENU(wxID_EXIT,  MainFrame::OnExit)
    EVT_MENU(wxID_ABOUT, MainFrame::OnAbout)
	EVT_TIMER(-1,        MainFrame::OnTimerEvent)
wxEND_EVENT_TABLE()

/// <summary>
/// Initialize the GUI.
/// </summary>
bool SurfaceMotionApp::OnInit() {
	if ( !wxApp::OnInit() )
        return false;

	// Logging
	wxExtra::wxLogConsole* pLogConsole = new wxExtra::wxLogConsole();
	wxLog* old = wxLog::SetActiveTarget(pLogConsole);
	if (NULL != old) delete old;

	// Create main window
	frame = new MainFrame("Surface motion tracking with Kinect", wxPoint(50, 50), wxSize(480, 350));
    frame->Show( true );

    return true;
}


// ----------------------------------------------------------------------------
// MainFrame
// ----------------------------------------------------------------------------

/// <summary>
/// Constructor
/// </summary>
/// <param name="kinectManager">main frame showing the options for the surface reconstruction and displaying the results.</param>
/// <param name="title">screen title</param>
/// <param name="pos">position of the frame within the screen</param>
/// <param name="myApp">App to which this frame belongs</param>
MainFrame::MainFrame(const wxString& title, const wxPoint& pos, const wxSize& size) :
	m_pKinectManager(NULL),
	m_pTimer(NULL),
	wxFrame(NULL, wxID_ANY, title, pos, size)
{
	SetIcon(wxIcon(kinect_xpm));

	///
	/// Kinect
	///

	// create kinect manager
	m_pKinectManager = new KinectManager(m_blackboard);

	// create menu bar
	m_pMenuKinect = new wxMenu;
    m_pMenuKinect->Append(ID_ConnectKinect, "&Connect Kinect...\tCtrl-K",
                     "Connects to an available Kinect.");
	m_pMenuKinect->Append(ID_DisconnectKinect, "&Disconnect Kinect...\tCtrl-D",
                     "Disconnects the Kinect.");
	m_pMenuKinect->Enable(ID_DisconnectKinect, false);
    m_pMenuKinect->AppendSeparator();
    m_pMenuKinect->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append( m_pMenuKinect, "&Kinect" );
    menuBar->Append( menuHelp, "&Help" );

    SetMenuBar( menuBar );

	// create the boxes
    wxBoxSizer *root = new wxBoxSizer( wxHORIZONTAL );

	///
	/// Kinect parameters
	///
	KinectParams& rKinectParams = m_blackboard.GetNextKinectParams();

	// create controls on a left box
	wxStaticBoxSizer *leftSizer = new wxStaticBoxSizer( wxVERTICAL, this, wxT("Controls") );


	//
	// Frames capture
	//
	wxStaticBoxSizer *framesControlsSizer = new wxStaticBoxSizer( wxVERTICAL, this,
                wxT("Frames capture"));
	
	// capture color
	m_pCheckCaptureColor = new wxCheckBox(this, wxID_ANY, wxT("Capture color") );
	m_pCheckCaptureColor->SetValue(true);
	m_pCheckCaptureColor->Enable(false);
	framesControlsSizer->Add(m_pCheckCaptureColor);

	// near mode
	m_pCheckNearMode = new wxCheckBox(this, wxID_ANY, wxT("Near mode") );
	m_pCheckNearMode->SetValue(true);
	m_pCheckNearMode->Enable(false);
	framesControlsSizer->Add(m_pCheckNearMode);

	// nui image resolution
	wxArrayString resolutions;
	resolutions.Add("NUI_IMAGE_RESOLUTION_80x60"); // Only for depth not for color. // Luckily this is cero, just as in the enum.
	resolutions.Add("NUI_IMAGE_RESOLUTION_320x240"); // Only for depth not for color.
	resolutions.Add("NUI_IMAGE_RESOLUTION_640x480");
	// resolutions.Add("NUI_IMAGE_RESOLUTION_1280x960"); // Only valid for color, not for depth http://msdn.microsoft.com/en-us/library/jj663864.aspx
	m_pChoiceNuiImageResolution = new wxChoice(this, ID_ImageResolution, wxDefaultPosition, wxDefaultSize, resolutions);
	m_pChoiceNuiImageResolution->SetSelection(rKinectParams.GetImageResolution());
	m_pChoiceNuiImageResolution->Enable(false);
	framesControlsSizer->Add(m_pChoiceNuiImageResolution, wxSizerFlags().Expand().Border(wxTOP | wxBOTTOM, 10));

	// Depth thresholds
	framesControlsSizer->Add(new wxStaticText(this, wxID_ANY, "Depth threshold"), wxSizerFlags().Expand());
	// minimum
	m_pSliderMinDepthThreshold = new wxExtra::wxSliderFloatCombo(this, ID_MinDepthThreshold,
		rKinectParams.GetMinDepthThreshold(), KinectParams::MinimumMinDepthThreshold, KinectParams::MaximumMinDepthThreshold, "Min");
	framesControlsSizer->Add(m_pSliderMinDepthThreshold, wxSizerFlags().Expand());
	// maximum
	m_pSliderMaxDepthThreshold = new wxExtra::wxSliderFloatCombo(this, ID_MaxDepthThreshold,
		rKinectParams.GetMaxDepthThreshold(), KinectParams::MinimumMaxDepthThreshold, KinectParams::MaximumMaxDepthThreshold, "Max");
	framesControlsSizer->Add(m_pSliderMaxDepthThreshold, wxSizerFlags().Expand());

	leftSizer->Add(framesControlsSizer, wxSizerFlags().Expand().Border());

	root->Add( leftSizer, wxSizerFlags().Expand().Border());


	///
	/// Images display
	///

	// Display canvas
	int width = rKinectParams.GetWidth();
	int height = rKinectParams.GetHeight();

	/// Create canvas to show reconstructions

	wxBoxSizer *reconstructionDataCanvasSizer = new wxBoxSizer( wxVERTICAL);

	// Color reconstruction canvas
	m_pColorReconstructionCanvas = new wxRGBACanvas( this, wxID_ANY, wxDefaultPosition, wxSize(width, height), wxSize(width, height));
	reconstructionDataCanvasSizer->Add(m_pColorReconstructionCanvas);

	root->Add(reconstructionDataCanvasSizer, wxSizerFlags().Expand().Border());

	/// Create canvas to show captured depth and color.

	const int WIDTH = 480;
	const int HEIGHT = 360;
	
	wxBoxSizer *dataCanvasSizer = new wxBoxSizer( wxVERTICAL);

	// Depth canvas
	m_pDepthCanvas = new wxRGBACanvas( this, wxID_ANY, wxDefaultPosition, wxSize(WIDTH, HEIGHT), wxSize(width, height));
	dataCanvasSizer->Add(m_pDepthCanvas);

	// Color canvas
	m_pColorCanvas= new wxRGBACanvas( this, wxID_ANY, wxDefaultPosition, wxSize(WIDTH, HEIGHT), wxSize(width, height));
	dataCanvasSizer->Add(m_pColorCanvas);

	root->Add(dataCanvasSizer, wxSizerFlags().Expand().Border());


	//
	// Mesh reconstruction
	// Nui parameters
	//
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();

	wxStaticBoxSizer *meshControlsSizer = new wxStaticBoxSizer( wxVERTICAL, this,
                wxT("Mesh reconstruction"));
	leftSizer->Add(meshControlsSizer, wxSizerFlags().Expand().Border());
	wxStaticText* pResolutionNotice = new wxStaticText(this, wxID_ANY, "Before increasing the voxels per meter reduce voxels in xyz.\nOtherwise execution may fail or be slow.");
	meshControlsSizer->Add(pResolutionNotice);
	meshControlsSizer->AddSpacer(4);

	// Voxels per meter
	wxBoxSizer *voxelsRow = new wxBoxSizer( wxHORIZONTAL );
	voxelsRow->Add(new wxStaticText(this, wxID_ANY, wxT("Voxels per meter")), 1);
	voxelsRow->AddSpacer(4);
	wxArrayString voxelsPerMeterOptions;
	for(int i = 0; i < NuiParams::VOXELS_PER_METER_OPTIONS_LENGTH; ++i) {
		voxelsPerMeterOptions.Add(NuiParams::voxelsPerMeterOptions[i].string);
	}
	m_pChoiceVoxelsPerMeter = new wxChoice(this, ID_VoxelsPerMeter, wxDefaultPosition, wxDefaultSize, voxelsPerMeterOptions);
	m_pChoiceVoxelsPerMeter->SetSelection(NSOvalueToIndex(NuiParams::voxelsPerMeterOptions, NuiParams::VOXELS_PER_METER_OPTIONS_LENGTH, rNuiParams.GetVoxelsPerMeter()));
	voxelsRow->Add(m_pChoiceVoxelsPerMeter, 0);
	meshControlsSizer->Add(voxelsRow, wxSizerFlags().Expand());

	// Voxels in X
	voxelsRow = new wxBoxSizer( wxHORIZONTAL );
	voxelsRow->Add(new wxStaticText(this, wxID_ANY, wxT("Voxels in X")), 1);
	voxelsRow->AddSpacer(4);
	wxArrayString voxelCount;
	for(int i = 0; i < NuiParams::VOXELS_COUNT_OPTIONS_LENGTH; ++i) {
		voxelCount.Add(NuiParams::voxelsCountOptions[i].string);
	}
	m_pChoiceVoxelCountX = new wxChoice(this, ID_CheckVoxelCountX, wxDefaultPosition, wxDefaultSize, voxelCount);
	m_pChoiceVoxelCountX->SetSelection(NSOvalueToIndex(NuiParams::voxelsCountOptions, NuiParams::VOXELS_COUNT_OPTIONS_LENGTH, rNuiParams.GetVoxelCountX()));
	voxelsRow->Add(m_pChoiceVoxelCountX, 0);
	meshControlsSizer->Add(voxelsRow, wxSizerFlags().Expand());

	// Voxels in Y
	voxelsRow = new wxBoxSizer( wxHORIZONTAL );
	voxelsRow->Add(new wxStaticText(this, wxID_ANY, wxT("Voxels in Y")), 1);
	voxelsRow->AddSpacer(4);
	m_pChoiceVoxelCountY = new wxChoice(this, ID_CheckVoxelCountY, wxDefaultPosition, wxDefaultSize, voxelCount);
	m_pChoiceVoxelCountY->SetSelection(NSOvalueToIndex(NuiParams::voxelsCountOptions, NuiParams::VOXELS_COUNT_OPTIONS_LENGTH, rNuiParams.GetVoxelCountY()));
	voxelsRow->Add(m_pChoiceVoxelCountY, 0);
	meshControlsSizer->Add(voxelsRow, wxSizerFlags().Expand());

	// Voxels in Z
	voxelsRow = new wxBoxSizer( wxHORIZONTAL );
	voxelsRow->Add(new wxStaticText(this, wxID_ANY, wxT("Voxels in Z")), 1);
	voxelsRow->AddSpacer(4);
	m_pChoiceVoxelCountZ = new wxChoice(this, ID_CheckVoxelCountZ, wxDefaultPosition, wxDefaultSize, voxelCount);
	m_pChoiceVoxelCountZ->SetSelection(NSOvalueToIndex(NuiParams::voxelsCountOptions, NuiParams::VOXELS_COUNT_OPTIONS_LENGTH, rNuiParams.GetVoxelCountZ()));
	voxelsRow->Add(m_pChoiceVoxelCountZ, 0);
	meshControlsSizer->Add(voxelsRow, wxSizerFlags().Expand());

	// Frames before reset
	voxelsRow = new wxBoxSizer( wxHORIZONTAL );
	voxelsRow->Add(new wxStaticText(this, wxID_ANY, wxT("Integrated frames")), 1);
	voxelsRow->AddSpacer(4);
	m_pSpinFramesBeforeReset = new wxSpinCtrl(this, ID_SpinFramesBeforeReset);
	m_pSpinFramesBeforeReset->SetRange(0, INT_MAX);
	m_pSpinFramesBeforeReset->SetValue(rNuiParams.GetFramesBeforeReset());
	voxelsRow->Add(m_pSpinFramesBeforeReset, 0);
	meshControlsSizer->Add(voxelsRow, wxSizerFlags().Expand());

	// Set sizer for window
    SetSizerAndFit( root );

	// Create timer to check for Kinect updates
	m_pTimer = new wxTimer(this);
	m_pTimer->Start();
}

/// <summary>
/// Destructor
/// </summary>
MainFrame::~MainFrame(){
	// clean up manager
	if (m_pTimer) m_pTimer->Stop();
	if ( m_pKinectManager != NULL ) {
		m_pKinectManager->RequestDisconnectKinectThread();
		m_pKinectManager->join();
        delete m_pKinectManager;
        m_pKinectManager = NULL;
    }
}

/// <summary>
/// Requests the creation of a new thread and initialization of the Kinect.
/// </summary>
void MainFrame::OnConnectKinect(wxCommandEvent &WXUNUSED(event)) {
	HRESULT hr = m_pKinectManager->LaunchKinectThread();
	if (S_OK == hr){
		m_pMenuKinect->Enable(ID_ConnectKinect, false);
		m_pMenuKinect->Enable(ID_DisconnectKinect, true);
		//m_pChoiceNuiImageResolution->Enable(false);
		wxMessageBox("Got a Kinect!", "Notice");
	} else {
		wxLogMessage("Connection to Kinect failed.");
	}
}

/// <summary>
/// Requests the end of the Kinect thread and disconnect the device.
/// </summary>
void MainFrame::OnDisconnectKinect(wxCommandEvent& event){
	HRESULT hr = m_pKinectManager->RequestDisconnectKinectThread();
	if (S_OK == hr){
		m_pMenuKinect->Enable(ID_ConnectKinect, true);
		m_pMenuKinect->Enable(ID_DisconnectKinect, false);
		//m_pChoiceNuiImageResolution->Enable(true);
		wxMessageBox("Kinect disconnected", "Notice");
	} else {
		wxLogMessage("Request to disconnect Kinect failed... current status %d.", m_blackboard.GetProcessingStatus().GetKinectConnected());
	}
}

/// <summary>
/// Changes the minimum depth threshold.
/// </summary>
void MainFrame::OnMinThresholdChange(wxCommandEvent &WXUNUSED(event)) {
	m_blackboard.GetNextKinectParams().SetMinDepthThreshold(m_pSliderMinDepthThreshold->GetValue());
}

/// <summary>
/// Changes the maximum depth threshold.
/// </summary>
void MainFrame::OnMaxThresholdChange(wxCommandEvent &WXUNUSED(event)) {
	m_blackboard.GetNextKinectParams().SetMaxDepthThreshold(m_pSliderMaxDepthThreshold->GetValue());
}

/// <summary>
/// Sets the resolution for data processing.
/// </summary>
void MainFrame::OnResolutionChoice(wxCommandEvent &WXUNUSED(event)){
	KinectParams& rKinectParams = m_blackboard.GetNextKinectParams();
	rKinectParams.SetImageResolution((NUI_IMAGE_RESOLUTION)m_pChoiceNuiImageResolution->GetSelection());

	//wxLogMessage("Image resolution set to %d", m_pChoiceNuiImageResolution->GetSelection());
}

/// <summary>
/// Sets the resolution for data reconstruction.
/// </summary>
void MainFrame::OnVoxelsPerMeterChoice(wxCommandEvent &WXUNUSED(event)) {
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();
	rNuiParams.SetVoxelsPerMeter(NuiParams::voxelsPerMeterOptions[m_pChoiceVoxelsPerMeter->GetSelection()].value);

	wxLogMessage("Changing voxels per meter to selection %d, value %d, it is now %f", m_pChoiceVoxelsPerMeter->GetSelection(), NuiParams::voxelsPerMeterOptions[m_pChoiceVoxelsPerMeter->GetSelection()].value, rNuiParams.GetVoxelsPerMeter());
}

/// <summary>
/// Sets the number of voxels in the X axis for data reconstruction.
/// </summary>
void MainFrame::OnVoxelsInXChoice(wxCommandEvent& event) {
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();
	rNuiParams.SetVoxelCountX(NuiParams::voxelsCountOptions[m_pChoiceVoxelCountX->GetSelection()].value);

	//wxLogMessage("Changing voxels per meter to selection %d, value %d, it is now %f", m_pCheckVoxelCountX->GetCurrentSelection(), NuiParams::voxelsCountOptions[m_pCheckVoxelCountX->GetSelection()].value, rNuiParams.GetVoxelCountX());
}

/// <summary>
/// Sets the number of voxels in the Y axis for data reconstruction.
/// </summary>
void MainFrame::OnVoxelsInYChoice(wxCommandEvent& event) {
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();
	rNuiParams.SetVoxelCountY(NuiParams::voxelsCountOptions[m_pChoiceVoxelCountY->GetSelection()].value);
}

/// <summary>
/// Sets the number of voxels in the Z axis for data reconstruction.
/// </summary>
void MainFrame::OnVoxelsInZChoice(wxCommandEvent& event) {
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();
	rNuiParams.SetVoxelCountZ(NuiParams::voxelsCountOptions[m_pChoiceVoxelCountZ->GetSelection()].value);
}

/// <summary>
/// Sets the number of frames before volume reconstruction is reset.
/// </summary>
void MainFrame::OnFramesBeforeResetSpin(wxSpinEvent& event) {
	NuiParams& rNuiParams = m_blackboard.GetNextNuiParams();
	rNuiParams.SetFramesBeforeReset(m_pSpinFramesBeforeReset->GetValue());
}

/// <summary>
/// Check if there is new frame data and display it on the screen.
/// </summary>
/// <param name="event">unused timer event object</param>
void MainFrame::OnTimerEvent(wxTimerEvent &event) {
	SurfaceReconstruction& surface = m_blackboard.GetSurfaceReconstruction();

	if(surface.GetFrameReady()) {
		//wxLogMessage("[SurfaceMotionGUI] Found frame ready");
		surface.lockBuffersFrame();
		m_pDepthCanvas->DrawBGRX(surface.GetDepthBuffer(), surface.GetImageBytesSize());
		m_pColorCanvas->DrawBGR(surface.GetColorBuffer(), surface.GetImageBytesSize(), true);
		surface.SetFrameReady(false);
		surface.unlockBuffersFrame();
	}

	if(surface.GetReconstructionFrameReady()) {
		surface.lockBuffersFrame();
		m_pColorReconstructionCanvas->DrawBGR(surface.GetColorReconstructionBuffer(), surface.GetImageBytesSize());
		surface.SetReconstructionFrameReady(false);
		surface.unlockBuffersFrame();
	}

	// Update Kinect status
	if(m_blackboard.GetProcessingStatus().GetKinectConnected() == ProcessingStatus::DISCONNECTED) {
		m_pMenuKinect->Enable(ID_ConnectKinect, true);
		m_pMenuKinect->Enable(ID_DisconnectKinect, false);
	}
}

/// <summary>
/// Close the application on exit.
/// </summary>
void MainFrame::OnExit(wxCommandEvent &WXUNUSED(event)) {
    Close( true );
}

/// <summary>
/// What this app is.
/// </summary>
void MainFrame::OnAbout(wxCommandEvent &WXUNUSED(event)) {
    wxMessageBox( "Pseudo tracking of 3D motion with\nframe by frame surface reconstruction",
                  "About Surface Motion", wxOK | wxICON_INFORMATION );
}