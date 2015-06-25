#include "wxLogConsole.h"
#include <wx/artprov.h>
#include <wx/imaglist.h>
#include "Reset_32x32.xpm"

namespace wxExtra {

	// Taken from the Artprov wxWidgets demo
	#define ART_ICON(id) \
		{ \
			int ind; \
			wxIcon icon = wxArtProvider::GetIcon(id, client, size); \
			if ( icon.IsOk() ) \
				ind = images->Add(icon); \
			else \
				ind = 0; \
			index++; \
		}

	wxBEGIN_EVENT_TABLE(wxLogConsoleFrame, wxFrame)
		EVT_TOOL(ID_Clear, wxLogConsoleFrame::OnClearTool)
		EVT_CLOSE(wxLogConsoleFrame::OnClose)
	wxEND_EVENT_TABLE()

	wxLogConsoleFrame::wxLogConsoleFrame(wxLogConsole* owner) :
		m_pOwner(owner),
		m_pMessagesList(NULL),
		wxFrame(NULL, wxID_ANY, L"Log Console")
	{
		scoped_guard paramsGuard(m_mutexLogConsole);
		
		//wxImage::AddHandler( new wxPNGHandler );
		wxBitmap clear(Reset_32x32_xpm, wxBITMAP_TYPE_XPM);
		m_pToolbar = CreateToolBar();
		m_pToolbar->AddTool(ID_Clear, "Reset", clear);
		m_pToolbar->Realize();
		
		wxBoxSizer *root = new wxBoxSizer( wxVERTICAL );
		this->SetMessageList();
		root->Add(m_pMessagesList);
		m_pRootSizer = root;
		Show();
	}

	wxLogConsoleFrame::~wxLogConsoleFrame() {
		scoped_guard paramsGuard(m_mutexLogConsole);

		if (NULL != m_pMessagesList) {
			delete m_pMessagesList;
			m_pMessagesList = NULL;
		}
	}

	void wxLogConsoleFrame::OnClose(wxCloseEvent& event) {
		scoped_guard paramsGuard(m_mutexLogConsole);

		m_pOwner->m_pLogFrame = NULL;
		wxFrame::OnCloseWindow(event);
	}

	void wxLogConsoleFrame::OnClearTool(wxCommandEvent& event) {
		scoped_guard paramsGuard(m_mutexLogConsole);

		if (NULL != m_pMessagesList) {
			m_pMessagesList->DeleteAllItems();
		}
	}

	void wxLogConsoleFrame::SetMessageList(void) {
		m_pMessagesList = new wxListCtrl(this, wxID_ANY);
		m_pMessagesList->SetSingleStyle(wxLC_REPORT);
		m_pMessagesList->AppendColumn("Level", wxLIST_FORMAT_CENTRE);
		m_pMessagesList->AppendColumn("Function", wxLIST_FORMAT_LEFT);
		m_pMessagesList->AppendColumn("Message", wxLIST_FORMAT_LEFT);
		m_pMessagesList->AppendColumn("First time", wxLIST_FORMAT_CENTRE);
		m_pMessagesList->AppendColumn("Last time", wxLIST_FORMAT_CENTRE);
		m_pMessagesList->AppendColumn("Occurrences", wxLIST_FORMAT_CENTRE);
		m_pMessagesList->AppendColumn("File", wxLIST_FORMAT_LEFT);
		m_pMessagesList->AppendColumn("Line", wxLIST_FORMAT_CENTRE);
		if(wxUSE_THREADS == 1) m_pMessagesList->AppendColumn("Thread", wxLIST_FORMAT_CENTRE);

		wxImageList *images = new wxImageList(16, 16);
		wxListCtrl *list = m_pMessagesList;
		int index = 0;
		const wxArtClient& client = wxART_MESSAGE_BOX;
		const wxSize& size = wxSize(16, 16);

		ART_ICON(wxART_ERROR)
		ART_ICON(wxART_QUESTION)
		ART_ICON(wxART_WARNING)
		ART_ICON(wxART_INFORMATION)

		m_pMessagesList->AssignImageList(images, wxIMAGE_LIST_SMALL);
	}

	/// <summary>
	/// Called to log a new record.
	/// </summary>
	void wxLogConsoleFrame::DoLogRecord(wxLogLevel level, const wxString &msg, const wxLogRecordInfo &info) {
		scoped_guard paramsGuard(m_mutexLogConsole);

		int lastItem = m_pMessagesList->GetItemCount() - 1;
		if(lastItem >= 0) {
			wxString lastMessage = m_pMessagesList->GetItemText(lastItem, MESSAGE_COLUMN);
			if(lastMessage.IsSameAs(msg)) {
				long times;
				bool isNumber = m_pMessagesList->GetItemText(lastItem, OCCURRENCES_COLUMN).ToLong(&times);
				if(!isNumber) times = 0;
				times++;
				wxString stimes;
				stimes << times;
				m_pMessagesList->SetItem(lastItem, OCCURRENCES_COLUMN, stimes);
				m_pMessagesList->SetItem(lastItem, LAST_TIME_COLUMN, wxDateTime(info.timestamp).Format(m_pOwner->GetTimestamp()));
				return;
			}
		}

		int newItemIndex = m_pMessagesList->GetItemCount();
		if (-1 == newItemIndex) newItemIndex = 0;
		int iconIndex = 3;
		switch (level) {
		case wxLOG_FatalError:
		case wxLOG_Error:
			m_pMessagesList->InsertItem(newItemIndex, 0);
			break;
		case wxLOG_Warning:
			m_pMessagesList->InsertItem(newItemIndex, 2);
			break;
		case wxLOG_Message:
		case wxLOG_Status:
		case wxLOG_Info:
			m_pMessagesList->InsertItem(newItemIndex, 3);
			break;
		case wxLOG_Debug:
		case wxLOG_Trace:
		case wxLOG_Progress:
		case wxLOG_User:
		case wxLOG_Max:
		default:
			m_pMessagesList->InsertItem(newItemIndex, -1);
			break;
		}
	
		m_pMessagesList->SetItem(newItemIndex, MESSAGE_COLUMN, msg);
		m_pMessagesList->SetColumnWidth(MESSAGE_COLUMN, wxLIST_AUTOSIZE);

		m_pMessagesList->SetItem(newItemIndex, FIRST_TIME_COLUMN, wxDateTime(info.timestamp).Format(m_pOwner->GetTimestamp()));
		m_pMessagesList->SetItem(newItemIndex, FILE_COLUMN, info.filename);
		m_pMessagesList->SetColumnWidth(FILE_COLUMN, wxLIST_AUTOSIZE);

		m_pMessagesList->SetItem(newItemIndex, FUNCTION_COLUMN, info.func);
		m_pMessagesList->SetColumnWidth(FUNCTION_COLUMN, wxLIST_AUTOSIZE);

		wxString intToString; intToString << info.line;
		m_pMessagesList->SetItem(newItemIndex, LINE_COLUMN, intToString);
		if(wxUSE_THREADS == 1) {
			intToString.Clear();
			intToString << info.threadId;
			m_pMessagesList->SetItem(newItemIndex, THREAD_ID_COLUMN, intToString);
		}

		m_pRootSizer->Fit(this);
	}




	/// <summary>
	/// Constructor
	/// </summary>
	wxLogConsole::wxLogConsole() :
		m_pLogFrame(NULL)
	{
	}

	/// <summary>
	/// Called to log a new record.
	/// </summary>
	void wxLogConsole::DoLogRecord(wxLogLevel level, const wxString &msg, const wxLogRecordInfo &info) {
		if(NULL == m_pLogFrame) {
			m_pLogFrame = new wxLogConsoleFrame(this);
			m_pLogFrame->Show();
		}
		m_pLogFrame->DoLogRecord(level, msg, info);
	}

}