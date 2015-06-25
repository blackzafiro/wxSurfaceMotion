#pragma once

#include <wx/log.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <wx/frame.h>
#include <wx/toolbar.h>
#include <wx/sizer.h>
#include <wx/listctrl.h>

namespace wxExtra {
	enum {
		ID_Clear
	};

	class wxLogConsole;

	class wxLogConsoleFrame : public wxFrame {
	private:
		typedef boost::recursive_mutex::scoped_lock scoped_lock;
		typedef boost::lock_guard<boost::recursive_mutex> scoped_guard;

		/// <summary>
		/// This is a shared resource, therefore thread safe access to this object is required.
		/// </summary>
		boost::recursive_mutex				m_mutexLogConsole;

		enum COLUMN_NUMBER {
			ICON_COLUMN = 0,
			FUNCTION_COLUMN,
			MESSAGE_COLUMN,
			FIRST_TIME_COLUMN,
			LAST_TIME_COLUMN,
			OCCURRENCES_COLUMN,
			FILE_COLUMN,
			LINE_COLUMN,
			THREAD_ID_COLUMN
		};

		void SetMessageList(void);

		void OnClose(wxCloseEvent& event);
		void OnClearTool(wxCommandEvent& event);

		wxDECLARE_EVENT_TABLE();

	public:
		wxLogConsoleFrame(wxLogConsole* owner);

		~wxLogConsoleFrame();

		/// <summary>
		/// Called to log a new record.
		/// </summary>
		virtual void DoLogRecord(wxLogLevel level, const wxString &msg, const wxLogRecordInfo &info);

	private:
		wxLogConsole* m_pOwner;

		wxToolBar*		m_pToolbar;
		wxBoxSizer*		m_pRootSizer;
		wxListCtrl*		m_pMessagesList;
	};

	class wxLogConsole : public wxLog {
	private:
		wxLogConsoleFrame*		m_pLogFrame;

	public:
		/// <summary>
		/// Constructor
		/// </summary>
		wxLogConsole();
		
		/// <summary>
		/// Called to log a new record.
		/// </summary>
		virtual void DoLogRecord(wxLogLevel level, const wxString &msg, const wxLogRecordInfo &info);

		friend class wxLogConsoleFrame;
	};

}