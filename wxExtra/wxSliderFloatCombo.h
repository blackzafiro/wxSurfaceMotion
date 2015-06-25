//------------------------------------------------------------------------------
// <copyright file="KinectParams.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#include <wx/slider.h>
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/log.h>

namespace wxExtra {

	/// Use EVT_SLIDER_FLOAT_COMBO to be notified when the slider changed value.
	wxDECLARE_EVENT(EVT_SLIDER_FLOAT_COMBO, wxCommandEvent);

	/// <summary>
	/// Constructor
	/// </summary>
	class wxSliderFloatCombo : public wxControl {
	private:
		float m_rangeMin;
		float m_rangeMax;
		float m_value;

		// format an integer value as string
		static wxString Format(float n) { return wxString::Format(wxT("%.2f"), n); }
		static unsigned int valueToInt(float value) { return (unsigned int)(value * 1000); }
		static float intToFloatValue(unsigned int i) { return ((float)i) / 1000; }

	
		void OnSliderChange(wxCommandEvent& event);

		wxDECLARE_EVENT_TABLE();

	public:

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="parent">parent window</param>
		/// <param name="id">id of this control, it can be used to catch its events</param>
		/// <param name="minValue">slider minimum value</param>
		/// <param name="maxValue">slider maximum value</param>
		/// <param name="label">label text</param>
		wxSliderFloatCombo(wxWindow* parent, wxWindowID id, float value, float minValue, float maxValue, wxString label);

		virtual float GetValue() const;

	private:
		wxSlider*			m_pSlider;
		wxStaticText*		m_pTextMin;
		wxStaticText*		m_pTextMax;
		wxStaticText*		m_pTextVal;
	};

}