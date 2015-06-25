//------------------------------------------------------------------------------
// <copyright file="KinectParams.h" company="UNAM">
//     Copyright (c) Universidad Nacional Autónoma de México.
// </copyright>
//------------------------------------------------------------------------------

#include "wxSliderFloatCombo.h"
#include <wx/event.h> 

namespace wxExtra {

	// Custom id events.
	enum
	{
		ID_SliderFloatCombo = 1000
	};

	wxBEGIN_EVENT_TABLE(wxSliderFloatCombo, wxControl)
		EVT_SLIDER(ID_SliderFloatCombo, wxSliderFloatCombo::OnSliderChange)
	wxEND_EVENT_TABLE()

	wxDEFINE_EVENT(EVT_SLIDER_FLOAT_COMBO, wxCommandEvent);

	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="parent">parent window</param>
	/// <param name="id">id of this control, it can be used to catch its events</param>
	/// <param name="minValue">slider minimum value</param>
	/// <param name="maxValue">slider maximum value</param>
	/// <param name="label">label text</param>
	wxSliderFloatCombo::wxSliderFloatCombo(wxWindow* parent, wxWindowID id, float value, float minValue, float maxValue, wxString label) :
		wxControl(parent, id),
		m_value(value),
		m_rangeMin(minValue),
		m_rangeMax(maxValue)
	{
		assert(m_rangeMin < m_rangeMax);

		wxBoxSizer* columnsSizer = new wxBoxSizer(wxHORIZONTAL);

		wxBoxSizer* row0Sizer = new wxBoxSizer(wxVERTICAL);
		row0Sizer->Add(new wxStaticText(this, wxID_ANY, label));
		columnsSizer->Add(row0Sizer, 0);
		columnsSizer->AddSpacer(8);

		wxBoxSizer* row1Sizer = new wxBoxSizer(wxVERTICAL);
		m_pTextMin = new wxStaticText(this, wxID_ANY, Format(m_rangeMin), wxDefaultPosition, wxSize(25,15));
		row1Sizer->Add(m_pTextMin, wxSizerFlags().Bottom());
		columnsSizer->Add(row1Sizer, 0);

		wxBoxSizer* row2Sizer = new wxBoxSizer(wxVERTICAL);
		m_pTextVal = new wxStaticText(this, wxID_ANY, Format(m_value), wxDefaultPosition, wxSize(25,15));
		row2Sizer->Add(m_pTextVal, wxSizerFlags().Center());
		m_pSlider = new wxSlider(this, ID_SliderFloatCombo, valueToInt(value), valueToInt(minValue), valueToInt(maxValue));
		row2Sizer->Add(m_pSlider, wxSizerFlags().Expand());
		columnsSizer->Add(row2Sizer, 1);

		wxBoxSizer* row3Sizer = new wxBoxSizer(wxVERTICAL);
		m_pTextMax = new wxStaticText(this, wxID_ANY, Format(m_rangeMax), wxDefaultPosition, wxSize(25,15));
		row3Sizer->Add(m_pTextMax, wxSizerFlags().Bottom());
		columnsSizer->Add(row3Sizer, 0);

		SetSizerAndFit(columnsSizer);
	}

	float wxSliderFloatCombo::GetValue() const {
		return m_value;
	}

	void  wxSliderFloatCombo::OnSliderChange(wxCommandEvent& event) {
		float value = intToFloatValue(event.GetSelection());
		assert(m_rangeMin <= value && value <= m_rangeMax);
		m_value = value;
		m_pTextVal->SetLabelText(Format(m_value));
		wxCommandEvent event_a(EVT_SLIDER_FLOAT_COMBO, this->GetId());
		wxPostEvent(GetParent(), event_a);
	}

}