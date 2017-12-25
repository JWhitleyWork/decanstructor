#ifndef DECANSTRUCTOR_NODE_H
#define DECANSTRUCTOR_NODE_H

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif
#include <ros/ros.h>

namespace DeCANstructor
{
  enum
  {
    ID_Hello = 1
  };

  class DCNode :
    public wxApp
  {
    public:
      virtual bool OnInit();
  };

  class DCFrame :
    public wxFrame
  {
    public:
      DCFrame(const wxString& title,
              const wxPoint& pos,
              const wxSize& size);

    private:
      void OnHello(wxCommandEvent& event);
      void OnExit(wxCommandEvent& event);
      void OnAbout(wxCommandEvent& event);

      wxDECLARE_EVENT_TABLE();
  };

	wxBEGIN_EVENT_TABLE(DCFrame, wxFrame)
		EVT_MENU(ID_Hello,   DCFrame::OnHello)
		EVT_MENU(wxID_EXIT,  DCFrame::OnExit)
		EVT_MENU(wxID_ABOUT, DCFrame::OnAbout)
	wxEND_EVENT_TABLE()
}

wxIMPLEMENT_APP(DeCANstructor::DCNode);

#endif
