#ifndef DECANSTRUCTOR_NODE_H
#define DECANSTRUCTOR_NODE_H

#include <memory>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include <ros/ros.h>
#include <can_msgs/Frame.h>

namespace DeCANstructor
{
  enum
  {
    ID_Hello = 1
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

  class DCNode :
    public wxApp
  {
    public:
      virtual bool OnInit();
      void CanCallback(const::can_msgs::Frame::ConstPtr& msg);

    private:
      std::unique_ptr<DCFrame> frame;

      std::unique_ptr<ros::NodeHandle> node_handle;
      std::unique_ptr<ros::NodeHandle> private_handle;
      std::unique_ptr<ros::AsyncSpinner> spinner;
      ros::Subscriber can_sub;
  };

	wxBEGIN_EVENT_TABLE(DCFrame, wxFrame)
		EVT_MENU(ID_Hello,   DCFrame::OnHello)
		EVT_MENU(wxID_EXIT,  DCFrame::OnExit)
		EVT_MENU(wxID_ABOUT, DCFrame::OnAbout)
	wxEND_EVENT_TABLE()
}

wxIMPLEMENT_APP(DeCANstructor::DCNode);
wxDECLARE_APP(DeCANstructor::DCNode);

#endif
