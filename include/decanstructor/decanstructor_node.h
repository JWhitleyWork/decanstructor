#ifndef DECANSTRUCTOR_NODE_H
#define DECANSTRUCTOR_NODE_H

#include <memory>
#include <cstdio>
#include <mutex>
#include <unordered_map>
#include <iomanip>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif
#include <wx/grid.h>

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <decanstructor/CanEvent.h>

namespace DeCANstructor
{
  wxDECLARE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);
  wxDECLARE_EVENT(wxEVT_CMD_EVENT_PUBLISHED, wxThreadEvent);

  enum
  {
    ID_BTN_UNCHECK_ALL = 1,
    ID_BTN_CHECK_ALL,
    ID_BTN_PUBLISH_EVENT
  };

  enum EventMode
  {
    REAL_TIME,
    PLAYBACK
  };

  struct CanMsgDetail
  {
    std::vector<uint8_t> bytes;
    std::vector<uint8_t> last_bytes;
    std::vector<uint64_t> last_updated_ms;
    uint64_t time_rcvd_ms = 0;
    uint64_t time_last_rcvd_ms = 0;
    unsigned int avg_rate = 0;
    bool hidden = false;
    int grid_index;
    int selector_index;
  };

  struct CellUpdate
  {
    int row;
    int col;
    uint64_t time_diff;
  };

  // TODO: Finish implementing custom grid table.
  class DCGridTable :
    public wxGridTableBase
  {
    int GetNumberRows();
    int GetNumberCols();
    wxString GetValue(int row, int col);
    void SetValue(int row, int col, const wxString &value);
    void Clear();
    bool InsertRows(size_t pos=0, size_t numRows=1);
    bool AppendRows(size_t numRows=1);
    bool DeleteRows(size_t pos=0, size_t numRows=1);
    bool InsertCols(size_t pos=0, size_t numCols=1);
    bool AppendCols(size_t numCols=1);
    bool DeleteCols(size_t pos=0, size_t numCols=1);
    void SetRowLabelValue(int row, const wxString &);
    void SetColLabelValue(int col, const wxString &);
    wxString GetRowLabelValue(int row);
    wxString GetColLabelValue(int col);
  };

  class DCOptions
  {
    public:
      static uint16_t fade_out_time_ms;
      static ros::Time one_day_ago;
      static EventMode event_mode;
  };

  class DCRenderTimer :
    public wxTimer
  {
    public:
      void Notify();
  };

  class DCFrame :
    public wxFrame
  {
    public:
      DCFrame(const wxString& title,
              const wxPoint& pos,
              const wxSize& size);

      std::shared_ptr<wxGrid> main_grid;
      std::shared_ptr<wxCheckListBox> selector_box;
      std::shared_ptr<DCRenderTimer> render_timer;
      std::shared_ptr<wxPanel> event_panel;
      std::shared_ptr<wxButton> pub_event_btn;
      std::shared_ptr<wxStaticText> pub_event_txt;

      std::mutex event_mut;
      bool in_playback_mode;
      bool got_new_event;
      uint64_t most_recent_event_time;

    private:
      std::unique_ptr<ros::AsyncSpinner> spinner;
      ros::Publisher event_pub;
      ros::Subscriber can_sub;
      ros::Subscriber event_sub;

      void OnCanMsg(const can_msgs::Frame::ConstPtr& msg);
      void OnExit(wxCommandEvent& event);
      void OnAbout(wxCommandEvent& event);
      void OnMsgsUpdate(wxThreadEvent& event);
      void OnSelectorBoxTick(wxCommandEvent& event);
      void OnUncheckAll(wxCommandEvent& event);
      void OnCheckAll(wxCommandEvent& event);
      void OnPublishEvent(wxCommandEvent& event);
      void OnEventPublished(const decanstructor::CanEvent::ConstPtr& msg);

      wxDECLARE_EVENT_TABLE();
  };

  class DCNode :
    public wxApp
  {
    public:
      virtual bool OnInit();
      DCFrame* frame;
      std::unordered_map<uint32_t, std::shared_ptr<CanMsgDetail>> rcvd_msgs;
      std::mutex rcvd_msgs_mut;
  };

	wxBEGIN_EVENT_TABLE(DCFrame, wxFrame)
		EVT_MENU(wxID_EXIT,  DCFrame::OnExit)
		EVT_MENU(wxID_ABOUT, DCFrame::OnAbout)
    EVT_CHECKLISTBOX(wxID_ANY, DCFrame::OnSelectorBoxTick)
    EVT_BUTTON(ID_BTN_UNCHECK_ALL, DCFrame::OnUncheckAll)
    EVT_BUTTON(ID_BTN_CHECK_ALL, DCFrame::OnCheckAll)
    EVT_BUTTON(ID_BTN_PUBLISH_EVENT, DCFrame::OnPublishEvent)
	wxEND_EVENT_TABLE()

  wxDEFINE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);
  wxDEFINE_EVENT(wxEVT_CMD_EVENT_PUBLISHED, wxThreadEvent);
}

wxIMPLEMENT_APP(DeCANstructor::DCNode);
wxDECLARE_APP(DeCANstructor::DCNode);

#endif
