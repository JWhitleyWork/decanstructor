#ifndef DECANSTRUCTOR_NODE_H
#define DECANSTRUCTOR_NODE_H

#include <common.h>
#include <message_analyzer.h>

#include <can_msgs/Frame.h>
#include <decanstructor/CanEvent.h>

namespace DeCANstructor
{
  wxDECLARE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);

  enum
  {
    ID_BTN_MESSAGE_ANALYZER = 1,
    ID_BTN_UNCHECK_ALL,
    ID_BTN_CHECK_ALL,
    ID_BTN_PUBLISH_EVENT
  };

  enum EventMode
  {
    REAL_TIME,
    PLAYBACK
  };

  enum SortDirection
  {
    NONE,
    ASCENDING,
    DESCENDING
  };

  struct CanMsgDetail
  {
    std::vector<uint8_t> bytes;
    std::vector<uint8_t> last_bytes;
    std::vector<uint64_t> last_updated_ms;
    uint64_t time_rcvd_ms = 0;
    uint64_t time_last_rcvd_ms = 0;
    unsigned int avg_rate = 0;
    int grid_index = -1;
    bool hidden = false;
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

      std::unique_ptr<wxGrid> main_grid;
      std::unique_ptr<wxButton> message_analyzer_btn;
      std::unique_ptr<wxCheckListBox> selector_box;
      std::unique_ptr<DCRenderTimer> render_timer;
      std::unique_ptr<wxPanel> event_panel;
      std::unique_ptr<wxButton> pub_event_btn;
      std::unique_ptr<wxStaticText> pub_event_txt;

      std::mutex main_grid_mut;
      std::mutex event_mut;
      bool got_new_event;
      uint64_t most_recent_event_time;
      bool new_grid_select;

    private:
      std::unique_ptr<ros::AsyncSpinner> spinner;
      ros::Publisher event_pub;
      ros::Subscriber can_sub;
      ros::Subscriber event_sub;

      void RedrawMessages();

      void OnCanMsg(const can_msgs::Frame::ConstPtr& msg);
      void OnExit(wxCommandEvent& event);
      void OnAbout(wxCommandEvent& event);
      void OnMsgsUpdate(wxThreadEvent& event);
      void OnMessageAnalyzerClick(wxCommandEvent& event);
      void OnSelectorBoxTick(wxCommandEvent& event);
      void OnUncheckAll(wxCommandEvent& event);
      void OnCheckAll(wxCommandEvent& event);
      void OnPublishEvent(wxCommandEvent& event);
      void OnEventPublished(const decanstructor::CanEvent::ConstPtr& msg);
      void OnGridSelect(wxGridEvent& event);
      void OnSortById(wxGridEvent& event);

      wxDECLARE_EVENT_TABLE();
  };

  class DCNode :
    public wxApp
  {
    public:
      virtual bool OnInit();
      DCFrame* frame;
      std::map<uint32_t, std::shared_ptr<CanMsgDetail>> rcvd_msgs;
      std::mutex rcvd_msgs_mut;
  };

	wxBEGIN_EVENT_TABLE(DCFrame, wxFrame)
		EVT_MENU(wxID_EXIT,  DCFrame::OnExit)
		EVT_MENU(wxID_ABOUT, DCFrame::OnAbout)
    EVT_BUTTON(ID_BTN_MESSAGE_ANALYZER, DCFrame::OnMessageAnalyzerClick)
    EVT_CHECKLISTBOX(wxID_ANY, DCFrame::OnSelectorBoxTick)
    EVT_BUTTON(ID_BTN_UNCHECK_ALL, DCFrame::OnUncheckAll)
    EVT_BUTTON(ID_BTN_CHECK_ALL, DCFrame::OnCheckAll)
    EVT_BUTTON(ID_BTN_PUBLISH_EVENT, DCFrame::OnPublishEvent)
    EVT_GRID_LABEL_LEFT_CLICK(DCFrame::OnGridSelect)
    EVT_GRID_CELL_LEFT_CLICK(DCFrame::OnGridSelect)
    EVT_GRID_COL_SORT(DCFrame::OnSortById)
	wxEND_EVENT_TABLE()

  wxDEFINE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);
}

wxIMPLEMENT_APP(DeCANstructor::DCNode);
wxDECLARE_APP(DeCANstructor::DCNode);

#endif
