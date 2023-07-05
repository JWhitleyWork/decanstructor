// Copyright 2017-2023 Joshua Whitley
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DECANSTRUCTOR__DECANSTRUCTOR_APP_HPP_
#define DECANSTRUCTOR__DECANSTRUCTOR_APP_HPP_

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif
#include <wx/grid.h>

#include <can_msgs/msg/frame.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "decanstructor/decanstructor_node.hpp"
#include "decanstructor/message_analyzer.hpp"

using CanFrameMsgT = can_msgs::msg::Frame;

namespace DeCANstructor
{

wxDECLARE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);

enum class ButtonType
{
  MESSAGE_ANALYZER = 1,
  UNCHECK_ALL,
  CHECK_ALL,
  PUBLISH_EVENT
};

enum class EventMode
{
  REAL_TIME,
  PLAYBACK
};

enum class SortDirection
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

// TODO(jwhitleywork): Finish implementing custom grid table.
class DCGridTable : public wxGridTableBase
{
  int GetNumberRows();
  int GetNumberCols();
  wxString GetValue(int row, int col);
  void SetValue(int row, int col, const wxString & value);
  void Clear();
  bool InsertRows(size_t pos = 0, size_t numRows = 1);
  bool AppendRows(size_t numRows = 1);
  bool DeleteRows(size_t pos = 0, size_t numRows = 1);
  bool InsertCols(size_t pos = 0, size_t numCols = 1);
  bool AppendCols(size_t numCols = 1);
  bool DeleteCols(size_t pos = 0, size_t numCols = 1);
  void SetRowLabelValue(int row, const wxString &);
  void SetColLabelValue(int col, const wxString &);
  wxString GetRowLabelValue(int row);
  wxString GetColLabelValue(int col);
};

class DCOptions
{
public:
  uint16_t fade_out_time_ms;
  EventMode event_mode;
};

class DCRenderTimer : public wxTimer
{
public:
  void Notify();
};

class DCFrame : public wxFrame
{
public:
  DCFrame(
    const wxString & title,
    const wxPoint & pos,
    const wxSize & size
  );

  void OnEventPublished();
  void OnCanMsg(const CanFrameMsgT::SharedPtr msg);

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
  void RedrawMessages();
  void OnExit(wxCommandEvent & event);
  void OnAbout(wxCommandEvent & event);
  void OnMsgsUpdate(wxThreadEvent & event);
  void OnMessageAnalyzerClick(wxCommandEvent & event);
  void OnSelectorBoxTick(wxCommandEvent & event);
  void OnUncheckAll(wxCommandEvent & event);
  void OnCheckAll(wxCommandEvent & event);
  void OnPublishEvent(wxCommandEvent & event);
  void OnGridSelect(wxGridEvent & event);
  void OnSortById(wxGridEvent & event);

  wxDECLARE_EVENT_TABLE();
};

class DCApp : public wxApp
{
public:
  bool OnInit() override;
  int OnExit() override;

  std::unique_ptr<DCFrame> frame;
  std::map<uint32_t, std::shared_ptr<CanMsgDetail>> rcvd_msgs;
  std::mutex rcvd_msgs_mut;
  std::unique_ptr<DCRosNode> ros_node;
  DCOptions options;
};

wxBEGIN_EVENT_TABLE(DCFrame, wxFrame)
EVT_MENU(wxID_EXIT, DCFrame::OnExit)
EVT_MENU(wxID_ABOUT, DCFrame::OnAbout)
EVT_BUTTON(static_cast<int>(ButtonType::MESSAGE_ANALYZER), DCFrame::OnMessageAnalyzerClick)
EVT_CHECKLISTBOX(wxID_ANY, DCFrame::OnSelectorBoxTick)
EVT_BUTTON(static_cast<int>(ButtonType::UNCHECK_ALL), DCFrame::OnUncheckAll)
EVT_BUTTON(static_cast<int>(ButtonType::CHECK_ALL), DCFrame::OnCheckAll)
EVT_BUTTON(static_cast<int>(ButtonType::PUBLISH_EVENT), DCFrame::OnPublishEvent)
EVT_GRID_LABEL_LEFT_CLICK(DCFrame::OnGridSelect)
EVT_GRID_CELL_LEFT_CLICK(DCFrame::OnGridSelect)
EVT_GRID_COL_SORT(DCFrame::OnSortById)
wxEND_EVENT_TABLE()

wxDEFINE_EVENT(wxEVT_CMD_UPDATE_MSGS, wxThreadEvent);

}  // namespace DeCANstructor

wxIMPLEMENT_APP(DeCANstructor::DCApp);
wxDECLARE_APP(DeCANstructor::DCApp);

#endif  // DECANSTRUCTOR__DECANSTRUCTOR_APP_HPP_
