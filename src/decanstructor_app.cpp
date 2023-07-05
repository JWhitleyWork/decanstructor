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

#include "decanstructor/decanstructor_app.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

using CanEventMsgT = decanstructor::msg::CanEvent;
using CanFrameMsgT = can_msgs::msg::Frame;

namespace DeCANstructor
{

void DCRenderTimer::Notify()
{
  auto & local_grid = wxGetApp().frame->main_grid;
  auto & local_msgs = wxGetApp().rcvd_msgs;
  auto & local_node = wxGetApp().ros_node;
  auto & local_options = wxGetApp().options;
  auto & local_logger = wxGetApp().frame->logger;
  wxLog::SetActiveTarget(local_logger);

  wxLogMessage("Notify called.");

  uint64_t ros_now_ms = local_node->get_ros_time().time_since_epoch().count() / 1000000;
  std::vector<CellUpdate> cells_to_update;
  uint32_t grid_index = 0;

  std::lock_guard<std::mutex> grid_mut(wxGetApp().frame->main_grid_mut);

  std::unique_lock<std::mutex> msg_lock{wxGetApp().rcvd_msgs_mut};  // Lock messages

  if (local_grid->IsSortOrderAscending()) {
    for (auto it = local_msgs.begin(); it != local_msgs.end(); ++it) {
      if (it->second->grid_index > -1) {
        for (uint8_t i = 0; i < it->second->last_updated_ms.size(); i++) {
          uint64_t time_diff = ros_now_ms - it->second->last_updated_ms[i];

          if (time_diff < local_options.fade_out_time_ms && !(it->second->hidden)) {
            CellUpdate cu;
            cu.row = grid_index;
            cu.col = i + 1;
            cu.time_diff = time_diff;
            cells_to_update.push_back(cu);
          }
        }

        if ((ros_now_ms - it->second->time_rcvd_ms) < local_options.fade_out_time_ms) {
          local_grid->SetCellValue(
            grid_index, 9,
            wxString::Format(wxT("%u"), it->second->avg_rate));
        }

        grid_index++;
      }
    }
  } else {
    for (auto it = local_msgs.rbegin(); it != local_msgs.rend(); ++it) {
      if (it->second->grid_index > -1) {
        for (uint8_t i = 0; i < it->second->last_updated_ms.size(); i++) {
          uint64_t time_diff = ros_now_ms - it->second->last_updated_ms[i];

          if (time_diff < local_options.fade_out_time_ms && !(it->second->hidden)) {
            CellUpdate cu;
            cu.row = grid_index;
            cu.col = i + 1;
            cu.time_diff = time_diff;
            cells_to_update.push_back(cu);
          }
        }

        if ((ros_now_ms - it->second->time_rcvd_ms) < local_options.fade_out_time_ms) {
          local_grid->SetCellValue(
            grid_index, 9,
            wxString::Format(wxT("%u"), it->second->avg_rate));
        }

        grid_index++;
      }
    }
  }

  msg_lock.unlock();  // Unlock messages

  for (auto it = cells_to_update.begin(); it != cells_to_update.end(); it++) {
    if (it->row >= 0 && it->row < local_grid->GetNumberRows()) {
      wxColour cell_color;
      wxColour text_color;
      float norm_time_interval =
        it->time_diff / static_cast<float>(local_options.fade_out_time_ms - 100);
      norm_time_interval =
        (norm_time_interval >
        1.0) ? 1.0 : ((norm_time_interval < 0.0) ? 0.0 : norm_time_interval);
      uint8_t text_fade = (uint8_t)((1.0 - norm_time_interval) * 255.0);
      text_color.Set(text_fade, text_fade, text_fade);

      if (norm_time_interval < 0.5) {
        uint8_t green = (uint8_t)((norm_time_interval * 2.0) * 255.0);
        cell_color.Set(255, green, 0);
      } else {
        uint8_t blue = (uint8_t)(((norm_time_interval - 0.5) * 2.0) * 255.0);
        cell_color.Set(255, 255, blue);
      }

      local_grid->SetCellBackgroundColour(it->row, it->col, cell_color);
      local_grid->SetCellTextColour(it->row, it->col, text_color);
    }
  }

  if (local_options.event_mode == EventMode::PLAYBACK) {
    std::lock_guard<std::mutex> event_mut(wxGetApp().frame->event_mut);

    if (wxGetApp().frame->got_new_event) {
      uint64_t time_diff = ros_now_ms - wxGetApp().frame->most_recent_event_time;

      if (time_diff < local_options.fade_out_time_ms) {
        float norm_time_interval =
          time_diff / static_cast<float>(local_options.fade_out_time_ms - 100);
        norm_time_interval =
          (norm_time_interval >
          1.0) ? 1.0 : ((norm_time_interval < 0.0) ? 0.0 : norm_time_interval);
        uint8_t color_fade = (uint8_t)(norm_time_interval * 255.0);

        wxColour panel_color;
        panel_color.Set(color_fade, 255, color_fade);
        wxGetApp().frame->event_panel->SetBackgroundColour(panel_color);

        wxColour text_color;
        text_color.Set(color_fade, color_fade, color_fade);
        wxGetApp().frame->pub_event_txt->SetForegroundColour(text_color);

        if (norm_time_interval == 1.0) {
          wxGetApp().frame->got_new_event = false;
        }
      }
    }
  }

  if (wxGetApp().frame->new_grid_select) {
    wxArrayInt selected_rows = wxGetApp().frame->main_grid->GetSelectedRows();

    if (selected_rows.GetCount() == 1) {
      wxGetApp().frame->message_analyzer_btn->Enable(true);
    } else {
      wxGetApp().frame->message_analyzer_btn->Enable(false);
    }

    wxGetApp().frame->new_grid_select = false;
  }

  // Update ROS stuff
  if (!rclcpp::ok()) {
    // Shutting down
    wxGetApp().frame->Close(true);
  } else {
    wxGetApp().ros_executor.spin_some();
  }
}

DCFrame::DCFrame(
  const wxString & title,
  const wxPoint & pos,
  const wxSize & size)
: wxFrame(NULL, wxID_ANY, title, pos, size),
  got_new_event(false),
  most_recent_event_time(0)
{
  // Set up the logger
  logger = new wxLogStderr();
  wxLog::SetActiveTarget(logger);

  wxLogMessage("Started initialization");

  // Set up basic window properties
  wxMenu * menu_file = new wxMenu;
  menu_file->Append(wxID_EXIT);

  wxMenu * menu_help = new wxMenu;
  menu_help->Append(wxID_ABOUT);

  wxMenuBar * menu_bar = new wxMenuBar;
  menu_bar->Append(menu_file, "&File");
  menu_bar->Append(menu_help, "&Help");

  SetMenuBar(menu_bar);

  CreateStatusBar();
  SetStatusText("Welcome to DeCANstructor.");

  // Create the sizers for the sub-widgets
  wxFlexGridSizer * main_sizer = new wxFlexGridSizer(4, 5, 5);
  wxBoxSizer * right_sizer = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer * message_analyzer_sizer = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer * chkbx_cntrl_sizer = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer * event_sizer = new wxStaticBoxSizer(wxHORIZONTAL, this, "Events");

  // Create basic flags
  wxSizerFlags no_flags;
  wxSizerFlags expand_flag;
  wxSizerFlags main_flags;

  expand_flag.Expand();
  main_flags.Expand().Align(wxALIGN_TOP);

  // Main Sizer - 1st Row
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  // Main Sizer - 2nd Row
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  // Create the main grid
  main_grid = std::make_unique<wxGrid>(this, -1, wxPoint(0, 0), wxSize(600, 250));

  // TODO(jwhitleywork): Automatically resize grid columns based on available space
  // on window resize.

  main_grid->CreateGrid(0, 10);
  main_grid->EnableEditing(false);
  main_grid->DisableDragColSize();
  main_grid->DisableDragRowSize();
  main_grid->DisableDragGridSize();
  main_grid->SetSelectionMode(wxGrid::wxGridSelectRows);
  main_grid->SetColLabelSize(wxGRID_AUTOSIZE);
  main_grid->SetRowLabelSize(40);
  main_grid->SetSelectionBackground(wxColour(150, 150, 255));

  main_grid->SetColLabelValue(0, "ID");
  main_grid->SetColLabelValue(1, "0");
  main_grid->SetColLabelValue(2, "1");
  main_grid->SetColLabelValue(3, "2");
  main_grid->SetColLabelValue(4, "3");
  main_grid->SetColLabelValue(5, "4");
  main_grid->SetColLabelValue(6, "5");
  main_grid->SetColLabelValue(7, "6");
  main_grid->SetColLabelValue(8, "7");
  main_grid->SetColLabelValue(9, "Avg Rate (ms)");

  main_grid->SetDefaultColSize(40);
  main_grid->SetColSize(0, 100);
  main_grid->SetColSize(9, 105);
  main_grid->SetColMinimalAcceptableWidth(40);
  main_grid->SetColMinimalWidth(0, 100);
  main_grid->SetColMinimalWidth(9, 105);
  main_grid->SetSortingColumn(0);

  main_grid->SetColFormatNumber(9);

  wxGridCellAttr * id_attr = new wxGridCellAttr();
  id_attr->SetAlignment(wxALIGN_LEFT, wxALIGN_CENTER);

  main_grid->SetColAttr(0, id_attr);

  for (uint8_t i = 1; i < 9; i++) {
    wxGridCellAttr * byte_attr = new wxGridCellAttr();
    byte_attr->SetAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    main_grid->SetColAttr(i, byte_attr);
  }

  main_sizer->Add(main_grid.get(), main_flags.Proportion(1));

  // Create the Message Analyzer button
  message_analyzer_btn = std::make_unique<wxButton>();
  message_analyzer_btn->Create(
    this, static_cast<int>(ButtonType::MESSAGE_ANALYZER),
    "Analyze Message");
  message_analyzer_btn->Enable(false);  // Start disabled

  message_analyzer_sizer->Add(message_analyzer_btn.get(), expand_flag.Proportion(1));

  right_sizer->Add(message_analyzer_sizer, expand_flag.Proportion(0));
  right_sizer->AddSpacer(5);

  // Create the Control Buttons for the CAN ID Selection Box
  wxButton * uncheck_all_btn = new wxButton();
  wxButton * check_all_btn = new wxButton();

  uncheck_all_btn->Create(this, static_cast<int>(ButtonType::UNCHECK_ALL), "Uncheck All");
  check_all_btn->Create(this, static_cast<int>(ButtonType::CHECK_ALL), "Check All");

  chkbx_cntrl_sizer->Add(uncheck_all_btn, expand_flag.Proportion(1));
  chkbx_cntrl_sizer->AddSpacer(5);
  chkbx_cntrl_sizer->Add(check_all_btn, expand_flag.Proportion(1));

  right_sizer->Add(chkbx_cntrl_sizer, expand_flag.Proportion(0));
  right_sizer->AddSpacer(5);

  // Create the CAN ID Selection Box
  selector_box = std::make_unique<wxCheckListBox>();
  selector_box->Create(this, -1, wxPoint(0, 0), wxSize(200, 350));

  right_sizer->Add(selector_box.get(), main_flags.Proportion(1));

  right_sizer->AddSpacer(5);

  // Create the Event Control Box
  event_panel = std::make_unique<wxPanel>(this, wxID_ANY, wxDefaultPosition, wxSize(200, 50));

  pub_event_btn = std::make_unique<wxButton>();
  pub_event_txt = std::make_unique<wxStaticText>();

  pub_event_btn->Create(
    event_panel.get(), static_cast<int>(ButtonType::PUBLISH_EVENT), "Publish Event", wxPoint(
      0,
      0), wxSize(
      200, 50));
  pub_event_txt->Create(event_panel.get(), wxID_ANY, "");
  pub_event_txt->SetLabelMarkup("<b><big>Event Published</big></b>");
  pub_event_txt->SetForegroundColour(wxColour(255, 255, 255));

  pub_event_txt->CentreOnParent();

  event_sizer->Add(event_panel.get());

  right_sizer->Add(event_sizer, expand_flag.Proportion(0));

  main_sizer->Add(right_sizer, main_flags.Right());

  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  // Main Sizer - 3rd Row
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  main_sizer->AddGrowableCol(1, 3);
  main_sizer->AddGrowableRow(1);

  Connect(wxID_ANY, wxEVT_CMD_UPDATE_MSGS, wxThreadEventHandler(DCFrame::OnMsgsUpdate), NULL, this);

  // Start the render update timer with a 10ms interval.
  render_timer = std::make_unique<DCRenderTimer>();
  render_timer->Start(100);

  if (wxGetApp().options.event_mode == EventMode::PLAYBACK) {
    pub_event_btn->Hide();
  } else {
    pub_event_txt->Hide();
  }

  // We want this to happen last after playback mode takes effect.
  SetSizerAndFit(main_sizer);
}

DCFrame::~DCFrame()
{
  wxLog::SetActiveTarget(nullptr);
  delete logger;
}

void DCFrame::RedrawMessages()
{
  std::lock_guard<std::mutex> msgs_mut(wxGetApp().rcvd_msgs_mut);
  std::lock_guard<std::mutex> grid_mut(wxGetApp().frame->main_grid_mut);

  auto & local_grid = wxGetApp().frame->main_grid;
  auto local_table = local_grid->GetTable();
  auto & local_msgs = wxGetApp().rcvd_msgs;

  if (local_table->GetNumberRows() > 0) {
    local_table->DeleteRows(0, local_table->GetNumberRows());
  }

  int item_count = wxGetApp().frame->selector_box->GetCount();

  if (item_count > 0) {
    for (int i = item_count - 1; i > -1; --i) {
      wxGetApp().frame->selector_box->Delete(i);
    }
  }

  if (local_grid->IsSortOrderAscending()) {
    for (auto it = local_msgs.begin(); it != local_msgs.end(); ++it) {
      local_table->AppendRows();
      int row_index = local_table->GetNumberRows() - 1;

      local_table->SetValue(row_index, 0, wxString::Format(wxT("0x%03X"), it->first));

      for (uint8_t i = 0; i < it->second->bytes.size(); i++) {
        local_table->SetValue(
          row_index, i + 1,
          wxString::Format(wxT("%02X"), it->second->bytes[i]));
      }

      local_table->SetValue(row_index, 9, wxString::Format(wxT("%u"), 0));

      // Add to selector box
      int selector_index = selector_box->Append(wxString::Format(wxT("0x%03X"), it->first));

      if (it->second->hidden) {
        local_grid->HideRow(row_index);
      } else {
        selector_box->Check(selector_index);
      }

      it->second->grid_index = row_index;
    }
  } else {
    for (auto it = local_msgs.rbegin(); it != local_msgs.rend(); ++it) {
      local_table->AppendRows();
      int row_index = local_table->GetNumberRows() - 1;

      local_table->SetValue(row_index, 0, wxString::Format(wxT("0x%03X"), it->first));

      for (uint8_t i = 0; i < it->second->bytes.size(); i++) {
        local_table->SetValue(
          row_index, i + 1,
          wxString::Format(wxT("%02X"), it->second->bytes[i]));
      }

      local_table->SetValue(row_index, 9, wxString::Format(wxT("%u"), 0));

      // Add to selector box
      int selector_index = selector_box->Append(wxString::Format(wxT("0x%03X"), it->first));

      if (it->second->hidden) {
        local_grid->HideRow(row_index);
      } else {
        selector_box->Check(selector_index);
      }

      it->second->grid_index = row_index;
    }
  }

  local_grid->ForceRefresh();
}

void DCFrame::OnExit(wxCommandEvent & event)
{
  (void)event;
  Close(true);
}

void DCFrame::OnAbout(wxCommandEvent & event)
{
  (void)event;
  wxMessageBox(
    "Copyright 2017 Joshua Whitley, All Rights Reserved", "About DeCANstructor",
    wxOK | wxICON_INFORMATION);
}

void DCFrame::OnMsgsUpdate(wxThreadEvent & event)
{
  if (event.GetString() == "true") {
    // New message - Redraw the grid
    RedrawMessages();
  } else if (event.GetString() == "false") {
    // Existing message - Just update the grid row
    std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);
    auto & local_msgs = wxGetApp().rcvd_msgs;
    auto found_msg = local_msgs[event.GetInt()];

    for (uint8_t i = 0; i < found_msg->bytes.size(); i++) {
      if (found_msg->bytes[i] != found_msg->last_bytes[i]) {
        main_grid->SetCellValue(
          found_msg->grid_index, i + 1,
          wxString::Format(wxT("%02X"), found_msg->bytes[i]));
      }
    }

    uint64_t time_diff = found_msg->time_rcvd_ms - found_msg->time_last_rcvd_ms;

    // The new time difference should be
    // less than 1 minute (to be safe)
    if (time_diff < 60000) {
      // Make sure we don't have an average already
      if (found_msg->avg_rate == 0) {
        found_msg->avg_rate = (unsigned int)time_diff;
      } else {
        found_msg->avg_rate = (found_msg->avg_rate + (unsigned int)time_diff) / 2;
      }
    }
  }
}

void DCFrame::OnMessageAnalyzerClick(wxCommandEvent & event)
{
  (void)event;

  // Set up Message Analyzer window
  auto ma_frame = std::make_unique<DCMessageAnalyzerFrame>(
    this, "Message Analyzer", wxPoint(100, 100), wxSize(250, 250));
  ma_frame->Show(true);
}

void DCFrame::OnSelectorBoxTick(wxCommandEvent & event)
{
  auto & local_selector_box = wxGetApp().frame->selector_box;

  if (local_selector_box->IsChecked(event.GetInt())) {
    wxGetApp().frame->main_grid->ShowRow(event.GetInt());
  } else {
    wxGetApp().frame->main_grid->HideRow(event.GetInt());
  }

  std::lock_guard<std::mutex> selector_mut(wxGetApp().rcvd_msgs_mut);

  auto & local_msgs = wxGetApp().rcvd_msgs;

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++) {
    if (it->second->grid_index == event.GetInt()) {
      it->second->hidden = !local_selector_box->IsChecked(event.GetInt());
    }
  }
}

void DCFrame::OnUncheckAll(wxCommandEvent & event)
{
  (void)event;

  std::lock_guard<std::mutex> uncheck_mut(wxGetApp().rcvd_msgs_mut);
  auto & local_msgs = wxGetApp().rcvd_msgs;

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++) {
    it->second->hidden = true;
    wxGetApp().frame->main_grid->HideRow(it->second->grid_index);
    wxGetApp().frame->selector_box->Check(it->second->grid_index, false);
  }
}

void DCFrame::OnCheckAll(wxCommandEvent & event)
{
  (void)event;

  std::lock_guard<std::mutex> check_mut(wxGetApp().rcvd_msgs_mut);
  auto & local_msgs = wxGetApp().rcvd_msgs;

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++) {
    it->second->hidden = false;
    wxGetApp().frame->main_grid->ShowRow(it->second->grid_index);
    wxGetApp().frame->selector_box->Check(it->second->grid_index, true);
  }
}

void DCFrame::OnPublishEvent(wxCommandEvent & event)
{
  (void)event;
  wxGetApp().ros_node->publish_can_event();
}

void DCFrame::OnEventPublished()
{
  std::lock_guard<std::mutex> event_mut(wxGetApp().frame->event_mut);
  wxGetApp().frame->got_new_event = true;
  wxGetApp().frame->most_recent_event_time =
    wxGetApp().ros_node->get_ros_time().time_since_epoch().count() / 1000000;
}

void DCFrame::OnCanMsg(const CanFrameMsgT::SharedPtr msg)
{
  std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);

  auto & local_msgs = wxGetApp().rcvd_msgs;
  auto found_msg = local_msgs.find(msg->id);

  // Do the following to get reasonable numbers to play with.
  uint64_t ros_now_ms = wxGetApp().ros_node->get_ros_time().time_since_epoch().count() / 1000000;

  // The int value is the CAN ID.
  // The string value (true or false) indicates if this is a new message.
  wxThreadEvent evt(wxEVT_CMD_UPDATE_MSGS);
  evt.SetInt(msg->id);

  if (found_msg != local_msgs.end()) {
    // The message has already been received.
    // Store the old bytes for later comparison.
    std::copy(
      found_msg->second->bytes.begin(),
      found_msg->second->bytes.end(), found_msg->second->last_bytes.begin());

    for (uint8_t i = 0; i < msg->data.size(); i++) {
      if (msg->data[i] != found_msg->second->bytes[i]) {
        found_msg->second->bytes[i] = msg->data[i];
        found_msg->second->last_updated_ms[i] = ros_now_ms;
      }
    }

    found_msg->second->time_last_rcvd_ms = found_msg->second->time_rcvd_ms;
    found_msg->second->time_rcvd_ms = ros_now_ms;

    evt.SetString("false");
  } else {
    // This is a new message.
    auto new_msg = std::make_shared<CanMsgDetail>();

    for (uint8_t i = 0; i < msg->data.size(); i++) {
      new_msg->bytes.push_back(msg->data[i]);
      new_msg->last_bytes.push_back(msg->data[i]);
      new_msg->last_updated_ms.push_back(ros_now_ms);
    }

    new_msg->time_rcvd_ms = ros_now_ms;
    new_msg->time_last_rcvd_ms = 0;
    local_msgs.insert(std::make_pair(msg->id, new_msg));

    evt.SetString("true");
  }

  wxGetApp().frame->GetEventHandler()->QueueEvent(evt.Clone());
}

void DCFrame::OnGridSelect(wxGridEvent & event)
{
  if (event.GetRow() != -1) {
    new_grid_select = true;
  }

  event.Skip();
}

void DCFrame::OnSortById(wxGridEvent & event)
{
  if (event.GetCol() == 0) {
    auto & local_grid = wxGetApp().frame->main_grid;

    if (local_grid->IsSortOrderAscending()) {
      // Switch to descending
      local_grid->SetSortingColumn(0, false);
    } else {
      // Switch back to ascending
      local_grid->SetSortingColumn(0, true);
    }

    local_grid->GetGridColLabelWindow()->Update();
    RedrawMessages();
  }

  event.Skip();
}

bool DCApp::OnInit()
{
  // ROS init
  rclcpp::init(wxGetApp().argc, wxGetApp().argv);

  // wxWidgets Init
  frame = std::make_unique<DCFrame>("DeCANstructor", wxPoint(50, 50), wxSize(450, 340));
  frame->Show(true);

  SetTopWindow(frame.get());

  rclcpp::NodeOptions node_options;
  ros_node = std::make_unique<DCRosNode>(node_options);

  if (ros_node->in_playback_mode()) {
    wxGetApp().options.event_mode = EventMode::PLAYBACK;
  } else {
    wxGetApp().options.event_mode = EventMode::REAL_TIME;
  }

  ros_node->register_can_msg_callback(
    std::bind(&DCFrame::OnCanMsg, frame.get(), std::placeholders::_1));

  if (options.event_mode == EventMode::PLAYBACK) {
    ros_node->register_can_event_callback(std::bind(&DCFrame::OnEventPublished, frame.get()));
  }

  ros_executor.add_node(ros_node->get_node_base_interface());

  return true;
}

int DCApp::OnExit()
{
  // Shut down ROS node
  rclcpp::shutdown();

  // Destroy objects and members
  ros_node.reset();
  frame.reset();

  return 0;
}

}  // namespace DeCANstructor
