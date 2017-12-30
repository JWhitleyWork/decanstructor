#include <decanstructor_node.h>

using namespace DeCANstructor;

uint16_t DCOptions::fade_out_time_ms = 3000;

void DCRenderTimer::Notify()
{
  auto& local_grid = wxGetApp().frame->active_grid;
  float ros_now_ms = ros::Time::now().toSec() * 1000.0;
  std::vector<CanMsgDetail> local_msgs;

  wxGetApp().rcvd_msgs_mut.lock();

  for (auto it = wxGetApp().rcvd_msgs.begin(); it != wxGetApp().rcvd_msgs.end(); it++)
  {
    bool needs_update = false;

    for (auto byte_it = it->second->last_updated.begin(); byte_it != it->second->last_updated.end(); byte_it++)
    {
      if (ros_now_ms - *byte_it < DCOptions::fade_out_time_ms)
        needs_update = true;
    }

    if (needs_update)
    {
      CanMsgDetail cmd_copy(*(it->second));
      local_msgs.push_back(cmd_copy);
    }
  }

  wxGetApp().rcvd_msgs_mut.unlock();

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++)
  {
    for (uint8_t i = 0; i < it->bytes.size(); i++)
    {
      float time_diff = (ros_now_ms - it->last_updated[i]);

      if (time_diff < DCOptions::fade_out_time_ms)
      {
        wxColour cell_color;
        float norm_time_interval = time_diff / (float)(DCOptions::fade_out_time_ms - 20);
        norm_time_interval = (norm_time_interval > 1.0) ? 1.0 : ((norm_time_interval < 0.0) ? 0.0 : norm_time_interval);

        if (norm_time_interval < 0.5)
        {
          float green = (norm_time_interval * 2.0) * 255.0;
          cell_color.Set(255, (unsigned char)green, 0);
        }
        else
        {
          float red_green = 255.0 - (((norm_time_interval - 0.5) * 2.0) * 255.0);
          cell_color.Set(red_green, red_green, 0);
        }

        local_grid->SetCellBackgroundColour(it->table_index, i + 1, cell_color);
      }
    }
  }
}

DCFrame::DCFrame(const wxString& title,
                 const wxPoint& pos,
                 const wxSize& size) :
  wxFrame(NULL,
          wxID_ANY,
          title,
          pos,
          size)
{
  // Set up basic window properties
  wxMenu* menu_file = new wxMenu;
  menu_file->Append(wxID_EXIT);

  wxMenu* menu_help = new wxMenu;
  menu_help->Append(wxID_ABOUT);

  wxMenuBar* menu_bar = new wxMenuBar;
  menu_bar->Append(menu_file, "&File");
  menu_bar->Append(menu_help, "&Help");

  SetMenuBar(menu_bar);

  CreateStatusBar();
  SetStatusText("Welcome to DeCANstructor.");

  // Create the sizer for the sub-widgets
  wxFlexGridSizer* main_sizer = new wxFlexGridSizer(4, 5, 5);

  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  wxSizerFlags main_flags;
  main_flags.Expand().Align(wxALIGN_TOP);

  // Create the main grid
  active_grid = std::shared_ptr<wxGrid>(new wxGrid(this, -1, wxPoint(0, 0), wxSize(510, 350)));

  // TODO: Automatically resize grid columns based on available space on window resize.

  active_grid->CreateGrid(0, 10);
  active_grid->EnableEditing(false);
  active_grid->DisableDragColSize();
  active_grid->DisableDragRowSize();
  active_grid->DisableDragGridSize();
  active_grid->SetSelectionMode(wxGrid::wxGridSelectRowsOrColumns);
  active_grid->SetColLabelSize(wxGRID_AUTOSIZE);
  active_grid->SetRowLabelSize(40);

  active_grid->SetColLabelValue(0, "ID");
  active_grid->SetColLabelValue(1, "0");
  active_grid->SetColLabelValue(2, "1");
  active_grid->SetColLabelValue(3, "2");
  active_grid->SetColLabelValue(4, "3");
  active_grid->SetColLabelValue(5, "4");
  active_grid->SetColLabelValue(6, "5");
  active_grid->SetColLabelValue(7, "6");
  active_grid->SetColLabelValue(8, "7");
  active_grid->SetColLabelValue(9, "Last Rcvd (ms)");

  active_grid->SetDefaultColSize(40);
  active_grid->SetColSize(0, 50);
  active_grid->SetColSize(9, 95);
  active_grid->SetColMinimalAcceptableWidth(40);
  active_grid->SetColMinimalWidth(0, 50);
  active_grid->SetColMinimalWidth(9, 95);

  wxGridCellAttr* id_attr = new wxGridCellAttr();
  id_attr->SetAlignment(wxALIGN_RIGHT, wxALIGN_CENTER);

  active_grid->SetColAttr(0, id_attr);

  for (uint8_t i = 1; i < 9; i++)
  {
    wxGridCellAttr* byte_attr = new wxGridCellAttr();
    byte_attr->SetAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    active_grid->SetColAttr(i, byte_attr);
  }

  active_grid->SetColFormatFloat(9);

  main_sizer->Add(active_grid.get(), main_flags);

  // Create the CAN ID selection box
  selector_box = std::shared_ptr<wxCheckListBox>(new wxCheckListBox());
  
  selector_box->Create(this, -1, wxPoint(0, 0), wxSize(200, 350));

  main_sizer->Add(selector_box.get(), main_flags);

  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);
  main_sizer->AddSpacer(5);

  main_sizer->AddGrowableCol(1, 3);
  main_sizer->AddGrowableCol(2, 1);
  main_sizer->AddGrowableRow(1);

  SetSizerAndFit(main_sizer);

  Connect(wxID_ANY, wxEVT_CMD_UPDATE_GRID, wxThreadEventHandler(DCFrame::OnGridUpdate), NULL, this);

  // Start the render update timer with a 10ms interval.
  render_timer = std::shared_ptr<DCRenderTimer>(new DCRenderTimer);
  render_timer->Start(10);
}

void DCFrame::OnExit(wxCommandEvent& event)
{
  Close(true);
}

void DCFrame::OnAbout(wxCommandEvent& event)
{
	wxMessageBox("Copyright 2017 Joshua Whitley, All Rights Reserved", "About DeCANstructor", wxOK | wxICON_INFORMATION );
}

void DCFrame::OnGridUpdate(wxThreadEvent& event)
{
  std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);
  auto& msgs_local = wxGetApp().rcvd_msgs;
  std::shared_ptr<CanMsgDetail> found_msg = std::shared_ptr<CanMsgDetail>(msgs_local[event.GetInt()]);

  if (found_msg->table_index < 0)
  {
    // Message not in grid
    active_grid->AppendRows();
    found_msg->table_index = active_grid->GetNumberRows() - 1;
    active_grid->SetCellValue(found_msg->table_index, 0, wxString::Format(wxT("0x%X"), event.GetInt()));

    for (uint8_t i = 0; i < found_msg->bytes.size(); i++)
    {
      active_grid->SetCellValue(found_msg->table_index, i + 1, wxString::Format(wxT("%X"), found_msg->bytes[i]));
    }

    active_grid->SetCellValue(found_msg->table_index, 9, wxString::Format(wxT("%.2f"), 0.00));
  }
  else
  {
    // Message in grid
    for (uint8_t i = 0; i < found_msg->bytes.size(); i++)
    {
      wxString formatted_byte = wxString::Format("%X", found_msg->bytes[i]);
      wxString current_byte = active_grid->GetCellValue(found_msg->table_index, i + 1);

      if (formatted_byte != current_byte)
      {
        active_grid->SetCellValue(found_msg->table_index, i + 1, wxString::Format(wxT("%X"), found_msg->bytes[i]));
      }
    }

    float time_diff = found_msg->time_rcvd - found_msg->time_last_rcvd;

    active_grid->SetCellValue(found_msg->table_index, 9, wxString::Format(wxT("%.2f"), time_diff));
  }
}

DCRosNode::DCRosNode()
{
  // ROS Init
  ros::init(wxGetApp().argc, wxGetApp().argv, "DeCANstructor");
  node_handle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
  private_handle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
  spinner = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));

  can_sub = node_handle->subscribe("can_in", 100, &DCRosNode::CanCallback, this);
  
  spinner->start();
}

void DCRosNode::CanCallback(const can_msgs::Frame::ConstPtr& msg)
{
  std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);

  auto& msgs_local = wxGetApp().rcvd_msgs;
  auto found_msg = msgs_local.find(msg->id);
  float header_ms = msg->header.stamp.toSec() * 1000.0;

  if (found_msg != msgs_local.end())
  {
    // The message has already been received.
    for (uint8_t i = 0; i < msg->data.size(); i++)
    {
      if (msg->data[i] != found_msg->second->bytes[i])
      {
        found_msg->second->bytes[i] = msg->data[i];
        found_msg->second->last_updated[i] = header_ms;
      }
    }

    found_msg->second->time_last_rcvd = found_msg->second->time_rcvd;
    found_msg->second->time_rcvd = header_ms;
  }
  else
  {
    // This is a new message.
    std::shared_ptr<CanMsgDetail> new_msg = std::shared_ptr<CanMsgDetail>(new CanMsgDetail);
    new_msg->table_index = -1;

    for (uint8_t i = 0; i < msg->data.size(); i++)
    {
      new_msg->bytes.push_back(msg->data[i]);
      new_msg->last_updated.push_back(header_ms);
    }

    new_msg->time_rcvd = header_ms;
    new_msg->time_last_rcvd = 0.0;
    msgs_local.insert(std::make_pair(msg->id, new_msg));
  }

  wxThreadEvent evt(wxEVT_CMD_UPDATE_GRID);
  evt.SetInt(msg->id);
  wxGetApp().frame->GetEventHandler()->QueueEvent(evt.Clone());
}

bool DCNode::OnInit()
{
  // wxWidgets Init
  frame = new DCFrame("DeCANstructor", wxPoint(50, 50), wxSize(450, 340));
  frame->Show(true);

  // ROS Node init
  ros_node = std::unique_ptr<DCRosNode>(new DCRosNode);

  return true;
}
