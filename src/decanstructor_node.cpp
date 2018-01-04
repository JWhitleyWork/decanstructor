#include <decanstructor_node.h>

using namespace DeCANstructor;

uint16_t DCOptions::fade_out_time_ms = 3000;
ros::Time DCOptions::one_day_ago = ros::Time();
EventMode DCOptions::event_mode = REAL_TIME;

void DCRenderTimer::Notify()
{
  auto& local_grid = wxGetApp().frame->main_grid;
  uint64_t ros_now_ms = (ros::Time::now() - DCOptions::one_day_ago).toNSec() / 1000000;
  std::vector<CellUpdate> cells_to_update;

  wxGetApp().rcvd_msgs_mut.lock();

  for (auto it = wxGetApp().rcvd_msgs.begin(); it != wxGetApp().rcvd_msgs.end(); it++)
  {
    for (uint8_t i = 0; i < it->second->last_updated_ms.size(); i++) 
    {
      uint64_t time_diff = ros_now_ms - it->second->last_updated_ms[i];

      if (time_diff < DCOptions::fade_out_time_ms && !(it->second->hidden))
      {
        CellUpdate cu;
        cu.row = it->second->grid_index;
        cu.col = i + 1;
        cu.time_diff = time_diff;
        cells_to_update.push_back(cu);
      }
    }

    if ((it->second->grid_index > -1) &&
        (ros_now_ms - it->second->time_rcvd_ms) < DCOptions::fade_out_time_ms)
      local_grid->SetCellValue(it->second->grid_index, 9, wxString::Format(wxT("%u"), it->second->avg_rate));
  }

  wxGetApp().rcvd_msgs_mut.unlock();

  for (auto it = cells_to_update.begin(); it != cells_to_update.end(); it++)
  {
    wxColour cell_color;
    wxColour text_color;
    float norm_time_interval = it->time_diff / (float)(DCOptions::fade_out_time_ms - 100);
    norm_time_interval = (norm_time_interval > 1.0) ? 1.0 : ((norm_time_interval < 0.0) ? 0.0 : norm_time_interval);
    uint8_t text_fade = (uint8_t)((1.0 - norm_time_interval) * 255.0);
    text_color.Set(text_fade, text_fade, text_fade);

    if (norm_time_interval < 0.5)
    {
      uint8_t green = (uint8_t)((norm_time_interval * 2.0) * 255.0);
      cell_color.Set(255, green, 0);
    }
    else
    {
      uint8_t blue = (uint8_t)(((norm_time_interval - 0.5) * 2.0) * 255.0);
      cell_color.Set(255, 255, blue);
    }

    local_grid->SetCellBackgroundColour(it->row, it->col, cell_color);
    local_grid->SetCellTextColour(it->row, it->col, text_color);
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

  // Create the sizers for the sub-widgets
  wxFlexGridSizer* main_sizer = new wxFlexGridSizer(4, 5, 5);
  wxBoxSizer* right_sizer = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer* chkbx_cntrl_sizer = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer* event_sizer = new wxStaticBoxSizer(wxHORIZONTAL, this, "Events");

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
  main_grid = std::shared_ptr<wxGrid>(new wxGrid(this, -1, wxPoint(0, 0), wxSize(600, 250)));

  // TODO: Automatically resize grid columns based on available space on window resize.

  main_grid->CreateGrid(0, 10);
  main_grid->EnableEditing(false);
  main_grid->DisableDragColSize();
  main_grid->DisableDragRowSize();
  main_grid->DisableDragGridSize();
  main_grid->SetSelectionMode(wxGrid::wxGridSelectRowsOrColumns);
  main_grid->SetColLabelSize(wxGRID_AUTOSIZE);
  main_grid->SetRowLabelSize(40);

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

  main_grid->SetColFormatNumber(9);

  wxGridCellAttr* id_attr = new wxGridCellAttr();
  id_attr->SetAlignment(wxALIGN_LEFT, wxALIGN_CENTER);

  main_grid->SetColAttr(0, id_attr);

  for (uint8_t i = 1; i < 9; i++)
  {
    wxGridCellAttr* byte_attr = new wxGridCellAttr();
    byte_attr->SetAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    main_grid->SetColAttr(i, byte_attr);
  }

  main_sizer->Add(main_grid.get(), main_flags.Proportion(1));

  // Create the Control Buttons for the CAN ID Selection Box
  wxButton* uncheck_all_btn = new wxButton();
  wxButton* check_all_btn = new wxButton();

  uncheck_all_btn->Create(this, ID_BTN_UNCHECK_ALL, "Uncheck All");
  check_all_btn->Create(this, ID_BTN_CHECK_ALL, "Check All");

  chkbx_cntrl_sizer->Add(uncheck_all_btn, expand_flag.Proportion(1));
  chkbx_cntrl_sizer->AddSpacer(5);
  chkbx_cntrl_sizer->Add(check_all_btn, expand_flag.Proportion(1));

  right_sizer->Add(chkbx_cntrl_sizer, expand_flag.Proportion(0));
  right_sizer->AddSpacer(5);

  // Create the CAN ID Selection Box
  selector_box = std::shared_ptr<wxCheckListBox>(new wxCheckListBox());
  selector_box->Create(this, -1, wxPoint(0, 0), wxSize(200, 350));

  right_sizer->Add(selector_box.get(), main_flags.Proportion(1));

  right_sizer->AddSpacer(5);

  // Create the Event Control Box
  wxPanel* event_panel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(200, 50));

  pub_event_btn = std::shared_ptr<wxButton>(new wxButton());
  pub_event_txt = std::shared_ptr<wxStaticText>(new wxStaticText());

  pub_event_btn->Create(event_panel, ID_BTN_PUBLISH_EVENT, "Publish Event", wxPoint(0, 0), wxSize(200, 50));
  pub_event_txt->Create(event_panel, wxID_ANY, "");
  pub_event_txt->SetLabelMarkup("<b><big>Event Published</big></b>");

  pub_event_txt->CentreOnParent();

  event_sizer->Add(event_panel);

  pub_event_txt->Hide();

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

  SetSizerAndFit(main_sizer);

  Connect(wxID_ANY, wxEVT_CMD_UPDATE_MSGS, wxThreadEventHandler(DCFrame::OnMsgsUpdate), NULL, this);
  Connect(wxID_ANY, wxEVT_CMD_EVENT_PUBLISHED, wxThreadEventHandler(DCFrame::OnEventPublished), NULL, this);

  // Start the render update timer with a 10ms interval.
  render_timer = std::shared_ptr<DCRenderTimer>(new DCRenderTimer);
  render_timer->Start(100);
}

void DCFrame::OnExit(wxCommandEvent& event)
{
  Close(true);
}

void DCFrame::OnAbout(wxCommandEvent& event)
{
	wxMessageBox("Copyright 2017 Joshua Whitley, All Rights Reserved", "About DeCANstructor", wxOK | wxICON_INFORMATION );
}

void DCFrame::OnMsgsUpdate(wxThreadEvent& event)
{
  std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);
  auto& msgs_local = wxGetApp().rcvd_msgs;
  std::shared_ptr<CanMsgDetail> found_msg = std::shared_ptr<CanMsgDetail>(msgs_local[event.GetInt()]);

  if (found_msg->grid_index < 0)
  {
    // New message
    // Add to grid
    main_grid->AppendRows();
    found_msg->grid_index = main_grid->GetNumberRows() - 1;
    main_grid->SetCellValue(found_msg->grid_index, 0, wxString::Format(wxT("0x%03X"), event.GetInt()));

    for (uint8_t i = 0; i < found_msg->bytes.size(); i++)
    {
      main_grid->SetCellValue(found_msg->grid_index, i + 1, wxString::Format(wxT("%02X"), found_msg->bytes[i]));
    }

    main_grid->SetCellValue(found_msg->grid_index, 9, wxString::Format(wxT("%u"), 0));

    // Add to selector box
    found_msg->selector_index = selector_box->Append(wxString::Format(wxT("0x%03X"), event.GetInt()));
    selector_box->Check(found_msg->selector_index);
  }
  else
  {
    // Message in grid
    for (uint8_t i = 0; i < found_msg->bytes.size(); i++)
    {
      if (found_msg->bytes[i] != found_msg->last_bytes[i])
      {
        main_grid->SetCellValue(found_msg->grid_index, i + 1, wxString::Format(wxT("%02X"), found_msg->bytes[i]));
      }
    }

    uint64_t time_diff = found_msg->time_rcvd_ms - found_msg->time_last_rcvd_ms;
    
    // The new time difference should be
    // less than 1 minute (to be safe)
    if (time_diff < 60000)
    {
      // Make sure we don't have an average already
      if (found_msg->avg_rate == 0)
        found_msg->avg_rate = (unsigned int)time_diff;
      else
        found_msg->avg_rate = (found_msg->avg_rate + (unsigned int)time_diff) / 2;
    }
  }
}

void DCFrame::OnSelectorBoxTick(wxCommandEvent& event)
{
  auto& local_selector_box = wxGetApp().frame->selector_box;

  wxGetApp().rcvd_msgs_mut.lock();

  for (auto it = wxGetApp().rcvd_msgs.begin(); it != wxGetApp().rcvd_msgs.end(); it++)
  {
    if (it->second->selector_index == event.GetInt())
    {
      it->second->hidden = !local_selector_box->IsChecked(event.GetInt());

      if (it->second->hidden)
        wxGetApp().frame->main_grid->HideRow(it->second->grid_index);
      else
        wxGetApp().frame->main_grid->ShowRow(it->second->grid_index);
    }
  }

  wxGetApp().rcvd_msgs_mut.unlock();
}

void DCFrame::OnUncheckAll(wxCommandEvent& event)
{
  std::lock_guard<std::mutex> uncheck_mut(wxGetApp().rcvd_msgs_mut);
  auto& local_msgs = wxGetApp().rcvd_msgs;

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++)
  {
    it->second->hidden = true;
    wxGetApp().frame->main_grid->HideRow(it->second->grid_index);
    wxGetApp().frame->selector_box->Check(it->second->selector_index, false);
  }
}

void DCFrame::OnCheckAll(wxCommandEvent& event)
{
  std::lock_guard<std::mutex> check_mut(wxGetApp().rcvd_msgs_mut);
  auto& local_msgs = wxGetApp().rcvd_msgs;

  for (auto it = local_msgs.begin(); it != local_msgs.end(); it++)
  {
    it->second->hidden = false;
    wxGetApp().frame->main_grid->ShowRow(it->second->grid_index);
    wxGetApp().frame->selector_box->Check(it->second->selector_index, true);
  }
}

void DCFrame::OnPublishEvent(wxCommandEvent& event)
{
}

void DCFrame::OnEventPublished(wxThreadEvent& event)
{
}

DCRosNode::DCRosNode()
{
  // ROS Init
  ros::init(wxGetApp().argc, wxGetApp().argv, "DeCANstructor");
  node_handle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
  private_handle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
  spinner = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));
  DCOptions::one_day_ago = ros::Time::now() - ros::Duration(60 * 60 * 24);

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);

  // TODO: Get initial event mode and hide an event control
  // based on the mode.

  can_sub = node_handle->subscribe("can_in", 100, &DCRosNode::CanCallback, this);
  
  spinner->start();
}

DCRosNode::~DCRosNode()
{
  spinner->stop();
  ros::shutdown();
}

void DCRosNode::CanCallback(const can_msgs::Frame::ConstPtr& msg)
{
  std::lock_guard<std::mutex> callback_mut(wxGetApp().rcvd_msgs_mut);

  auto& msgs_local = wxGetApp().rcvd_msgs;
  auto found_msg = msgs_local.find(msg->id);
  // Do the following to get reasonable numbers to play with.
  uint64_t header_ms = (msg->header.stamp - DCOptions::one_day_ago).toNSec() / 1000000;

  if (found_msg != msgs_local.end())
  {
    // The message has already been received.
    // Store the old bytes for later comparison.
    std::copy(found_msg->second->bytes.begin(), found_msg->second->bytes.end(), found_msg->second->last_bytes.begin());

    for (uint8_t i = 0; i < msg->data.size(); i++)
    {
      if (msg->data[i] != found_msg->second->bytes[i])
      {
        found_msg->second->bytes[i] = msg->data[i];
        found_msg->second->last_updated_ms[i] = header_ms;
      }
    }

    found_msg->second->time_last_rcvd_ms = found_msg->second->time_rcvd_ms;
    found_msg->second->time_rcvd_ms = header_ms;
  }
  else
  {
    // This is a new message.
    std::shared_ptr<CanMsgDetail> new_msg = std::shared_ptr<CanMsgDetail>(new CanMsgDetail);
    new_msg->grid_index = -1;

    for (uint8_t i = 0; i < msg->data.size(); i++)
    {
      new_msg->bytes.push_back(msg->data[i]);
      new_msg->last_bytes.push_back(msg->data[i]);
      new_msg->last_updated_ms.push_back(header_ms);
    }

    new_msg->time_rcvd_ms = header_ms;
    new_msg->time_last_rcvd_ms = 0;
    msgs_local.insert(std::make_pair(msg->id, new_msg));
  }

  wxThreadEvent evt(wxEVT_CMD_UPDATE_MSGS);
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
