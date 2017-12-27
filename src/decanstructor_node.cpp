#include <decanstructor_node.h>

using namespace DeCANstructor;

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
  menu_file->Append(ID_Hello, "&Hello...\tCtrl-H", "Help string shown in status bar for this menu item");
  menu_file->AppendSeparator();
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
  wxFlexGridSizer* main_sizer = new wxFlexGridSizer(1, 2, 5, 5);

  // Create the main grid
  active_grid = std::shared_ptr<wxGrid>(new wxGrid(this, -1, wxPoint(0, 0), wxSize(100, 350)));

  active_grid->CreateGrid(1, 10);
  active_grid->EnableEditing(false);
  active_grid->SetColLabelSize(wxGRID_AUTOSIZE);
  active_grid->SetRowLabelSize(0);

  active_grid->SetColLabelValue(0, "ID");
  active_grid->SetColLabelValue(1, "0");
  active_grid->SetColLabelValue(2, "1");
  active_grid->SetColLabelValue(3, "2");
  active_grid->SetColLabelValue(4, "3");
  active_grid->SetColLabelValue(5, "4");
  active_grid->SetColLabelValue(6, "5");
  active_grid->SetColLabelValue(7, "6");
  active_grid->SetColLabelValue(8, "7");
  active_grid->SetColLabelValue(9, "Last Rcvd");

  main_sizer->Add(active_grid.get());

  SetSizerAndFit(main_sizer);
}

void DCFrame::OnExit(wxCommandEvent& event)
{
  Close(true);
}

void DCFrame::OnAbout(wxCommandEvent& event)
{
	wxMessageBox("Copyright 2017 Joshua Whitley, All Rights Reserved", "About DeCANstructor", wxOK | wxICON_INFORMATION );
}

void DCFrame::OnHello(wxCommandEvent& event)
{
  wxLogMessage("Hey.");
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
  wxGetApp().frame->PushStatusText("We got one!");
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
