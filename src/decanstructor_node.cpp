#include <decanstructor_node.h>

using namespace DeCANstructor;

bool DCNode::OnInit()
{
  DCFrame* frame = new DCFrame("DeCANstructor", wxPoint(50, 50), wxSize(450, 340));
  frame->Show(true);
  return true;
}

DCFrame::DCFrame(const wxString& title,
                 const wxPoint& pos,
                 const wxSize& size) :
  wxFrame(NULL, wxID_ANY, title, pos, size)
{
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
