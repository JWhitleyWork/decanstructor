#include <decanstructor_node.h>

using namespace DeCANstructor;

void DCGridCellHexRenderer::Draw(wxGrid& grid,
                                 wxGridCellAttr& attr,
                                 wxDC& dc,
                                 const wxRect& rectCell,
                                 int row, int col,
                                 bool isSelected)
{
  wxRect rect = rectCell;
  rect.Inflate(-1);

  // erase only this cells background, overflow cells should have been erased
  wxGridCellRenderer::Draw(grid, attr, dc, rectCell, row, col, isSelected);

  int hAlign, vAlign;

  attr.GetAlignment(&hAlign, &vAlign);

  int overflowCols = 0;

  if (attr.GetOverflow())
  {
    int cols = grid.GetNumberCols();
    int best_width = GetBestSize(grid,attr,dc,row,col).GetWidth();
    int cell_rows, cell_cols;

    attr.GetSize( &cell_rows, &cell_cols ); // shouldn't get here if <= 0

    if ((best_width > rectCell.width) && (col < cols) && grid.GetTable())
    {
      int i, c_cols, c_rows;

      for (i = col+cell_cols; i < cols; i++)
      {
        bool is_empty = true;

        for (int j=row; j < row + cell_rows; j++)
        {
          // check w/ anchor cell for multicell block
          grid.GetCellSize(j, i, &c_rows, &c_cols);

          if (c_rows > 0)
            c_rows = 0;

          if (!grid.GetTable()->IsEmptyCell(j + c_rows, i))
          {
            is_empty = false;
            break;
          }
        }

        if (is_empty)
        {
          rect.width += grid.GetColSize(i);
        }
        else
        {
          i--;
          break;
        }

        if (rect.width >= best_width)
          break;
      }

      overflowCols = i - col - cell_cols + 1;

      if (overflowCols >= cols)
        overflowCols = cols - 1;
    }

    if (overflowCols > 0) // redraw overflow cells w/ proper hilight
    {
      hAlign = wxALIGN_LEFT; // if oveflowed then it's left aligned
      wxRect clip = rect;
      clip.x += rectCell.width;

      // draw each overflow cell individually
      int col_end = col + cell_cols + overflowCols;

      if (col_end >= grid.GetNumberCols())
        col_end = grid.GetNumberCols() - 1;

      for (int i = col + cell_cols; i <= col_end; i++)
      {
        clip.width = grid.GetColSize(i) - 1;
        dc.DestroyClippingRegion();
        dc.SetClippingRegion(clip);

        SetTextColoursAndFont(grid, attr, dc,
                              grid.IsInSelection(row,i));

        grid.DrawTextRectangle(dc, grid.GetCellValue(row, col),
                               rect, hAlign, vAlign);
        clip.x += grid.GetColSize(i) - 1;
      }

      rect = rectCell;
      rect.Inflate(-1);
      rect.width++;

      dc.DestroyClippingRegion();
    }
  }

  // now we only have to draw the text
  SetTextColoursAndFont(grid, attr, dc, isSelected);

  grid.DrawTextRectangle(dc, GetString(grid, row, col),
                         rect, hAlign, vAlign);
}

wxString DCGridCellHexRenderer::GetString(const wxGrid& grid, int row, int col)
{
  wxGridTableBase* table = grid.GetTable();
  wxString text;

  if (table->CanGetValueAs(row, col, wxGRID_VALUE_NUMBER))
    text.Printf(wxT("%X"), table->GetValueAsLong(row, col));
  else
    text = table->GetValue(row, col);

  return text;
}

wxString DCGridCellPrefixedHexRenderer::GetString(const wxGrid& grid, int row, int col)
{
  wxGridTableBase* table = grid.GetTable();
  wxString text;

  if (table->CanGetValueAs(row, col, wxGRID_VALUE_NUMBER))
    text.Printf(wxT("%#X"), table->GetValueAsLong(row, col));
  else
    text = table->GetValue(row, col);

  return text;
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

  active_grid->CreateGrid(1, 10);
  active_grid->EnableEditing(false);
  active_grid->DisableDragColSize();
  active_grid->DisableDragRowSize();
  active_grid->DisableDragGridSize();
  active_grid->SetSelectionMode(wxGrid::wxGridSelectRowsOrColumns);
  active_grid->SetColLabelSize(wxGRID_AUTOSIZE);
  active_grid->SetRowLabelSize(40);

  DCGridCellHexRenderer* hex_renderer = new DCGridCellHexRenderer();
  active_grid->SetDefaultRenderer(hex_renderer);

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
}

void DCFrame::OnExit(wxCommandEvent& event)
{
  Close(true);
}

void DCFrame::OnAbout(wxCommandEvent& event)
{
	wxMessageBox("Copyright 2017 Joshua Whitley, All Rights Reserved", "About DeCANstructor", wxOK | wxICON_INFORMATION );
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
