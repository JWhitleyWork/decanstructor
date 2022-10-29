#include <decanstructor/message_analyzer.hpp>

using namespace DeCANstructor;

DCMessageAnalyzerFrame::DCMessageAnalyzerFrame(wxWindow* parent,
                                               const wxString& title,
                                               const wxPoint& pos,
                                               const wxSize& size) :
  wxFrame(parent,
          wxID_ANY,
          title,
          pos,
          size) 
{
}
