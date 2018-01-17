#include <message_analyzer.h>

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
