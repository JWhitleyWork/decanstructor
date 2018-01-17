#include <signal_analyzer.h>

using namespace DeCANstructor;

DCSignalAnalyzerFrame::DCSignalAnalyzerFrame(wxWindow* parent,
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
