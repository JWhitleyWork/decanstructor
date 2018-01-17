#ifndef MESSAGE_ANALYZER_H
#define MESSAGE_ANALYZER_H

#include <common.h>

namespace DeCANstructor
{
  class DCMessageAnalyzerFrame :
    public wxFrame
  {
    public:
      DCMessageAnalyzerFrame(wxWindow* parent,
                             const wxString& title,
                             const wxPoint& pos,
                             const wxSize& size);
  };
}

#endif
