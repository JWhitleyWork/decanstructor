#ifndef SIGNAL_ANALYZER_H
#define SIGNAL_ANALYZER_H

#include <common.h>

namespace DeCANstructor
{
  class DCSignalAnalyzerFrame :
    public wxFrame
  {
    public:
      DCSignalAnalyzerFrame(wxWindow* parent,
                            const wxString& title,
                            const wxPoint& pos,
                            const wxSize& size);
  };
}

#endif
