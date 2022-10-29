#ifndef DECANSTRUCTOR__MESSAGE_ANALYZER_HPP_
#define DECANSTRUCTOR__MESSAGE_ANALYZER_HPP_

#include <decanstructor/common.hpp>

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

#endif  // DECANSTRUCTOR__MESSAGE_ANALYZER_HPP_
