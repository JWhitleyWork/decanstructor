// Copyright 2017-2023 Joshua Whitley
// 
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
