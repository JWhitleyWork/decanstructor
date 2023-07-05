// Copyright 2017-2023 Joshua Whitley
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <decanstructor/message_analyzer.hpp>

namespace DeCANstructor
{

DCMessageAnalyzerFrame::DCMessageAnalyzerFrame(
  wxWindow * parent, const wxString & title, const wxPoint & pos, const wxSize & size)
: wxFrame(parent, wxID_ANY, title, pos, size)
{
}

}  // namespace DeCANstructor
