/*
 * Copyright (c) 2007 SCHUNK GmbH & Co. KG
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simplestirnglist_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSimpleStringList.

  \section sdhlibrary_cpp_simplestirnglist_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simplestirnglist_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
      \par SVN file revision:
        $Id: simplestringlist.h 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_simplestirnglist_h_changelog Changelog of this file:
      \include simplestringlist.h.log
*/
//======================================================================

#ifndef SIMPLESTRINGLIST_H_
#define SIMPLESTRINGLIST_H_

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <iosfwd>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------



//! A simple string list. (Fixed maximum number of strings of fixed maximum length)
class VCC_EXPORT cSimpleStringList
{
public:

  //! the index of the current line. For empty cSimpleStringLists this is -1.
  int current_line;


  //! anonymous enum instead of define macros
  enum
  {
    eMAX_LINES = 256,
    eMAX_CHARS = 256,
  };

  //! Default constructor: init members
  cSimpleStringList();


  //! Return the current line
  char* CurrentLine();


  //! Return the next line, this increases current_line
  char* NextLine();


  //! Return number of lines stored
  int Length() const;


  //! return ptr to line with index.
  /*!
    if index < 0 then the numbering starts from the end,
    thus [-1] gives the last line, [-2] the next to last, ...
  */
  char* operator[](int index);

  //! return ptr to line with index.
  /*!
    if index < 0 then the numbering starts from the end,
    thus [-1] gives the last line, [-2] the next to last, ...
  */
  char const* operator[](int index) const;


  //! reset list
  void Reset();

protected:
  //! a fixed length array of lines with fixed length
  char line[ eMAX_LINES ][ eMAX_CHARS ];

}; // cSimpleStringList
//-----------------------------------------------------------------


//! Output of cSimpleStringList objects in 'normal' output streams.
VCC_EXPORT std::ostream& operator<<(std::ostream& stream, cSimpleStringList const& ssl);


//-----------------------------------------------------------------

NAMESPACE_SDH_END

#endif


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================

