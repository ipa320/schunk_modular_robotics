/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: schunk_modular_robotics
 * \note
 *   ROS stack name: schunk_modular_robotics
 * \note
 *   ROS package name: schunk_powercube_chain
 *
 * \author
 *   Author: Tim Fröhlich, email:tim.froehlich@ipa.fhg.de
 * \author
 *   Supervised by: Tim Fröhlich, email:tim.froehlich@ipa.fhg.de
 *
 * \date Date of creation: Aug 2012
 *
 * \brief
 *   Implementation of a diagnostic class.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __DIAGNOSTICS_H_
#define __DIAGNOSTICS_H_

#include <chrono>
#include <deque>

/*!
 * \brief ErrorcodeReport (helper-)class for status reporting of common libraries 
 *
 * This class holds the error code and an error code namespace of a reported error. 
 * It is also a Code_Exists flag provided, which is default false and indicates that 
 * there is no error code for this message (e.g. if a hand written hind is reported)
 * The error code namespace is used to lookup an error is there are multiple 
 * error code LUTs, as it is common in larger libraries. 
 */
class ErrorcodeReport						// Errorcode from library
{	
	public:
	bool Code_Exists; 						// true when errorcode returned 
	int ErrorCode;							// code number				
	std::string Errorcode_Namespace;		// namespace of the code (for lookup) 
	
	/// Constructor
	ErrorcodeReport():
		Code_Exists(false), ErrorCode(0), Errorcode_Namespace("") {};			
};

/*!
 * \brief DiagnosticStatus (helper-)class for status reporting of common libraries 
 *
 * This class holds a error level, an error message, a error recommendation, a time stamp 
 * and a ErrorcodeReport object. The level's default value is 0 (=OK), Message and 
 * Recommendation strings are empty. Time is set to now() during initializing. 
 */
class DiagnosticStatus 
{	
	public:
	short Level; 							// 0 = OK, 1 = WARN, 2=ERROR		
	std::string Message; 					// Description of the error
	std::string Recommendation;				// Possible solutions for the user
	ErrorcodeReport Errorcode_Report; 		// Errorcode that is returned from a function

	std::chrono::time_point<std::chrono::system_clock> Time; 	// time when status is reported
	
	/// Constructor
	DiagnosticStatus():
		Level(0), Message(""), Recommendation(""), Time(std::chrono::system_clock::now()) {};
};

/*!
 * \brief Diagnostics class for status reporting of common libraries 
 *
 * This class is ment to provide a easy and short way to report the status 
 * (Errors and non-Errors) to the library itself and to ROS. It is designed close 
 * to the diagnostics msg topic to be easly wrapped. 
 */
class Diagnosics
{

	/*TODO:	- select format for errorlookups (.h, .txt, yaml) 
	 * 		- implement report status 
	 * 		- implement read methodes
	 * 		- implement QueLength methodes
	 */
public:

	/// Constructor
	Diagnosics()
	{	
		m_StatusQueLength = 5; 		// 5 is default value

		// initialize member variabeles
		m_StatusList = new std::deque<DiagnosticStatus>[m_StatusQueLength];
		
		// initialize default ok return value (default 0) 		
		m_Default_Ok_Value = 0; 

		//TODO: read in errorcode lists 
		
	}

	/// Destructor
	~Diagnosics()
	{
		delete m_StatusList; 
	};
	
	/*******************************************|
	| 			Useage interface				|
	|*******************************************/

	/// Report a Status
	
	/*!
 	 * \brief for simple status report by hand
	 */
	void ReportStatus(short Level, std::string Message)
	{	
		DiagnosticStatus NewStatus; 
		
		NewStatus.Level = Level; 
		NewStatus.Time = std::chrono::system_clock::now(); 		
		NewStatus.Message = Message;
		
		m_StatusList->push_front(NewStatus); 
		m_StatusList->pop_back();
	}
	
	/*!
 	 * \brief for simple status report by hand with problem solution 
	 */
	void ReportStatus(short Level, std::string Message, std::string Recommendation)
	{
		DiagnosticStatus NewStatus; 
		
		NewStatus.Level = Level; 
		NewStatus.Time = std::chrono::system_clock::now(); 		
		NewStatus.Message = Message;
		NewStatus.Message = Recommendation;
		
		m_StatusList->push_front(NewStatus); 
		m_StatusList->pop_back();
	}

	/*!
 	 * \brief report with manually set level for a errorcode and additional comments 
	 */		
	void ReportStatus(short Level, int Errorcode, std::string Errorcode_Namespace, std::string Recommendation)
	{
		DiagnosticStatus NewStatus; 
		
		NewStatus.Level = Level; 
		NewStatus.Time = std::chrono::system_clock::now(); 
		
		//TODO: generate message from errorcode

		NewStatus.Message = Recommendation;

		m_StatusList->push_front(NewStatus); 
		m_StatusList->pop_back();
	}
	
	/*!
 	 * \brief automatic report of status by errorcode. Can be used for return values of library functions. 
	 */	
	void ReportStatus(int Errorcode, std::string Errorcode_Namespace)
	{
		DiagnosticStatus NewStatus; 
		
		//TODO: get message from errorcodelist

		NewStatus.Time = std::chrono::system_clock::now(); 

		m_StatusList->push_front(NewStatus); 
		m_StatusList->pop_back();
	}

	
	/// Read the status

	/*!
 	 * \brief Outputs the actual status to the passed pointers. 
	 * 
	 * string arguments are resized for the message
	 */
	void ReadActualStatus(short* Level, std::string* Message, std::string* Recommendation)
	{
		DiagnosticStatus ActualStatus = m_StatusList->at(0);
		
		// resize strings for the size of the upcoming message 
		Message->clear();
		Recommendation->clear(); 
		
		// write output
		Level = (short*) ActualStatus.Level;	
		Message->append(ActualStatus.Message);
		Recommendation->append(ActualStatus.Recommendation);
	}

	/*!
 	 * \brief Retuns the actual status level.
	 */
	int ReadActualStatusLevel()
	{	
		DiagnosticStatus ActualStatus = m_StatusList->at(0);
		return ActualStatus.Level; 
	}

	/*!
 	 * \brief Retuns the actual status message.
	 */
	std::string ReadActualStatusMessage()
	{
		DiagnosticStatus ActualStatus = m_StatusList->at(0);
		return ActualStatus.Message;
	}

	/*!
 	 * \brief Retuns the actual status recommendation.
	 */
	std::string ReadActualStatusRecommendation()
	{
		DiagnosticStatus ActualStatus = m_StatusList->at(0);
		return ActualStatus.Recommendation;
	}

	/*******************************************|
	| 			Configurations					|
	|*******************************************/
 
	/// status que length
	
	/*!
 	 * \brief returns the Status que length right now
	 */
	int GetActualStatusQueLength()
	{
		return m_StatusQueLength; 
	}

	/*!
 	 * \brief Sets the maximal length of status history that can be read. Default is 5 elements. 
	 */
	void SetMaxStatusQueLength(int StatusQueLength)
	{
		m_StatusQueLength = StatusQueLength; 	
	}

	/*!
 	 * \brief Gets the maximal length of status history that can be read. 
	 */
	int GetMaxStatusQueLength()
	{
		return m_StatusQueLength;
	}


	/// Default ok value read/write
	
	/*!
 	 * \brief returns the actual default ok value  
	 */
	int GetDefaultOKValue()
	{
		return m_Default_Ok_Value = 0;
	}

	/*!
 	 * \brief sets a new default ok value  
	 */
	void SetDefaultOKValue(int Default_Ok_Value)
	{
		m_Default_Ok_Value = Default_Ok_Value;
	}

private:
	
	std::deque<DiagnosticStatus> *m_StatusList;	// List that holds the last status messages in a deque

	int m_StatusQueLength; 						// maximal length of status que
	int m_Default_Ok_Value;						// is used for check if return value needs to
												// be reported
	
};

#endif
