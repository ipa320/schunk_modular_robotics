/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: 
 * ROS stack name: 
 * ROS package name: 
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: 
 * Supervised by: 
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
#include <stdlib.h>
//#include <MathSup.h>
#include <cob_utilities/Mutex.h>


// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <diagnostic_msgs/DiagnosticStatus.h>

// ROS service includes
//--

// external includes
//#include <CANOpen_driver.h>
#include <CANOpenMaster.h>
#include <pthread.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	      
	ros::NodeHandle n;
	

	// TODO Namen der Klasse ändern
	// create Axis handle
	//CANOpenMaster* Axis; 
	
    // topics to publish       
	ros::Publisher topicPub_CANError;
	
	// topics to subscribe, callback is called for new messages arriving
        
    // service servers
    //--
        
    // service clients
    //--
    
    // global variables
    int SYNC_Freq_HZ;
    
    // msg containers  
	diagnostic_msgs::DiagnosticStatus msg_Error;
    

    // Constructor
    NodeClass()
    {
	// create a handle for this node, initialize node
	ros::NodeHandle private_nh_("~");

	// fill Axis handle
	//Axis = new CANOpenMaster();
	
	// global variables
	SYNC_Freq_HZ = 100; //Looprate for SYNC commands in Hz  

	// msg
	msg_Error.name = "CANError";

	// init topic publisher
	topicPub_CANError = n.advertise<diagnostic_msgs::DiagnosticStatus>("CANError",1);

	// init topic subscriber
		
    }
    

    // Destructor
    ~NodeClass() 
    {
		exit(0); 
    }

    // topic callback functions 
    // function will be called when a new message arrives on a topic
	
    // service callback functions
    // function will be called when a service is querried
    //--
 	
	// TODO callbacks for timingdemands

    // other function declarations
	
};

/*---------------------------------------------------------------*/
//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "CANOpenMaster");    
    NodeClass nodeClass;
	
	int err;

	InitMaster();	

	//StartMasterThread(100, nodeClass.Axis);

	// TODO init als funktion für API bereitstellen
	// init of axis
/*	err = nodeClass.Axis->Init();
	if (err!=0)
	{	ROS_INFO("Init of CAN-Master failed. Error: %i",err); return -1;}
	else
	{	ROS_INFO("Init of CAN-Master successfully."); }
	
	// TODO homing als funktion für API bereitstellen
	//homing
	err = nodeClass.Axis->Homing();  
	if (err!=0)
	{	ROS_INFO("Homing of axis failed. Code: %i", err); }
	else
	{	ROS_INFO("Homing of axis complete."); }
*/	
	int counter = 0; 

	// main loop
	ros::Rate loop_rate(100); // Hz
    while(nodeClass.n.ok())
    {	

    	//nodeClass.Axis->SendSYNC(); 
		//        motorcontroller stat | enalbe IP-mode     
		//nodeClass.Axis->WritePDO((0x011F),counter);
    	
		counter++;	
	    ros::spinOnce();
        loop_rate.sleep();
    }
	
	// terminate all threads 
	exit(0); 

    return 0;
}


