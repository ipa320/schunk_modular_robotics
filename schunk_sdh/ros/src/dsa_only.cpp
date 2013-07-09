/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
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
 *   ROS package name: schunk_sdh
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Jan 2010
 *
 * \brief
 *   Implementation of ROS node for DSA.
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

//##################
//#### includes ####

// standard includes
#include <unistd.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <schunk_sdh/TactileSensor.h>
#include <schunk_sdh/TactileMatrix.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// ROS diagnostic msgs
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <schunk_sdh/dsa.h>

#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

template <typename T> bool read_vector(ros::NodeHandle &n_, const std::string &key, std::vector<T> & res){
    XmlRpc::XmlRpcValue namesXmlRpc;
    if (!n_.hasParam(key))
    {
	return false;
    }

    n_.getParam(key, namesXmlRpc);
    /// Resize and assign of values to the vector
    res.resize(namesXmlRpc.size());
    for (int i = 0; i < namesXmlRpc.size(); i++)
    {
    	res[i] = (T)namesXmlRpc[i];
    }
    return true;
}

/*!
* \brief Implementation of ROS node for DSA.
*
* Offers actionlib and direct command interface.
*/
class DsaNode
{
	public:
		/// create a handle for this node, initialize node
		ros::NodeHandle nh_;
	private:
		// declaration of topics to publish
		ros::Publisher topicPub_TactileSensor_;
		ros::Publisher topicPub_Diagnostics_;
		
		// topic subscribers

		// service servers

		// actionlib server

		// service clients
		//--

		// other variables
		SDH::cDSA *dsa_;
		SDH::UInt32 last_data_publish_; // time stamp of last data publishing

		std::string dsadevicestring_;
		int dsadevicenum_;
		int maxerror_; // maximum error count allowed

		bool isDSAInitialized_;
		int error_counter_;
		bool polling_; // try to publish on each response
		bool auto_publish_;
		bool use_rle_;
		bool debug_;
		double frequency_;
		
		
		ros::Timer timer_dsa,timer_publish, timer_diag;
		
		std::vector<int> dsa_reorder_;
	public:
		/*!
		* \brief Constructor for SdhNode class
		*
		* \param name Name for the actionlib server
		*/
		DsaNode():nh_("~"),dsa_(0),last_data_publish_(0),isDSAInitialized_(false),error_counter_(0)
		{
			topicPub_Diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
			topicPub_TactileSensor_ = nh_.advertise<schunk_sdh::TactileSensor>("tactile_data", 1);
		}

		/*!
		* \brief Destructor for SdhNode class
		*/
		~DsaNode() 
		{
			if(isDSAInitialized_)
				dsa_->Close();
			if(dsa_)
				delete dsa_;
		}

		void shutdown(){
		    timer_dsa.stop();
		    timer_publish.stop();
		    timer_diag.stop();
		    nh_.shutdown();
		}


		/*!
		* \brief Initializes node to get parameters, subscribe and publish to topics.
		*/
		bool init()
		{
			// implementation of topics to publish
 
			nh_.param("dsadevicestring", dsadevicestring_, std::string(""));
			if (dsadevicestring_.empty()) return false;

			nh_.param("dsadevicenum", dsadevicenum_, 0);
			nh_.param("maxerror", maxerror_, 8);
			
 			double publish_frequency, diag_frequency;
			
			nh_.param("debug", debug_, false);
			nh_.param("polling", polling_, false);
			nh_.param("use_rle", use_rle_, true);
			nh_.param("diag_frequency", diag_frequency, 5.0);
			frequency_ = 30.0;
			if(polling_) nh_.param("poll_frequency", frequency_, 5.0);
			nh_.param("publish_frequency", publish_frequency, 0.0);
			
			auto_publish_ = true;

				
			if(polling_){
			    timer_dsa = nh_.createTimer(ros::Rate(frequency_).expectedCycleTime(),boost::bind(&DsaNode::pollDsa,  this));
 			}else{
			    timer_dsa = nh_.createTimer(ros::Rate(frequency_*2.0).expectedCycleTime(),boost::bind(&DsaNode::readDsaFrame,  this));
			    if(publish_frequency > 0.0){
				auto_publish_ = false;
				timer_publish = nh_.createTimer(ros::Rate(publish_frequency).expectedCycleTime(),boost::bind(&DsaNode::publishTactileData, this));
			    }
			}

			timer_diag = nh_.createTimer(ros::Rate(diag_frequency).expectedCycleTime(),boost::bind(&DsaNode::publishDiagnostics, this));
			
			if(!read_vector(nh_, "dsa_reorder", dsa_reorder_)){
			    dsa_reorder_.resize(6);
			    dsa_reorder_[0] = 2; // t1
			    dsa_reorder_[1] = 3; // t2
			    dsa_reorder_[2] = 4; // f11
			    dsa_reorder_[3] = 5; // f12
			    dsa_reorder_[4] = 0; // f21
			    dsa_reorder_[5] = 1; // f22
			}
			
			return true;
		}
		bool stop(){
			if(dsa_){
			    if(isDSAInitialized_)
				dsa_->Close();
			    delete dsa_;
			}
			dsa_ = 0;
			isDSAInitialized_ = false;
			return true;
		}

		bool start()
		{

			if (isDSAInitialized_ == false)
			{
				//Init tactile data
				if(!dsadevicestring_.empty())  {
					try
					{
						dsa_ = new SDH::cDSA(0, dsadevicenum_, dsadevicestring_.c_str());
						if(!polling_)
						    dsa_->SetFramerate( frequency_, use_rle_ );
                        			else
                            			    dsa_->SetFramerate( 0, use_rle_ );
						
						ROS_INFO("Initialized RS232 for DSA Tactile Sensors on device %s",dsadevicestring_.c_str());
						// ROS_INFO("Set sensitivity to 1.0");
						// for(int i=0; i<6; i++)
						// 	dsa_->SetMatrixSensitivity(i, 1.0);
						error_counter_ = 0;
						isDSAInitialized_ = true;
					}
					catch (SDH::cSDHLibraryException* e)
					{
						isDSAInitialized_ = false;
						ROS_ERROR("An exception was caught: %s", e->what());
						delete e;
						
						shutdown();
						return false;
					}
				}
			}
			
			return true;
		}

	void readDsaFrame()
	{
		if(debug_) ROS_DEBUG("readDsaFrame");

		if(isDSAInitialized_)
		{
			try
			{
				SDH::UInt32 last_time;
				last_time = dsa_->GetFrame().timestamp;
				dsa_->UpdateFrame();
				if(last_time != dsa_->GetFrame().timestamp){ // new data
				    if(error_counter_ > 0) --error_counter_;
				    if(auto_publish_) publishTactileData();
				}
				
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
				++error_counter_;
			}
			if(error_counter_ > maxerror_) stop();

		}else{
		    start();
		}
	}
	
	void pollDsa(){
		if(debug_) ROS_DEBUG("pollDsa");

		if(isDSAInitialized_)
		{
			try
			{
				dsa_->SetFramerate( 0, use_rle_ );
				readDsaFrame();
				
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
				++error_counter_;
			}
			if(error_counter_ > maxerror_) stop();

		}else{
		    start();
		}
	}
	
	void publishTactileData()
	{
	    if(debug_) ROS_DEBUG("publishTactileData %ul %ul",dsa_->GetFrame().timestamp, last_data_publish_);
	    if(!isDSAInitialized_ || dsa_->GetFrame().timestamp == last_data_publish_) return; // no new frame available
	    last_data_publish_ = dsa_->GetFrame().timestamp;
	    
	    schunk_sdh::TactileSensor msg;
	    msg.header.stamp = ros::Time::now();
	    int m, x, y;
	    msg.tactile_matrix.resize(dsa_->GetSensorInfo().nb_matrices);
	    ROS_ASSERT(dsa_->GetSensorInfo().nb_matrices == dsa_reorder_.size());
	    for ( unsigned int i = 0; i < dsa_reorder_.size(); i++ )
	    {
		    m = dsa_reorder_[i];                                  
		    schunk_sdh::TactileMatrix &tm = msg.tactile_matrix[i];
		    tm.matrix_id = i;
		    tm.cells_x = dsa_->GetMatrixInfo( m ).cells_x;
		    tm.cells_y = dsa_->GetMatrixInfo( m ).cells_y;
		    tm.tactile_array.resize(tm.cells_x * tm.cells_y);
		    for ( y = 0; y < tm.cells_y; y++ )
		    {
			    for ( x = 0; x < tm.cells_x; x++ )
				    tm.tactile_array[tm.cells_x*y + x] = dsa_->GetTexel( m, x, y );
		    }
	    }
	    //publish matrix
	    topicPub_TactileSensor_.publish(msg);
	    
	}
	void publishDiagnostics()
	{
	    // publishing diagnotic messages
	    diagnostic_msgs::DiagnosticArray diagnostics;
	    diagnostics.status.resize(1);
	    diagnostics.status[0].name = nh_.getNamespace();
	    diagnostics.status[0].values.resize(1);
	    diagnostics.status[0].values[0].key = "error_count";
	    diagnostics.status[0].values[0].value = boost::lexical_cast<std::string>( error_counter_);

	    // set data to diagnostics
	    if (isDSAInitialized_)
	    {
		diagnostics.status[0].level = 0;
		diagnostics.status[0].message = "DSA tactile sensing initialized and running";
	    }
	    else if(error_counter_ == 0)
	    {
		diagnostics.status[0].level = 1;
		diagnostics.status[0].message = "DSA not initialized";
	    }
	    else
	    {
		diagnostics.status[0].level = 2;
		diagnostics.status[0].message = "DSA exceeded eror count";
	    }
	    // publish diagnostic message
	    topicPub_Diagnostics_.publish(diagnostics);
	    if(debug_) ROS_DEBUG_STREAM("publishDiagnostics " << diagnostics);

	}	 
}; //DsaNode

/*!
* \brief Main loop of ROS node.
*
* Running with a specific frequency defined by loop_rate.
*/
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "schunk_dsa");
	
	DsaNode dsa_node;
	
	if (!dsa_node.init()) return 0;
	
	dsa_node.start();
	
	ROS_INFO("...dsa node running...");

	ros::spin();
	
	ROS_INFO("...dsa node shut down...");
	return 0;
}

