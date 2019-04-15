//======================================================================//
//  This software is free: you can redistribute it and/or modify        //
//  it under the terms of the GNU General Public License Version 3,     //
//  as published by the Free Software Foundation.                       //
//  This software is distributed in the hope that it will be useful,    //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
//  GNU General Public License for more details.                        //
//  You should have received a copy of the GNU General Public License   //
//  Version 3 in the file COPYING that came with this distribution.     //
//  If not, see <http://www.gnu.org/licenses/>                          //
//======================================================================//
//                                                                      //
//      Copyright (c) 2019 SinfonIA Pepper RoboCup Team                 //
//      Sinfonia - Colombia                                             //
//      https://sinfoniateam.github.io/sinfonia/index.html              //
//                                                                      //
//======================================================================//


#ifndef JOINT_STATE_CONVERTER_HPP
#define JOINT_STATE_CONVERTER_HPP


#include "converter_base.hpp"
#include "../tools/robot_description.hpp"
#include "robot_toolkit/message_actions.h"


#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace Sinfonia
{
    namespace Converter
    {

	class JointStateConverter : public BaseConverter<JointStateConverter>
	{

	typedef boost::function<void(sensor_msgs::JointState&, std::vector<geometry_msgs::TransformStamped>&) > callbackT;

	typedef boost::shared_ptr<tf2_ros::Buffer> bufferPtr;

	typedef std::map<std::string, boost::shared_ptr<urdf::JointMimic> > mimicMap;

	public:
	    JointStateConverter( const std::string& name, const float& frequency, const bufferPtr& tf2_buffer, const qi::SessionPtr& session );

	    ~JointStateConverter();

	    virtual void reset( );

	    void registerCallback( const MessageAction::MessageAction action, callbackT callBack );

	    void callAll( const std::vector<MessageAction::MessageAction>& actions );

	private:

	    
	    void addChildren(const KDL::SegmentMap::const_iterator segment);
	    std::map<std::string, robot_state_publisher::SegmentPair> segments_, segmentsFixed_;
	    void setTransforms(const std::map<std::string, double>& jointPositions, const ros::Time& time, const std::string& tfPrefix);
	    void setFixedTransforms(const std::string& tfPrefix, const ros::Time& time);

	    bufferPtr tf2Buffer_;
  
	    qi::AnyObject pMotion_;

	    std::map<MessageAction::MessageAction, callbackT> callbacks_;
	    
	    std::string robotDesc_;
	    
	    mimicMap mimic_;
	    
	    sensor_msgs::JointState msgJointStates_;
	    
	    std::vector<geometry_msgs::TransformStamped> tfTransforms_;

	};

    } 
} 

#endif
