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


#ifndef NAVIGATION_TOOLS_CONVERTER_HPP
#define NAVIGATION_TOOLS_CONVERTER_HPP


#include "../../converters/converter_base.hpp"
#include "../../tools/robot_description.hpp"
#include "robot_toolkit/message_actions.h"


#include <urdf/model.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>
#include <robot_state_publisher/robot_state_publisher.h>

namespace Sinfonia
{
    namespace Converter
    {

	class NavigationToolsConverter : public BaseConverter<NavigationToolsConverter>
	{

	typedef boost::function<void(std::vector<geometry_msgs::TransformStamped>&, nav_msgs::Odometry) > callbackT;

	typedef boost::shared_ptr<tf2_ros::Buffer> bufferPtr;

	typedef std::map<std::string, boost::shared_ptr<urdf::JointMimic> > mimicMap;

	public:
	    NavigationToolsConverter( const std::string& name, const float& frequency, const bufferPtr& tf2Buffer, const qi::SessionPtr& session );
	    ~NavigationToolsConverter();

	    virtual void reset( );
	    void registerCallback( const MessageAction::MessageAction action, callbackT callBack );
	    void callAll( const std::vector<MessageAction::MessageAction>& actions );

	private:    
	    void setTransforms(const std::map<std::string, double>& jointPositions, const ros::Time& time, const std::string& tfPrefix);
	    void setFixedTransforms(const std::string& tfPrefix, const ros::Time& time);
	    void addChildren(const KDL::SegmentMap::const_iterator segment);
	    void callTF();
	    void callOdom();
	    
	    bufferPtr _tf2Buffer;
  
	    qi::AnyObject _pMotion;

	    std::map<MessageAction::MessageAction, callbackT> _callbacks;
	    std::map<std::string, robot_state_publisher::SegmentPair> _segments, _segmentsFixed;
	    
	    std::string _robotDesc;
	    
	    mimicMap _mimic;
	    
	    sensor_msgs::JointState _msgJointStates;
	    
	    std::vector<geometry_msgs::TransformStamped> _tfTransforms;
	    nav_msgs::Odometry _msgOdom;

	};

    } 
} 

#endif
