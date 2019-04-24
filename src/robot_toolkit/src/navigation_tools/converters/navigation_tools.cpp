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




#include "navigation_tools.hpp"

#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH


#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Sinfonia
{

    namespace Converter
    {

	NavigationToolsConverter::NavigationToolsConverter( const std::string& name, const float& frequency, const bufferPtr& tf2Buffer, const qi::SessionPtr& session ):
	BaseConverter( name, frequency, session )
	{
	    _pMotion =  session->service("ALMotion");
	    _tf2Buffer = tf2Buffer;
	    _robotDesc = Tools::getRobotDescription();
    
	}

	NavigationToolsConverter::~NavigationToolsConverter()
	{
	}

	void NavigationToolsConverter::reset()
	{
	    if ( _robotDesc.empty() )
	    {
		std::cout << "error in loading robot description" << std::endl;
		return;
	    }
	    
	    urdf::Model model;
	    model.initString( _robotDesc );
	    KDL::Tree tree;
	    kdl_parser::treeFromUrdfModel( model, tree );

	    addChildren( tree.getRootSegment() );

	    _mimic.clear();
	    for(std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
	    {
		if(i->second->mimic)
		{
		    _mimic.insert(make_pair(i->first, i->second->mimic));
		}
	    }
	    
	    _msgJointStates.name = _pMotion.call<std::vector<std::string> >("getBodyNames", "Body" );
	}

	void NavigationToolsConverter::registerCallback( const MessageAction::MessageAction action, callbackT callBack )
	{
	    _callbacks[action] = callBack;
	}

	void NavigationToolsConverter::callAll( const std::vector<MessageAction::MessageAction>& actions )
	{
	    callTF();
	    for_each( MessageAction::MessageAction action, actions )
	    {
		_callbacks[action](_tfTransforms);
	    }
	}


	// Copied from robot state publisher
	void NavigationToolsConverter::setTransforms(const std::map<std::string, double>& jointPositions, const ros::Time& time, const std::string& tfPrefix)
	{
	    geometry_msgs::TransformStamped tfTransform;
	    tfTransform.header.stamp = time;

	    // loop over all joints
	    for (std::map<std::string, double>::const_iterator jnt=jointPositions.begin(); jnt != jointPositions.end(); jnt++)
	    {
		std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg = _segments.find(jnt->first);
		if (seg != _segments.end()){
		seg->second.segment.pose(jnt->second).M.GetQuaternion(tfTransform.transform.rotation.x,
									tfTransform.transform.rotation.y,
									tfTransform.transform.rotation.z,
									tfTransform.transform.rotation.w);
		tfTransform.transform.translation.x = seg->second.segment.pose(jnt->second).p.x();
		tfTransform.transform.translation.y = seg->second.segment.pose(jnt->second).p.y();
		tfTransform.transform.translation.z = seg->second.segment.pose(jnt->second).p.z();

		tfTransform.header.frame_id = seg->second.root;
		tfTransform.child_frame_id = seg->second.tip;

		_tfTransforms.push_back(tfTransform);

		if (_tf2Buffer)
		    _tf2Buffer->setTransform(tfTransform, "naoqiconverter", false);
		}
	    }

	}

	// Copied from robot state publisher
	void NavigationToolsConverter::setFixedTransforms(const std::string& tfPrefix, const ros::Time& time)
	{
	    geometry_msgs::TransformStamped tfTransform;
	    tfTransform.header.stamp = time/*+ros::Duration(0.5)*/;  // future publish by 0.5 seconds

	    // loop over all fixed segments
	    for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg=_segmentsFixed.begin(); seg != _segmentsFixed.end(); seg++)
	    {
		seg->second.segment.pose(0).M.GetQuaternion(tfTransform.transform.rotation.x,
							    tfTransform.transform.rotation.y,
							    tfTransform.transform.rotation.z,
							    tfTransform.transform.rotation.w);
		tfTransform.transform.translation.x = seg->second.segment.pose(0).p.x();
		tfTransform.transform.translation.y = seg->second.segment.pose(0).p.y();
		tfTransform.transform.translation.z = seg->second.segment.pose(0).p.z();
		tfTransform.header.frame_id = seg->second.root;
		tfTransform.child_frame_id = seg->second.tip;

		_tfTransforms.push_back(tfTransform);

		if (_tf2Buffer)
		_tf2Buffer->setTransform(tfTransform, "naoqiconverter", true);
	    }
	 }

	void NavigationToolsConverter::addChildren(const KDL::SegmentMap::const_iterator segment)
	{
	    const std::string& root = GetTreeElementSegment(segment->second).getName();

	    const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
	    for (unsigned int i=0; i<children.size(); i++)
	    {
		const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
		robot_state_publisher::SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
		if (child.getJoint().getType() == KDL::Joint::None)
		{
		    _segmentsFixed.insert(std::make_pair(child.getJoint().getName(), s));
		    ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
		}
		else
		{
		    _segments.insert(std::make_pair(child.getJoint().getName(), s));
		    ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
		}
		addChildren(children[i]);
	    }
	}
	
	void NavigationToolsConverter::callTF()
	{
 std::vector<double> alJointAngles = _pMotion.call<std::vector<double> >("getAngles", "Body", true );
	    const ros::Time& stamp = ros::Time::now();
	    
	    _msgJointStates.header.stamp = stamp;

	    _msgJointStates.position = std::vector<double>( alJointAngles.begin(), alJointAngles.end() );

	    std::map< std::string, double > jointStateMap;
	    
	    std::vector<double>::const_iterator itPos = _msgJointStates.position.begin();
	    for(std::vector<std::string>::const_iterator itName = _msgJointStates.name.begin(); itName != _msgJointStates.name.end(); ++itName, ++itPos)
	    {
		jointStateMap[*itName] = *itPos;
	    }

	    // for mimic map
	    for(mimicMap::iterator i = _mimic.begin(); i != _mimic.end(); i++)
	    {
		if(jointStateMap.find(i->second->joint_name) != jointStateMap.end())
		{
		    double pos = jointStateMap[i->second->joint_name] * i->second->multiplier + i->second->offset;
		    jointStateMap[i->first] = pos;
		}
	    }

	    // reset the transforms we want to use at this time
	    _tfTransforms.clear();
	    static const std::string& jtTfPrefix = "";
	    setTransforms(jointStateMap, stamp, jtTfPrefix);
	    setFixedTransforms(jtTfPrefix, stamp);


	    std::vector<float> alOdometryData = _pMotion.call<std::vector<float> >( "getPosition", "Torso", 1, true );
	    const ros::Time& odomStamp = ros::Time::now();
	    const float& odomX  =  alOdometryData[0];
	    const float& odomY  =  alOdometryData[1];
	    const float& odomZ  =  alOdometryData[2];
	    const float& odomWX =  alOdometryData[3];
	    const float& odomWY =  alOdometryData[4];
	    const float& odomWZ =  alOdometryData[5];
	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    tf2::Quaternion tfQuaternion;
	    tfQuaternion.setRPY( odomWX, odomWY, odomWZ );
	    geometry_msgs::Quaternion odomQuaternion = tf2::toMsg( tfQuaternion );

	    static geometry_msgs::TransformStamped msgTfOdom;
	    msgTfOdom.header.frame_id = "odom";
	    msgTfOdom.child_frame_id = "base_link";
	    msgTfOdom.header.stamp = odomStamp;

	    msgTfOdom.transform.translation.x = odomX;
	    msgTfOdom.transform.translation.y = odomY;
	    msgTfOdom.transform.translation.z = odomZ;
	    msgTfOdom.transform.rotation = odomQuaternion;

	    _tfTransforms.push_back( msgTfOdom );
	    _tf2Buffer->setTransform( msgTfOdom, "naoqiconverter", false);

	    // If nobody uses that buffer, do not fill it next time
	    if (( _tf2Buffer ) && ( _tf2Buffer.use_count() == 1 ))
	    {
		_tf2Buffer.reset();
	    }
	}
    }
} 
