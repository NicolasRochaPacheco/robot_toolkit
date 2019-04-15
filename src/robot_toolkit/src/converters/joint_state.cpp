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




#include "joint_state.hpp"
#include "nao_footprint.hpp"


#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH


#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Sinfonia
{

    namespace Converter
    {

	JointStateConverter::JointStateConverter( const std::string& name, const float& frequency, const bufferPtr& tf2_buffer, const qi::SessionPtr& session ):
	BaseConverter( name, frequency, session ),
	pMotion_( session->service("ALMotion") ),
	tf2Buffer_(tf2_buffer)
	{
	    robotDesc_ = Tools::getRobotDescription(robot_);
	}

	JointStateConverter::~JointStateConverter()
	{
	}

	void JointStateConverter::reset()
	{
	    if ( robotDesc_.empty() )
	    {
		std::cout << "error in loading robot description" << std::endl;
		return;
	    }
	    
	    urdf::Model model;
	    model.initString( robotDesc_ );
	    KDL::Tree tree;
	    kdl_parser::treeFromUrdfModel( model, tree );

	    addChildren( tree.getRootSegment() );

	    mimic_.clear();
	    for(std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++)
	    {
		if(i->second->mimic)
		{
		    mimic_.insert(make_pair(i->first, i->second->mimic));
		}
	    }
	    
	    msgJointStates_.name = pMotion_.call<std::vector<std::string> >("getBodyNames", "Body" );
	}

	void JointStateConverter::registerCallback( const MessageAction::MessageAction action, callbackT callBack )
	{
	    callbacks_[action] = callBack;
	}

	void JointStateConverter::callAll( const std::vector<MessageAction::MessageAction>& actions )
	{
	    std::vector<double> alJointAngles = pMotion_.call<std::vector<double> >("getAngles", "Body", true );
	    const ros::Time& stamp = ros::Time::now();
	    
	    msgJointStates_.header.stamp = stamp;

	    msgJointStates_.position = std::vector<double>( alJointAngles.begin(), alJointAngles.end() );

	    std::map< std::string, double > jointStateMap;
	    
	    std::vector<double>::const_iterator itPos = msgJointStates_.position.begin();
	    for(std::vector<std::string>::const_iterator itName = msgJointStates_.name.begin(); itName != msgJointStates_.name.end(); ++itName, ++itPos)
	    {
		jointStateMap[*itName] = *itPos;
	    }

	    // for mimic map
	    for(mimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++)
	    {
		if(jointStateMap.find(i->second->joint_name) != jointStateMap.end()){
		double pos = jointStateMap[i->second->joint_name] * i->second->multiplier + i->second->offset;
		jointStateMap[i->first] = pos;
		}
	    }

	    // reset the transforms we want to use at this time
	    tfTransforms_.clear();
	    static const std::string& jt_tf_prefix = "";
	    setTransforms(jointStateMap, stamp, jt_tf_prefix);
	    setFixedTransforms(jt_tf_prefix, stamp);


	    std::vector<float> alOdometryData = pMotion_.call<std::vector<float> >( "getPosition", "Torso", 1, true );
	    const ros::Time& odom_stamp = ros::Time::now();
	    const float& odomX  =  alOdometryData[0];
	    const float& odomY  =  alOdometryData[1];
	    const float& odomZ  =  alOdometryData[2];
	    const float& odomWX =  alOdometryData[3];
	    const float& odomWY =  alOdometryData[4];
	    const float& odomWZ =  alOdometryData[5];
	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    tf2::Quaternion tf_quat;
	    tf_quat.setRPY( odomWX, odomWY, odomWZ );
	    geometry_msgs::Quaternion odom_quat = tf2::toMsg( tf_quat );

	    static geometry_msgs::TransformStamped msgTfOdom;
	    msgTfOdom.header.frame_id = "odom";
	    msgTfOdom.child_frame_id = "base_link";
	    msgTfOdom.header.stamp = odom_stamp;

	    msgTfOdom.transform.translation.x = odomX;
	    msgTfOdom.transform.translation.y = odomY;
	    msgTfOdom.transform.translation.z = odomZ;
	    msgTfOdom.transform.rotation = odom_quat;

	    tfTransforms_.push_back( msgTfOdom );
	    tf2Buffer_->setTransform( msgTfOdom, "naoqiconverter", false);

	    if (robot_ == Robot::NAO )
	    {
		Nao::addBaseFootprint( tf2Buffer_, tfTransforms_, odom_stamp-ros::Duration(0.1) );
	    }

	    // If nobody uses that buffer, do not fill it next time
	    if (( tf2Buffer_ ) && ( tf2Buffer_.use_count() == 1 ))
	    {
		tf2Buffer_.reset();
	    }

	    for_each( MessageAction::MessageAction action, actions )
	    {
		callbacks_[action]( msgJointStates_, tfTransforms_ );
	    }
	}


	// Copied from robot state publisher
	void JointStateConverter::setTransforms(const std::map<std::string, double>& jointPositions, const ros::Time& time, const std::string& tfPrefix)
	{
	    geometry_msgs::TransformStamped tfTransform;
	    tfTransform.header.stamp = time;

	    // loop over all joints
	    for (std::map<std::string, double>::const_iterator jnt=jointPositions.begin(); jnt != jointPositions.end(); jnt++)
	    {
		std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg = segments_.find(jnt->first);
		if (seg != segments_.end()){
		seg->second.segment.pose(jnt->second).M.GetQuaternion(tfTransform.transform.rotation.x,
									tfTransform.transform.rotation.y,
									tfTransform.transform.rotation.z,
									tfTransform.transform.rotation.w);
		tfTransform.transform.translation.x = seg->second.segment.pose(jnt->second).p.x();
		tfTransform.transform.translation.y = seg->second.segment.pose(jnt->second).p.y();
		tfTransform.transform.translation.z = seg->second.segment.pose(jnt->second).p.z();

		//tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
		//tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
		tfTransform.header.frame_id = seg->second.root; // tf2 does not suppport tf_prefixing
		tfTransform.child_frame_id = seg->second.tip;

		tfTransforms_.push_back(tfTransform);

		if (tf2Buffer_)
		    tf2Buffer_->setTransform(tfTransform, "naoqiconverter", false);
		}
	    }

	}

	// Copied from robot state publisher
	void JointStateConverter::setFixedTransforms(const std::string& msgJointStates_, const ros::Time& time)
	{
	    geometry_msgs::TransformStamped tfTransform;
	    tfTransform.header.stamp = time/*+ros::Duration(0.5)*/;  // future publish by 0.5 seconds

	    // loop over all fixed segments
	    for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg=segmentsFixed_.begin(); seg != segmentsFixed_.end(); seg++)
	    {
		seg->second.segment.pose(0).M.GetQuaternion(tfTransform.transform.rotation.x,
							    tfTransform.transform.rotation.y,
							    tfTransform.transform.rotation.z,
							    tfTransform.transform.rotation.w);
		tfTransform.transform.translation.x = seg->second.segment.pose(0).p.x();
		tfTransform.transform.translation.y = seg->second.segment.pose(0).p.y();
		tfTransform.transform.translation.z = seg->second.segment.pose(0).p.z();

		//tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
		//tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
		tfTransform.header.frame_id = seg->second.root;
		tfTransform.child_frame_id = seg->second.tip;

		tfTransforms_.push_back(tfTransform);

		if (tf2Buffer_)
		tf2Buffer_->setTransform(tfTransform, "naoqiconverter", true);
	    }
	    //tf_broadcaster_.sendTransform(tf_transforms);
	 }

	void JointStateConverter::addChildren(const KDL::SegmentMap::const_iterator segment)
	{
	    const std::string& root = GetTreeElementSegment(segment->second).getName();

	    const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
	    for (unsigned int i=0; i<children.size(); i++)
	    {
		const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
		robot_state_publisher::SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
		if (child.getJoint().getType() == KDL::Joint::None)
		{
		    segmentsFixed_.insert(std::make_pair(child.getJoint().getName(), s));
		    ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
		}
		else
		{
		    segments_.insert(std::make_pair(child.getJoint().getName(), s));
		    ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
		}
		addChildren(children[i]);
	    }
	}

    }
} 
