/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef NAO_FOOTPRINT_HPP
#define NAO_FOOTPRINT_HPP

/*
* ROS includes
*/
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
* loca includes
*/
#include "../helpers/transform_helpers.hpp"

namespace Sinfonia
{
    namespace Converter
    {
	namespace Nao
	{
	    inline void addBaseFootprint(boost::shared_ptr<tf2_ros::Buffer> tf2_buffer, std::vector<geometry_msgs::TransformStamped>& tf_transforms, const ros::Time& time)
	    {
		bool canTransform = tf2_buffer->canTransform("odom", "l_sole", time, ros::Duration(0.1) );
		if (!canTransform)
		{
		    ROS_ERROR_STREAM("Do not calculate NAO Footprint, no transform possible " << time);
		    return;
		}

		geometry_msgs::TransformStamped tfOdomToBase, tfOdomToLeftFoot, tfOdomToRightFoot;
		try 
		{
		    tfOdomToLeftFoot  = tf2_buffer->lookupTransform("odom", "l_sole",    time );
		    tfOdomToBase = tf2_buffer->lookupTransform("odom", "r_sole",    time );
		    tfOdomToRightFoot = tf2_buffer->lookupTransform("odom", "base_link", time );
		}
		catch (const tf2::TransformException& ex)
		{
		    ROS_ERROR("NAO Footprint error %s",ex.what());
		    return ;
		}
		tf2::Vector3 newOrigin(float(tfOdomToRightFoot.transform.translation.x + tfOdomToLeftFoot.transform.translation.x)/2.0,
					float(tfOdomToRightFoot.transform.translation.y + tfOdomToLeftFoot.transform.translation.y)/2.0,
					std::min(tfOdomToLeftFoot.transform.translation.z, tfOdomToRightFoot.transform.translation.z));

		double yaw = Helpers::Transform::getYaw( tfOdomToBase.transform ) ;
		tf2::Quaternion newQuaternion;
		newQuaternion.setRPY(0.0f, 0.0f, yaw);
		tf2::Transform tf_odom_to_footprint( newQuaternion, newOrigin);

		tf2::Quaternion q( tfOdomToBase.transform.rotation.x,
				    tfOdomToBase.transform.rotation.y,
				    tfOdomToBase.transform.rotation.z,
				    tfOdomToBase.transform.rotation.w);
		tf2::Vector3 r( tfOdomToBase.transform.translation.x,
				tfOdomToBase.transform.translation.y,
				tfOdomToBase.transform.translation.z);
		tf2::Transform tf_odom_to_base_conv( q,r);
		tf2::Transform tf_base_to_footprint = tf_odom_to_base_conv.inverse() * tf_odom_to_footprint;

		geometry_msgs::TransformStamped message;
		message.header.stamp = time;
		message.header.frame_id = "base_link";
		message.child_frame_id = "base_footprint";

		message.transform.rotation.x = tf_base_to_footprint.getRotation().x();
		message.transform.rotation.y = tf_base_to_footprint.getRotation().y();
		message.transform.rotation.z = tf_base_to_footprint.getRotation().z();
		message.transform.rotation.w = tf_base_to_footprint.getRotation().w();
		message.transform.translation.x = tf_base_to_footprint.getOrigin().x();
		message.transform.translation.y = tf_base_to_footprint.getOrigin().y();
		message.transform.translation.z = tf_base_to_footprint.getOrigin().z();

		tf2_buffer->setTransform( message, "naoqiconverter", false );
		tf_transforms.push_back( message );
	    }

	}
    } 
}

#endif
