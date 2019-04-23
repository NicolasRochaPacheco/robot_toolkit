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


#include "odom.hpp"

namespace Sinfonia
{
    namespace Converter
    {	
	OdomConverter::OdomConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session):
	BaseConverter( name, frequency, session )
	{
	    _pMotion =  session->service("ALMotion");

	}
	
	void OdomConverter::registerCallback(message_actions::MessageAction action, Callback_t callback)
	{
	    _callbacks[action] = callback;
	}
	
	void OdomConverter::callAll(const std::vector<message_actions::MessageAction>& actions )
	{
	    int FRAME_WORLD = 1;
	    bool use_sensor = true;
	    // documentation of getPosition available here: http://doc.aldebaran.com/2-1/naoqi/motion/control-cartesian.html
	    std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getPosition", "Torso", FRAME_WORLD, use_sensor );
	    
	    const ros::Time& odom_stamp = ros::Time::now();
	    std::vector<float> al_speed_data = p_motion_.call<std::vector<float> >( "getRobotVelocity" );
	    
	    const float& odomX  =  al_odometry_data[0];
	    const float& odomY  =  al_odometry_data[1];
	    const float& odomZ  =  al_odometry_data[2];
	    const float& odomWX =  al_odometry_data[3];
	    const float& odomWY =  al_odometry_data[4];
	    const float& odomWZ =  al_odometry_data[5];
	    
	    const float& dX = al_speed_data[0];
	    const float& dY = al_speed_data[1];
	    const float& dWZ = al_speed_data[2];

	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    tf2::Quaternion tf_quat;
	    tf_quat.setRPY( odomWX, odomWY, odomWZ );
	    geometry_msgs::Quaternion odom_quat = tf2::toMsg( tf_quat );

	    static nav_msgs::Odometry msg_odom;
	    msg_odom.header.frame_id = "odom";
	    msg_odom.child_frame_id = "base_link";
	    msg_odom.header.stamp = odom_stamp;

	    msg_odom.pose.pose.orientation = odom_quat;
	    msg_odom.pose.pose.position.x = odomX;
	    msg_odom.pose.pose.position.y = odomY;
	    msg_odom.pose.pose.position.z = odomZ;
	    
	    msg_odom.twist.twist.linear.x = dX;
	    msg_odom.twist.twist.linear.y = dY;
	    msg_odom.twist.twist.linear.z = 0;
	    
	    msg_odom.twist.twist.angular.x = 0;
	    msg_odom.twist.twist.angular.y = 0;
	    msg_odom.twist.twist.angular.z = dWZ;

	    for_each( message_actions::MessageAction action, actions )
	    {
		callbacks_[action](msg_odom);
		
	    }
	}


    }
    
}