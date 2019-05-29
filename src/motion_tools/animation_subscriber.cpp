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


#include "robot_toolkit/motion_tools/animation_subscriber.hpp"

namespace Sinfonia 
{
    namespace Subscriber 
    {
	AnimationSubscriber::AnimationSubscriber(const std::string& name, const std::string& topic, const qi::SessionPtr& session):
	  BaseSubscriber(name, topic, session)
	{
	    _topic = topic;
	    _pBehaviour = session->service("ALBehaviorManager");
	    _isInitialized = false;
	}
	void AnimationSubscriber::reset(ros::NodeHandle& nodeHandle)
	{
	    _subscriberSpeech = nodeHandle.subscribe(_topic, 10, &AnimationSubscriber::animationCallback, this);
	    _isInitialized = true;
	}
	void AnimationSubscriber::animationCallback(const std_msgs::String message)
	{
	    std::string animationPath = "animations_sinfonia/animations/" + message.data;
	 //   std::cout << "Path: " << animationPath << std::endl;
	    _pBehaviour.async<void>("startBehavior", animationPath);
	}
	void AnimationSubscriber::shutdown()
	{
	    _subscriberSpeech.shutdown();
	    _isInitialized = false;
	}
	std::vector< float > AnimationSubscriber::getParameters()
	{
	    std::vector<float> result;
	    return result;
	}

	std::vector< float > AnimationSubscriber::setDefaultParameters()
	{	
	    return getParameters();
	}
	std::vector< float > AnimationSubscriber::setParameters(std::vector< float > parameters)
	{
	    return getParameters();
	}
    }
}