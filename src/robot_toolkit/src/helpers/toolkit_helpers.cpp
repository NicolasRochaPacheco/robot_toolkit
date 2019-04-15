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


#include "toolkit_helpers.hpp"

namespace Sinfonia
{
    namespace Helpers
    {
	namespace Toolkit
	{

	    static naoqi_bridge_msgs::RobotInfo& getRobotInfoLocal( const qi::SessionPtr& session)
	    {
		static naoqi_bridge_msgs::RobotInfo info;
		static qi::Url robotUrl;

		if (robotUrl == session->url())
		{
		    return info;
		}

		robotUrl = session->url();

		std::cout << "Receiving information about robot model" << std::endl;
		qi::AnyObject robotMemory = session->service("ALMemory");
		std::string robot = robotMemory.call<std::string>("getData", "RobotConfig/Body/Type" );
		std::transform(robot.begin(), robot.end(), robot.begin(), ::tolower);

		if (std::string(robot) == "nao")
		{
		    info.type = naoqi_bridge_msgs::RobotInfo::NAO;
		}
		if (std::string(robot) == "pepper" || std::string(robot) == "juliette" )
		{
		    info.type = naoqi_bridge_msgs::RobotInfo::PEPPER;
		}
		if (std::string(robot) == "romeo" )
		{
		    info.type = naoqi_bridge_msgs::RobotInfo::ROMEO;
		}

		qi::AnyObject robotMotion = session->service("ALMotion");
		std::vector<std::vector<qi::AnyValue> > config = robotMotion.call<std::vector<std::vector<qi::AnyValue> > >("getRobotConfig");

		for (size_t i=0; i<config[0].size();++i)
		{
		    if (config[0][i].as<std::string>() == "Model Type")
		    {
			try
			{
			    info.model = config[1][i].as<std::string>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Head Version")
		    {
			try
			{
			    info.head_version = config[1][i].as<std::string>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Body Version")
		    {
			try
			{
			    info.body_version = config[1][i].as<std::string>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Arm Version")
		    {
			try
			{
			    info.arm_version = config[1][i].as<std::string>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Laser")
		    {
			try
			{
			    info.has_laser = config[1][i].as<bool>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Extended Arms")
		    {
			try
			{
			    info.has_extended_arms = config[1][i].as<bool>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Number of Legs")
		    {
			try
			{
			    info.number_of_legs = config[1][i].as<int>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Number of Arms")
		    {
			try
			{
			    info.number_of_arms = config[1][i].as<int>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		    if (config[0][i].as<std::string>() == "Number of Hands")
		    {
			try
			{
			    info.number_of_hands = config[1][i].as<int>();
			}
			catch(const std::exception& e)
			{
			    std::cout << "Error in robot config variable " << (config[0][i]).as<std::string>() << std::endl;
			}
		    }

		}
		return info;
	    }

	    const naoqi_bridge_msgs::RobotInfo& getRobotInfo( const qi::SessionPtr& session )
	    {
		static naoqi_bridge_msgs::RobotInfo robotInfo =  getRobotInfoLocal(session);
		return robotInfo;
	    }
	    
	    const Robot::Robot& getRobot( const qi::SessionPtr& session )
	    {
		static Robot::Robot robot = Robot::UNIDENTIFIED;

		if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::PEPPER )
		{
		    robot = Robot::PEPPER;
		}
		
		else if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::NAO )
		{
		    robot = Robot::NAO;
		}
		
		else if ( getRobotInfo(session).type == naoqi_bridge_msgs::RobotInfo::ROMEO )
		{
		    robot = Robot::ROMEO;
		}

		return robot;
	    }
	    
	    bool& setLanguage( const qi::SessionPtr& session, naoqi_bridge_msgs::SetStringRequest request)
	    {
		static bool success;
		std::cout << "Receiving service call of setting speech language" << std::endl;
		try
		{
		    qi::AnyObject robotTextToSpeech = session->service("ALTextToSpeech");
		    robotTextToSpeech.call<void>("setLanguage", request.data);
		    success = true;
		    return success;
		}
		catch(const std::exception& e)
		{
		    success = false;
		    return success;
		}
	    }

	    std::string& getLanguage( const qi::SessionPtr& session )
	    {
		static std::string language;
		std::cout << "Receiving service call of getting speech language" << std::endl;
		qi::AnyObject robotTextToSpeech = session->service("ALTextToSpeech");
		language = robotTextToSpeech.call<std::string>("getLanguage");
		return language;
	    }

	}
    } 
} 
