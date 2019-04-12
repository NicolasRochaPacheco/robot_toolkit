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



#include "robot_toolkit/robot_toolkit.hpp"
#include "robot_toolkit/ros_environment.hpp"
namespace Sinfonia
{
    RobotToolkit::RobotToolkit(qi::SessionPtr session, const std::string& prefix)
    {
	if(prefix == "")
	{
	    std::cout << "Error driver prefix must not be empty" << std::endl;
	    throw new ros::Exception("Error driver prefix must not be empty");
	}
	else
	{
	    Sinfonia::RosEnvironment::setPrefix(prefix);
	}
	isRosLoopEnabled = true;
    }

    RobotToolkit::~RobotToolkit()
    {
	
    }
    
    std::string RobotToolkit::_whoWillWin()
    {
	return "SinfonIA SSPL Robocup Team";
    }
    
    void RobotToolkit::init()
    {
	ros::Time::init();
	startRosLoop();
    }
    
    void RobotToolkit::rosLoop()
    {
	int counter = 0;
	while(isRosLoopEnabled)
	{
	    printf("Hello world! %d times \n", counter);
	    counter++;
	    ros::Duration(1).sleep();
	}
    }
    
    void RobotToolkit::startRosLoop()
    {
	if (mainThread.get_id() ==  boost::thread::id())
	    mainThread = boost::thread( &RobotToolkit::rosLoop, this );
	/*
	for(EventIter i = eventMap.begin(); i != eventMap.end(); i++)
	{
	    i->second.startProcess();
	}*/
	isRosLoopEnabled = true;
    }

    void RobotToolkit::stopRosLoop()
    {
	isRosLoopEnabled = false;
	if (mainThread.get_id() !=  boost::thread::id())
	    mainThread.join();
	/*for(EventIter i = eventMap.begin(); i != eventMap.end(); i++)
	{
	    i->second.stopProcess();
	}*/	
    }
    
    void RobotToolkit::setMasterURINet(const std::string& uri, const std::string& networkInterface)
    {
	boost::mutex::scoped_lock lock( mutexConvertersQueue );
	{
	    nodeHandlerPtr.reset();
	    std::cout << "nodehandle reset " << std::endl;
	    Sinfonia::RosEnvironment::setMasterURI( uri, networkInterface );
	    nodeHandlerPtr.reset( new ros::NodeHandle("~") );
	}
	
    }

    void RobotToolkit::stopService()
    {
	stopRosLoop();
    }


    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet);
}
