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
#include "robot_toolkit/message_actions.h"

#include "robot_toolkit/ros_environment.hpp"

#include "publishers/joint_state.hpp"
#include "recorders/joint_state.hpp"
#include "converters/joint_state.hpp"




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
	sessionPtr = session;
	isRosLoopEnabled = true;
    }

    RobotToolkit::~RobotToolkit()
    {
	std::cout << "robot_toolkit is shutting down.." << std::endl;
	if(nodeHandlerPtr)
	{
	    nodeHandlerPtr->shutdown();
	    ros::shutdown();
	}
    }
    
    std::string RobotToolkit::_whoWillWin()
    {
	return "SinfonIA SSPL Robocup Team";
    }
    
    void RobotToolkit::init()
    {
	ros::Time::init();
	registerDefaultConverter();
	startRosLoop();
    }
    
    void RobotToolkit::rosLoop()
    {
	int counter = 0;
	while(isRosLoopEnabled)
	{
	    printf("Hello world! %d times from Manuel world\n", counter);
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

    void RobotToolkit::registerDefaultConverter()
    {
	tf2Buffer.reset<tf2_ros::Buffer>( new tf2_ros::Buffer() );
	tf2Buffer->setUsingDedicatedThread(true);
	
	boost::shared_ptr<Publisher::JointStatePublisher> jointStatePublisher = boost::make_shared<Publisher::JointStatePublisher>( "/joint_states" );
	boost::shared_ptr<Recorder::JointStateRecorder> jointStateRecorder = boost::make_shared<Recorder::JointStateRecorder>( "/joint_states" );
	boost::shared_ptr<Converter::JointStateConverter> jointStateConverter = boost::make_shared<Converter::JointStateConverter>( "joint_states", 50, tf2Buffer, sessionPtr );
	jointStateConverter->registerCallback( MessageAction::PUBLISH, boost::bind(&Publisher::JointStatePublisher::publish, jointStatePublisher, _1, _2) );
	jointStateConverter->registerCallback( MessageAction::RECORD, boost::bind(&Recorder::JointStateRecorder::write, jointStateRecorder, _1, _2) );
	jointStateConverter->registerCallback( MessageAction::LOG, boost::bind(&Recorder::JointStateRecorder::bufferize, jointStateRecorder, _1, _2) );
	registerGroup( jointStateConverter, jointStatePublisher, jointStateRecorder );
    }

    void RobotToolkit::registerGroup(Converter::Converter converter, Publisher::Publisher publisher, Recorder::Recorder recorder)
    {
	registerConverter(converter);
	registerPublisher(converter.name(), publisher);
	registerRecorder(converter.name(), recorder, converter.frequency());
    }
    
    void RobotToolkit::registerConverter(Converter::Converter& converter)
    {
	
    }

    void RobotToolkit::registerPublisher(const std::string& converterName, Publisher::Publisher& publisher)
    {
	
    }
    void RobotToolkit::registerRecorder(const std::string& converterName, Recorder::Recorder& recorder, float frequency)
    {

    }

    QI_REGISTER_OBJECT( RobotToolkit, _whoWillWin, setMasterURINet);
}
