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

#ifndef ROBOT_TOOLKIT_HPP
#define ROBOT_TOOLKIT_HPP

#include <qi/session.hpp>
#include <boost/thread/mutex.hpp>

#include "robot_toolkit/ros_environment.hpp"

namespace Sinfonia
{
    class RobotToolkit
    {
	public:
	    RobotToolkit(qi::SessionPtr session, const std::string& prefix);
	    ~RobotToolkit();
	    
	    std::string _whoWillWin();
	    void init();
	    void stopService();
	    void setMasterURINet(const std::string& uri, const std::string& networkInterface);

	    
	private:
	    boost::thread mainThread;
	    //std::map< std::string, event::Event > eventMap;
	    bool isRosLoopEnabled;
	    boost::scoped_ptr<ros::NodeHandle> nodeHandlerPtr;
	    boost::mutex mutexConvertersQueue;
	    
	    void rosLoop();
	    void startRosLoop();
	    void stopRosLoop();
	   
    };
}

#endif
