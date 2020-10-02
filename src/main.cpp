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


// qi::applicationsession allow the communication with naoqi from C++
#include <qi/applicationsession.hpp>

// qi::AnyModule allows an user to use qi modules w/out previous knowledge
#include <qi/anymodule.hpp>

// A boost package that allows passing arguments from command line
#include <boost/program_options.hpp>

// Header files for robot toolkit
#include "robot_toolkit/tools/tools.hpp"
#include "robot_toolkit/robot_toolkit.hpp"
#include "robot_toolkit/naoqi_environment.hpp"


/*
 * Main function for robot toolkit.
 */
int main(int argc, char **argv) {

    // Sets the floating point input/outputs to 3 significant values
    std::cout << std::fixed << std::setprecision(3);
    
    // Creates the qi session object
    qi::ApplicationSession app(argc, argv);
    
    // Creates a string vector
    std::vector<std::string> argsOut;
    
    // Returns a vector of program arguments that do not include any ROS remapping arguments.
    ros::removeROSArgs( argc, argv, argsOut );
    
    // Retrieves the program_options namespace
    namespace programOptions = boost::program_options;
    
    // Creates the parameters/options description
    programOptions::options_description optionsDescription("Options");
    
    // Adds the options for program. (parameter name, parameter type (and default value), description)
    optionsDescription.add_options()
	("help,h", "print help message")
	("roscore_ip,r", programOptions::value<std::string>(), "set the ip of the roscore to use")
	("network_interface,i", programOptions::value<std::string>()->default_value("eth0"),  "set the network interface over which to connect")
	("namespace,n", programOptions::value<std::string>()->default_value("robot_toolkit_node"), "set an explicit namespace in case ROS namespace variables cannot be used");

    // Creates the variables map to store the program parameters/options
    programOptions::variables_map variablesMap;
    
    // Tries to store program options, and throws error if argument has invalid syntax or its unknown
    try {
	   programOptions::store( programOptions::parse_command_line(argc, argv, optionsDescription), variablesMap );
    } catch (boost::program_options::invalid_command_line_syntax& e) {
	   std::cout << "Error 0x01: Invialid Sintax" << e.what() << std::endl;
	   throw ros::Exception(e.what());
    } catch (boost::program_options::unknown_option& e) {
	   std::cout << "Error 0x02: Unknown option" << e.what() << std::endl;
	   throw ros::Exception(e.what());
    }

    // Checks if help was passed as a program option
    if(variablesMap.count("help")) {
	   std::cout << "This is the help message for the SinfonIA RobotToolkit" << std::endl << optionsDescription << std::endl;
	   exit(0);
    }

    // Checks qi version to allow different versions compatibility
    #if LIBQI_VERSION>24
	   app.startSession();
    #else
	   app.start();
    #endif
    
    // shared_ptr: It is a smart pointer, but still a pointer
    // make_shared: Creates the pointer so boost::shared_ptr is happy
    // Creates a RobotToolkit instance as a smart pointer
    boost::shared_ptr<Sinfonia::RobotToolkit> robotToolkit = boost::make_shared<Sinfonia::RobotToolkit>(app.session(), variablesMap["namespace"].as<std::string>());
    
    // Registers robot toolkit as qi service and assigns the robot_toolkit name to it
    app.session()->registerService("robot_toolkit", robotToolkit);
    
    // Checks for roscore IP parameter
    if (variablesMap.count("roscore_ip")){
	   // Retrieves the roscore IP string
       std::string roscoreIp = variablesMap["roscore_ip"].as<std::string>();
	   
       // Retrieves the network interface string
       std::string networkInterface = variablesMap["network_interface"].as<std::string>();

       // Prints the IP and Network Interface on console
	   std::cout << BOLDYELLOW << "using ip address: " << BOLDCYAN << roscoreIp << " @ " << networkInterface << RESETCOLOR << std::endl;
	   
       // Calls the init function from RobotToolkit
	   robotToolkit->init();

       // Calls the setMasterURINet function from RobotToolkit
	   robotToolkit->setMasterURINet( "http://"+roscoreIp+":11311", networkInterface); 

       // Calls the startInitialTopics function from RobotToolkit
	   robotToolkit->startInitialTopics();
    
    // If no roscore IP is passed, toolkit is intialized but no MasterURI is set.
    } else {
        // Prints status into console
        std::cout << BOLDRED << "No ip address given. Run qicli call to set the master uri" << RESETCOLOR << std::endl;
	   
        // Calls the init function from RobotToolkit
        robotToolkit->init();
    }
    
    // Runs the qi application
    app.run();
    
    // When qi application is stopped, robot toolkit service is also stopped
    robotToolkit->stopService();
    
    // Closes the qi session
    app.session()->close();
    
    // Exits with error code 0
    return 0;
}

