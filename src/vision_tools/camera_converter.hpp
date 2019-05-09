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


#ifndef CAMERA_CONVERTER_HPP
#define CAMERA_CONVERTER_HPP

#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

#include "../converters/converter_base.hpp"
#include "robot_toolkit/message_actions.h"

#include <boost/assign/list_of.hpp>

#include <image_transport/image_transport.h>
#include "../helpers/vision_helpers.hpp"

#include <qi/anyvalue.hpp>

namespace Sinfonia
{
    namespace Converter
    {

	class CameraConverter : public BaseConverter<CameraConverter>
	{

	    typedef boost::function<void(sensor_msgs::ImagePtr, sensor_msgs::CameraInfo)> CallbackT;

	    public:
		CameraConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session,  int cameraSource, int resolution);
		~CameraConverter();

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void callAll( const std::vector<MessageAction::MessageAction>& actions );

		void reset( );

	    private:
		
		
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;

		qi::AnyObject _pVideo;
		int _cameraSource;
		int _resolution;
		int _colorSpace;
		std::string _handle;

		std::string _msgColorspace;
		int _cvMatType;
		
		std::string _msgFrameid;
		sensor_msgs::CameraInfo _cameraInfo;
		sensor_msgs::ImagePtr _imageMsg;
		
		void callCamera();
		const sensor_msgs::CameraInfo& getCameraInfo( int cameraSource, int resolution );
		const sensor_msgs::CameraInfo& getEmptyInfo();
		
		void setCameraConfig(int cameraSource, int resolution, float frecuency);
		
		Helpers::VisionHelpers::NaoqiImage fromAnyValueToNaoqiImage(qi::AnyValue& value);
		
		sensor_msgs::CameraInfo createCameraInfoTOPVGA();
		sensor_msgs::CameraInfo createCameraInfoTOPQVGA();
		sensor_msgs::CameraInfo createCameraInfoTOPQQVGA();
		sensor_msgs::CameraInfo createCameraInfoBOTTOMVGA();
		sensor_msgs::CameraInfo createCameraInfoBOTTOMQVGA();
		sensor_msgs::CameraInfo createCameraInfoBOTTOMQQVGA();
		sensor_msgs::CameraInfo createCameraInfoDEPTHVGA();
		sensor_msgs::CameraInfo createCameraInfoDEPTHQVGA();
		sensor_msgs::CameraInfo createCameraInfoDEPTHQQVGA();
		
	};

    } 
} 

#endif
