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


#ifndef FACE_DETECTOR_HPP
#define FACE_DETECTOR_HPP

#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

#include <sstream>

#include "robot_toolkit/converter/converter_base.hpp"
#include "robot_toolkit/message_actions.h"

#include <boost/assign/list_of.hpp>
#include <ros/package.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <image_transport/image_transport.h>
#include "robot_toolkit/helpers/vision_helpers.hpp"

#include <qi/anyvalue.hpp>

#include "robot_toolkit_msgs/camera_parameters_msg.h"

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/enable_shared_from_this.hpp>

#include <queue>

#include "robot_toolkit/vision_tools/face_publisher.hpp"

namespace Sinfonia
{
    namespace Converter
    {

	class FaceDetector : public BaseConverter<FaceDetector>, public boost::enable_shared_from_this<FaceDetector>
	{

	    typedef boost::function<void(sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr)> CallbackT;

	    public:
		FaceDetector( const std::string& name, const float& frequency, const qi::SessionPtr& session,  int cameraSource, int resolution, int colorSpace);
		~FaceDetector();

		void registerCallback( MessageAction::MessageAction action, CallbackT callback );

		void reset(ros::NodeHandle& nodeHandle );
		
		std::vector<float> setParameters(std::vector<float> parameters);
		std::vector<float> setAllParametersToDefault();
		std::vector<float> getParameters();
		
		
		void setConfig(std::vector<float> configs);
		
		void faceDetectedCallback(std::string key,  qi::AnyValue value, std::string subscriberIdentifier);

	    private:
		
		
		std::map<MessageAction::MessageAction, CallbackT> _callbacks;

		qi::AnyObject _pVideo;
		qi::AnyObject _pMemory;
		
		Helpers::VisionHelpers::NaoqiImage _lastImage;
		
		bool _compress;
		
		int _cvMatType;
		int _resolution;
		int _colorSpace;
		int _cameraSource;
		int _compressionFactor;
		
		unsigned int _serviceId;
		
		std::string _handle;
		std::string _msgFrameid;
		std::string _msgColorspace;
		
		
		sensor_msgs::ImagePtr _imageMsg;
		sensor_msgs::CameraInfoPtr _cameraInfo;
		sensor_msgs::CameraInfoPtr loadCameraInfo();
		
		const sensor_msgs::CameraInfoPtr getEmptyInfo();
		
		Helpers::VisionHelpers::NaoqiImage fromAnyValueToNaoqiImage(qi::AnyValue& value);
		
		void callCamera();
		void compressImage();
		void activateFaceDetection();
		void deactivateFaceDetection();
		
		boost::shared_ptr<Publisher::FacePublisher> _publisher;
		
		
	};
	
	QI_REGISTER_OBJECT(FaceDetector, faceDetectedCallback)

    } 
} 

#endif
