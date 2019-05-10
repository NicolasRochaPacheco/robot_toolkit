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


#include "camera_converter.hpp"
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace Sinfonia
{
    namespace Converter
    {
	CameraConverter::CameraConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session, int cameraSource, int resolution, int colorSpace): 
	  BaseConverter(name, frequency, session)
	{	    
	    _pVideo = session->service("ALVideoDevice");
	    _cameraSource = cameraSource;
	    std::vector<int> configs;
	    configs.push_back(resolution);
	    configs.push_back(frequency);
	    configs.push_back(colorSpace);
	    setConfig(configs);
	}
	
	CameraConverter::~CameraConverter()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "Unsubscribe camera handle: " << _handle << std::endl;
		_handle.clear();
	    }
	}

	void CameraConverter::registerCallback(MessageAction::MessageAction action, Converter::CameraConverter::CallbackT callback)
	{
	    _callbacks[action] = callback;
	}
	void CameraConverter::callAll(const std::vector< MessageAction::MessageAction >& actions)
	{
	    if (_handle.empty() )
	    {
		std::cerr << _name << "Camera Handle is empty - cannot retrieve image" << std::endl;
		std::cerr << _name << "Might be a NAOqi problem. Try to restart the ALVideoDevice." << std::endl;
		return;
	    }
	    callCamera();
	    for_each(MessageAction::MessageAction action, actions)
	    {
		_callbacks[action](_imageMsg, _cameraInfo);
	    }
	}
	void CameraConverter::reset()
	{
	    if (!_handle.empty())
	    {
		_pVideo.call<qi::AnyValue>("unsubscribe", _handle);
		_handle.clear();
	    }
	    _handle = _pVideo.call<std::string>("subscribeCamera", _name, _cameraSource, _resolution, _colorSpace, (int)_frequency);
	    
	    
	}
	
	std::vector<int> CameraConverter::setParameters(std::vector<int> parameters)
	{
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraBrightnessID, parameters[0]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraContrastID, parameters[1]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraSaturationID, parameters[2]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraHueID, parameters[3]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraHFlipID, parameters[4]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraVFlipID, parameters[5]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoExpositionID, parameters[6]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoWhiteBalanceID, parameters[7]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoGainID, parameters[8]);
	    if(!parameters[8])
	    {
		_pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraGainID, parameters[9]);
	    }
	    if(!parameters[6] )
	    {
		_pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraExposureID, parameters[10]);
	    }
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraSetDefaultParamsID, parameters[11]);
	    _pVideo.call<bool>("setCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoFocusID, parameters[18]);
	    
	    return getParameters();
	}
	
	std::vector<int> CameraConverter::setAllParametersToDefault()
	{
	    _pVideo.call<bool>("setAllParametersToDefault", _cameraSource);
	    return getParameters();
	}

	std::vector<int> CameraConverter::getParameters()
	{
	    std::vector<int> result;
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBrightnessID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraContrastID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraSaturationID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraHueID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraHFlipID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraVFlipID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoExpositionID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoWhiteBalanceID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoGainID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraGainID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraExposureID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraSetDefaultParamsID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcRedID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcGbID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraBlcBlueID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraResolutionID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraFrameRateID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAverageLuminanceID));
	    result.push_back(_pVideo.call<int>("getCameraParameter", _handle, Helpers::VisionHelpers::kCameraAutoFocusID));
	    return result;
	}
	
	void CameraConverter::callCamera()
	{
	    qi::AnyValue imageAnyValue = _pVideo.call<qi::AnyValue>("getImageRemote", _handle);
	    Helpers::VisionHelpers::NaoqiImage image;
	    try{
		image = fromAnyValueToNaoqiImage(imageAnyValue);
	    }
	    catch(std::runtime_error& e)
	    {
		std::cout << "Cannot retrieve image" << std::endl;
		return;
	    }

	    cv::Mat cvImage(image.height, image.width, _cvMatType, image.buffer);
	    _imageMsg = cv_bridge::CvImage(std_msgs::Header(), _msgColorspace, cvImage).toImageMsg();
	    _imageMsg->header.frame_id = _msgFrameid;

	    _imageMsg->header.stamp = ros::Time::now();
	    _cameraInfo.header.stamp = _imageMsg->header.stamp;
	}
	
	const sensor_msgs::CameraInfo& CameraConverter::getEmptyInfo()
	{
	    static const sensor_msgs::CameraInfo camInfoMsg;
	    return camInfoMsg;
	}

	const sensor_msgs::CameraInfo& CameraConverter::getCameraInfo(int cameraSource, int resolution)
	{
	    if ( _cameraSource == Helpers::VisionHelpers::kTopCamera )
	    {
		if ( resolution == Helpers::VisionHelpers::kVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoTOPVGA();
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoTOPQVGA();
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoTOPQQVGA();
		    return camInfoMsg;
		}
		else
		{
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "no camera information found for camera source " << _cameraSource << " and res: " << resolution << std::endl;
		    return getEmptyInfo();
		}
	    }
	    else if ( _cameraSource == Helpers::VisionHelpers::kBottomCamera )
	    {
		if ( resolution == Helpers::VisionHelpers::kVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoBOTTOMVGA();
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoBOTTOMQVGA();
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoBOTTOMQQVGA();
		    return camInfoMsg;
		}
		else
		{
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "no camera information found for camera source " << _cameraSource << " and res: " << resolution << std::endl;
		    return getEmptyInfo();
		}
	    }
	    else if ( _cameraSource == Helpers::VisionHelpers::kDepthCamera )
	    {
		if ( resolution == Helpers::VisionHelpers::kVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoDEPTHVGA();
		    ROS_WARN("VGA resolution is not supported for the depth camera, use QVGA or lower");
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoDEPTHQVGA();
		    return camInfoMsg;
		}
		else if( resolution == Helpers::VisionHelpers::kQQVGA )
		{
		    static const sensor_msgs::CameraInfo camInfoMsg = createCameraInfoDEPTHQQVGA();
		    return camInfoMsg;
		}
		else
		{
		    std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "no camera information found for camera source " << _cameraSource << " and res: " << resolution << std::endl;
		    return getEmptyInfo();
		}
	    }
	    else
	    {
		std::cout << BOLDYELLOW << "[" << ros::Time::now().toSec() << "] " << "no camera information found for camera source " << _cameraSource << " and res: " << resolution << std::endl;
		return getEmptyInfo();
	    }
	}
	
	void CameraConverter::setConfig(std::vector<int> configs)
	{
	    _resolution = configs[0];
	    setFrequency(configs[1]);
	    _colorSpace = configs[2];

	    if( _colorSpace == Helpers::VisionHelpers::kYuvColorSpace || _colorSpace == Helpers::VisionHelpers::kyUvColorSpace || _colorSpace == Helpers::VisionHelpers::kyuVColorSpace || _colorSpace == Helpers::VisionHelpers::kRgbColorSpace ||
		_colorSpace == Helpers::VisionHelpers::krGbColorSpace || _colorSpace == Helpers::VisionHelpers::krgBColorSpace || _colorSpace == Helpers::VisionHelpers::kHsyColorSpace || _colorSpace == Helpers::VisionHelpers::khSyColorSpace ||
		_colorSpace == Helpers::VisionHelpers::khsYColorSpace )
	    {
		_msgColorspace = "mono8";
		_cvMatType = CV_8U;
	    }
	    else if( _colorSpace == Helpers::VisionHelpers::kYUV422ColorSpace || _colorSpace == Helpers::VisionHelpers::kYYCbCrColorSpace )
	    {
		_msgColorspace = "mono16";
		_cvMatType = CV_16UC2;
	    }
	    else if( _colorSpace == Helpers::VisionHelpers::kDepthColorSpace || _colorSpace == Helpers::VisionHelpers::kDistanceColorSpace || _colorSpace == Helpers::VisionHelpers::kRawDepthColorSpace )
	    {
		_msgColorspace = "16UC1";
		_cvMatType = CV_16U;
	    }
	    else if ( _colorSpace == Helpers::VisionHelpers::kXYZColorSpace )
	    {
		_msgColorspace = "rgb8";
		_cvMatType = CV_32FC3;
	    }
	    else
	    {
		_msgColorspace = "rgb8"; 
		_cvMatType = CV_8UC3;
	    }
	    _cameraInfo = getCameraInfo(_cameraSource, _resolution);
	    
	    if ( _cameraSource == Helpers::VisionHelpers::kTopCamera )
	    {
		_msgFrameid = "CameraTop_optical_frame";
	    }
	    else if (_cameraSource == Helpers::VisionHelpers::kBottomCamera )
	    {
		_msgFrameid = "CameraBottom_optical_frame";
	    }
	    else
	    {
		_msgFrameid = "CameraDepth_optical_frame";
	    }
	}
	
	Helpers::VisionHelpers::NaoqiImage CameraConverter::fromAnyValueToNaoqiImage(qi::AnyValue& value)
	{
	    qi::AnyReferenceVector anyReference;
	    Helpers::VisionHelpers::NaoqiImage result;
	    std::ostringstream stringStream;
	    try
	    {
		anyReference = value.asListValuePtr();
	    }
	    catch(std::runtime_error& e)
	    {
		stringStream << "Could not transform AnyValue into list: " << e.what();
		throw std::runtime_error(stringStream.str());
	    }
	    
	    qi::AnyReference ref;

	    ref = anyReference[0].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.width = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve width";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[1].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.height = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve height";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[2].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.numberOfLayers = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve number of layers";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[3].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.colorSpace = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve colorspace";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[4].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.timeStampS = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve timestamp_s";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[5].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.timeStampUs = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve timestamp_us";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[6].content();
	    if(ref.kind() == qi::TypeKind_Raw)
	    {
		result.buffer = (void*)ref.asRaw().first;
	    }
	    else
	    {
		stringStream << "Could not retrieve buffer";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[7].content();
	    if(ref.kind() == qi::TypeKind_Int)
	    {
		result.camId = ref.asInt32();
	    }
	    else
	    {
		stringStream << "Could not retrieve camId";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[8].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovLeft = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_left";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[9].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovTop = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_top";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[10].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovRight = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_right";
		throw std::runtime_error(stringStream.str());
	    }

	    ref = anyReference[11].content();
	    if(ref.kind() == qi::TypeKind_Float)
	    {
		result.fovBottom = ref.asFloat();
	    }
	    else
	    {
		stringStream << "Could not retrieve fov_bottom";
		throw std::runtime_error(stringStream.str());
	    }
	    return result;
	}

	sensor_msgs::CameraInfo CameraConverter::createCameraInfoTOPVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraTop_optical_frame";

	    cameraInfoMessage.width = 640;
	    cameraInfoMessage.height = 480;
	   
	    cameraInfoMessage.K[0] = 556.845054830986;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 309.366895338178;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 555.898679730161;
	    cameraInfoMessage.K[5] = 230.592233628776;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;
	     
	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0545211535376379)(0.0691973423510287)(-0.00241094929163055)(-0.00112245009306511)(0).convert_to_container<std::vector<double> >();

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 551.589721679688, 0, 308.271132841983, 0, 0, 550.291320800781, 229.20143668168, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 551.589721679688;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 308.271132841983;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 550.291320800781;
	    cameraInfoMessage.P[6] = 229.20143668168;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoTOPQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraTop_optical_frame";

	    cameraInfoMessage.width = 320;
	    cameraInfoMessage.height = 240;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 274.139508945831, 0, 141.184472810944, 0, 275.741846757374, 106.693773654172, 0, 0, 1 }};
	    cameraInfoMessage.K[0] = 274.139508945831;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 141.184472810944;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 275.741846757374;
	    cameraInfoMessage.K[5] = 106.693773654172;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0870160932911717)(0.128210165050533)(0.003379500659424)(-0.00106205540818586)(0).convert_to_container<std::vector<double> >();

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;


	    //cameraInfoMessage.P = boost::array<double, 12>{{ 272.423675537109, 0, 141.131930791285, 0, 0, 273.515747070312, 107.391746054313, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 272.423675537109;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 141.131930791285;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 273.515747070312;
	    cameraInfoMessage.P[6] = 107.391746054313;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoTOPQQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraTop_optical_frame";

	    cameraInfoMessage.width = 160;
	    cameraInfoMessage.height = 120;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 139.424539568966, 0, 76.9073669920582, 0, 139.25542782325, 59.5554242026743, 0, 0, 1 }};
	    cameraInfoMessage.K[0] = 139.424539568966;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 76.9073669920582;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 139.25542782325;
	    cameraInfoMessage.K[5] = 59.5554242026743;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0843564504845967)(0.125733083790192)(0.00275901756247071)(-0.00138645823460527)(0).convert_to_container<std::vector<double> >();

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 137.541534423828, 0, 76.3004646597892, 0, 0, 136.815216064453, 59.3909799751191, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 272.423675537109;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 141.131930791285;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 273.515747070312;
	    cameraInfoMessage.P[6] = 107.391746054313;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoDEPTHVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraDepth_optical_frame";

	    cameraInfoMessage.width = 640;
	    cameraInfoMessage.height = 480;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 525, 0, 319.5000000, 0, 525, 239.5000000000000, 0, 0, 1  }};
	    cameraInfoMessage.K[0] = 525;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 319.5000000;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 525;
	    cameraInfoMessage.K[5] = 239.5000000000000;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 525, 0, 319.500000, 0, 0, 525, 239.5000000000, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 525;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 319.500000;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 525;
	    cameraInfoMessage.P[6] = 239.5000000000;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoDEPTHQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraDepth_optical_frame";

	    cameraInfoMessage.width = 320;
	    cameraInfoMessage.height = 240;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 525/2.0f, 0, 319.5000000/2.0f, 0, 525/2.0f, 239.5000000000000/2.0f, 0, 0, 1  }};
	    cameraInfoMessage.K[0] = 525/2.0f;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 319.5000000/2.0f;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 525/2.0f;
	    cameraInfoMessage.K[5] = 239.5000000000000/2.0f;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 525/2.0f, 0, 319.500000/2.0f, 0, 0, 525/2.0f, 239.5000000000/2.0f, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 525/2.0f;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 319.500000/2.0f;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 525/2.0f;
	    cameraInfoMessage.P[6] = 239.5000000000/2.0f;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoDEPTHQQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraDepth_optical_frame";

	    cameraInfoMessage.width = 160;
	    cameraInfoMessage.height = 120;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 525/4.0f, 0, 319.5000000/4.0f, 0, 525/4.0f, 239.5000000000000/4.0f, 0, 0, 1  }};
	    cameraInfoMessage.K[0] = 525/4.0f;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 319.5000000/4.0f;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 525/4.0f;
	    cameraInfoMessage.K[5] = 239.5000000000000/4.0f;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;
	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 525/4.0f, 0, 319.500000/4.0f, 0, 0, 525/4.0f, 239.5000000000/4.0f, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 525/4.0f;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 319.5000000/4.0f;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 525/4.0f;
	    cameraInfoMessage.P[6] = 239.5000000000000/4.0f;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;
	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoBOTTOMVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraBottom_optical_frame";

	    cameraInfoMessage.width = 640;
	    cameraInfoMessage.height = 480;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 558.570339530768, 0, 308.885375457296, 0, 556.122943034837, 247.600724811385, 0, 0, 1 }};
	    cameraInfoMessage.K[0] = 558.570339530768;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 308.885375457296;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 556.122943034837;
	    cameraInfoMessage.K[5] = 247.600724811385;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;	
	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0648763971625288)(0.0612520196884308)(0.0038281538281731)(-0.00551104078371959)(0).convert_to_container<std::vector<double> >();

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 549.571655273438, 0, 304.799679526441, 0, 0, 549.687316894531, 248.526959297022, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 549.571655273438;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 304.799679526441;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 549.687316894531;
	    cameraInfoMessage.P[6] = 248.526959297022;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoBOTTOMQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraBottom_optical_frame";

	    cameraInfoMessage.width = 320;
	    cameraInfoMessage.height = 240;
	    //cameraInfoMessage.K = boost::array<double, 9>{{ 278.236008818534, 0, 156.194471689706, 0, 279.380102992049, 126.007123836447, 0, 0, 1 }};
	    cameraInfoMessage.K[0] = 278.236008818534;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 156.194471689706;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 279.380102992049;
	    cameraInfoMessage.K[5] = 126.007123836447;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0481869853715082)(0.0201858398559121)(0.0030362056699177)(-0.00172241952442813)(0).convert_to_container<std::vector<double> >();

	    //cameraInfoMessage.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    //cameraInfoMessage.P = boost::array<double, 12>{{ 273.491455078125, 0, 155.112454709117, 0, 0, 275.743133544922, 126.057357467223, 0, 0, 0, 1, 0 }};
	    cameraInfoMessage.P[0] = 273.491455078125;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 155.112454709117;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 275.743133544922;
	    cameraInfoMessage.P[6] = 126.057357467223;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}
	sensor_msgs::CameraInfo CameraConverter::createCameraInfoBOTTOMQQVGA()
	{
	    sensor_msgs::CameraInfo cameraInfoMessage;

	    cameraInfoMessage.header.frame_id = "CameraBottom_optical_frame";

	    cameraInfoMessage.width = 160;
	    cameraInfoMessage.height = 120;
	    
	    cameraInfoMessage.K[0] = 141.611855886672;
	    cameraInfoMessage.K[1] = 0;
	    cameraInfoMessage.K[2] = 78.6494086288656;
	    cameraInfoMessage.K[3] = 0;
	    cameraInfoMessage.K[4] = 141.367163830175;
	    cameraInfoMessage.K[5] = 58.9220646201529;
	    cameraInfoMessage.K[6] = 0;
	    cameraInfoMessage.K[7] = 0;
	    cameraInfoMessage.K[8] = 1;

	    cameraInfoMessage.distortion_model = "plumb_bob";
	    cameraInfoMessage.D = boost::assign::list_of(-0.0688388724945936)(0.0697453843669642)(0.00309518737071049)(-0.00570486993696543)(0).convert_to_container<std::vector<double> >();

	    
	    cameraInfoMessage.R[0] = 1;
	    cameraInfoMessage.R[1] = 0;
	    cameraInfoMessage.R[2] = 0;
	    cameraInfoMessage.R[3] = 0;
	    cameraInfoMessage.R[4] = 1;
	    cameraInfoMessage.R[5] = 0;
	    cameraInfoMessage.R[6] = 0;
	    cameraInfoMessage.R[7] = 0;
	    cameraInfoMessage.R[8] = 1;

	    cameraInfoMessage.P[0] = 138.705535888672;
	    cameraInfoMessage.P[1] = 0;
	    cameraInfoMessage.P[2] = 77.2544255212306;
	    cameraInfoMessage.P[3] = 0;
	    cameraInfoMessage.P[4] = 0;
	    cameraInfoMessage.P[5] = 138.954086303711;
	    cameraInfoMessage.P[6] = 58.7000861760043;
	    cameraInfoMessage.P[7] = 0;
	    cameraInfoMessage.P[8] = 0;
	    cameraInfoMessage.P[9] = 0;
	    cameraInfoMessage.P[10] = 1;
	    cameraInfoMessage.P[11] = 0;

	    return cameraInfoMessage;
	}

    }
}
