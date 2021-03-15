/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <jetson-utils/gstCamera.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include "image_converter.h"
#include <memory>

// globals	
gstCamera* camera = NULL;
std::string CAMERA_FRAME{"pantilt_camera_optical_frame"};
std::string CAMERA_NAME{"pantilt_camera"};

imageConverter* camera_cvt = NULL;
image_transport::CameraPublisher* it_pub;

/** camera calibration information */
std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr;

// aquire and publish camera frame
bool aquireFrame()
{
	float4* imgRGBA = NULL;

	// get the latest frame
	if( !camera->CaptureRGBA((float**)&imgRGBA, 1000) )
	{
		ROS_ERROR("failed to capture camera frame");
		return false;
	}

	// assure correct image size
	if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), IMAGE_RGBA32F) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;
    const sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(cam_info_mgr->getCameraInfo()));

	if( !camera_cvt->Convert(msg, imageConverter::ROSOutputFormat, imgRGBA) )
	{
		ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
		return false;
	}

	// publish the message
    info->header.frame_id = CAMERA_FRAME;
    msg.header.frame_id = CAMERA_FRAME;
    it_pub->publish(msg, *info, ros::Time::now());
    //std::cout << *info;
	// RKJ: Don't do a log entry on each frame
	//ROS_INFO("published camera frame");
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, CAMERA_NAME);
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

    /** camera calibration information */
    cam_info_mgr = std::unique_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh));

	/*
	 * retrieve parameters
	 */
	std::string camera_device = "0";	// MIPI CSI camera by default

	private_nh.param<std::string>("device", camera_device, camera_device);
	
	ROS_INFO("opening camera device %s", camera_device.c_str());
    cam_info_mgr->setCameraName(CAMERA_NAME);
	
	/*
	 * open camera device
	 */
	camera = gstCamera::Create(camera_device.c_str());
    image_transport::ImageTransport it_(nh);

    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(cam_info_mgr->getCameraInfo()));

	if( !camera )
	{
		ROS_ERROR("failed to open camera device %s", camera_device.c_str());
		return 0;
	}


	/*
	 * create image converter
	 */
	camera_cvt = new imageConverter();

	if( !camera_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
    image_transport::CameraPublisher it_publisher = it_.advertiseCamera(CAMERA_NAME + "/image_raw", 1);
    it_pub = &it_publisher;

	/*
	 * start the camera streaming
	 */
	if( !camera->Open() )
	{
		ROS_ERROR("failed to start camera streaming");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	while( ros::ok() )
	{
		//if( raw_pub->getNumSubscribers() > 0 )
			aquireFrame();

		ros::spinOnce();
	}

	delete camera;
	return 0;
}

