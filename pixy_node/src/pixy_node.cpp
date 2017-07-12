/*
 * pixy_node is part of the pixy_ros package for interfacing with
 * a CMUcam5 pixy with ROS.
 * Copyright (C) 2014 Justin Eskesen
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>

#include "pixy.h"
#define BLOCK_BUFFER_SIZE 100

class PixyNode
{
public:
	PixyNode();

	void spin();

private:
	void update();
	void setServo(const pixy_msgs::Servo& msg) {pixy_rcs_set_position(msg.channel, msg.position);}
	void interpolateBayer(uint16_t width, uint16_t x, uint16_t y, unsigned char *pixel, uint8_t *r, uint8_t *g, uint8_t *b);
	cv::Mat renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame);
	void getImage();
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;

	ros::Rate rate_;
	tf::TransformBroadcaster tf_broadcaster_;
	
	ros::Publisher publisher_;
	image_transport::ImageTransport it;
	image_transport::Publisher pub_im_;
	ros::Subscriber servo_subscriber_;
	std::string frame_id;

	bool use_servos_;

};

PixyNode::PixyNode() :
		node_handle_(),
		it(node_handle_),
		private_node_handle_("~"),
		use_servos_(false),
		rate_(50.0)
{	

	private_node_handle_.param<std::string>(std::string("frame_id"), frame_id,
			std::string("pixy_frame"));
	pub_im_	= it.advertise("image_raw", 1);
	double rate;
	private_node_handle_.param("rate", rate, 50.0);
	rate_=ros::Rate(rate);

    private_node_handle_.param("use_servos", use_servos_, false);

    if(use_servos_)
    {
        servo_subscriber_ = node_handle_.subscribe("servo_cmd", 20, &PixyNode::setServo, this);
    }

	int ret = pixy_init();
	if (ret != 0)
	{
		ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
				__FUNCTION__, ret);
		ROS_BREAK();
	}
    publisher_ = node_handle_.advertise<pixy_msgs::PixyData>("block_data", 50.0);


}



void PixyNode::update()
{

	// Pixy Block buffer //
	struct Block blocks[BLOCK_BUFFER_SIZE];

	// Get blocks from Pixy //
	int blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, blocks);

	pixy_msgs::PixyData data;

	if (blocks_copied > 0)
	{
		data.header.stamp = ros::Time::now();
		for (int i = 0; i < blocks_copied; i++)
		{
			pixy_msgs::PixyBlock pixy_block;
			pixy_block.type = blocks[i].type;
			pixy_block.signature = blocks[i].signature;
			pixy_block.roi.x_offset = blocks[i].x;
			pixy_block.roi.y_offset = blocks[i].y;
			pixy_block.roi.height = blocks[i].height;
			pixy_block.roi.width = blocks[i].width;
			pixy_block.roi.do_rectify = false;
			pixy_block.angle =
					(pixy_block.type == TYPE_COLOR_CODE) ?
							angles::from_degrees((double) blocks[i].angle) :
							0.0;

			data.blocks.push_back(pixy_block);
		}

	}
	else if(blocks_copied < 0)
	{
		ROS_INFO("Pixy read error.");
		return;
	}

	// publish the message
	publisher_.publish(data);
	getImage();
}


void PixyNode::spin()
{

	while (node_handle_.ok())
	{
		update();

		ros::spinOnce();
		rate_.sleep();
	}
}


void PixyNode::getImage()
{
    unsigned char *pixels;
    int32_t response, fourcc;
    int8_t renderflags;
    //int return_value, res;
    int return_value;
    uint16_t rwidth, rheight;
    uint32_t  numPixels;
    //uint16_t height,width;
    //uint16_t mode;

    return_value = pixy_command("run", END_OUT_ARGS, &response, END_IN_ARGS);
    return_value = pixy_command("stop", END_OUT_ARGS, &response, END_IN_ARGS);

    return_value = pixy_command("cam_getFrame",  // String id for remote procedure
                                 0x01, 0x21,      // mode
                                 0x02,   0,        // xoffset
                                 0x02,   0,         // yoffset
                                 0x02, 320,       // width
                                 0x02, 200,       // height
                                 0,            // separator
                                 &response, &fourcc, &renderflags, &rwidth, &rheight, &numPixels, &pixels, 0);

    cv::Mat result = renderBA81(renderflags,rwidth,rheight,numPixels,pixels);

	cv_bridge::CvImage im;
	im.image = result;
	im.encoding = sensor_msgs::image_encodings::BGR8;
	im.header.frame_id = "pixy_frame";	
	im.header.stamp = ros::Time::now();
	
     pub_im_.publish(im.toImageMsg());
}

inline void PixyNode::interpolateBayer(uint16_t width, uint16_t x, uint16_t y, uint8_t *pixel, uint8_t *r, uint8_t *g, uint8_t * b)
{
    if (y&1)
    {
        if (x&1)
        {
            *r = *pixel;
            *g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            *b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
        }
        else
        {
            *r = (*(pixel-1)+*(pixel+1))>>1;
            *g = *pixel;
            *b = (*(pixel-width)+*(pixel+width))>>1;
        }
    }
    else
    {
        if (x&1)
        {
            *r = (*(pixel-width)+*(pixel+width))>>1;
            *g = *pixel;
            *b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            *r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
            *g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            *b = *pixel;
        }
    }

}

cv::Mat PixyNode::renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
    uint16_t x, y;
    uint8_t r, g, b;
    cv::Mat imageRGB;

    frame += width;
    uchar data[3*((height-2)*(width-2))];

    uint m = 0;
    for (y=1; y<height-1; y++)
    {
        frame++;
        for (x=1; x<width-1; x++, frame++)
        {
            interpolateBayer(width, x, y, frame, &r, &g, &b);
            data[m++] = b;
            data[m++] = g;
            data[m++] = r;
        }
        frame++;
    }

    imageRGB =  cv::Mat(height - 2,width -2, CV_8UC3, data);

    return imageRGB;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pixy_node");

	ROS_INFO("PixyNode for ROS");

	PixyNode myPixy;
	myPixy.spin();

	return (0);
}

// EOF
