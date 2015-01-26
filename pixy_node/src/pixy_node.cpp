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
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <std_msgs/Float32.h>

#include "pixy.h"
#define BLOCK_BUFFER_SIZE 100

class PixyNode
{
public:
	PixyNode();

	void spin();

private:
	void update();

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;

	ros::Rate rate_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher publisher_;
	std::string frame_id;

	class Servo
	{
	public:
		Servo(uint8_t channel, double range_min=-M_PI, double range_max=M_PI) : channel_(channel){setRange(range_min, range_max);}
		void subscribe(ros::NodeHandle& nh)
		{
			char topic[64];
			sprintf(topic, "servo_%d_cmd", channel_);
			sub_ = nh.subscribe(std::string(topic), 1, &PixyNode::Servo::set, this);
		}

		void set(const std_msgs::Float32ConstPtr& msg)
		{

			float value = msg->data;
			value = (value > range_max_) ? range_max_ : value;
			value = (value < range_min_) ? range_min_ : value;

			//ROS_INFO("in=%f, out=%d", msg->data, (uint16_t) (value*scale_ + shift_));
			pixy_rcs_set_position(channel_, (uint16_t) (value*scale_ + shift_));
		}

		void setRange(float range_min, float range_max)
		{
			range_min_ = range_min;
			range_max_ = range_max;
			scale_ = 1000.0/(range_max_ - range_min_);
			shift_ = ((range_max_ + range_min_)/2.0) + 500.0;
			ROS_INFO("scale: %f, shift %f", scale_, shift_);


		}
	private:
		uint8_t channel_;
		float range_min_, range_max_, scale_, shift_;
		ros::Subscriber sub_;

	};

	Servo servo_0_;
	Servo servo_1_;
	bool use_servos_;
	//ros::Subscriber servo_1_subscriber_;


};

PixyNode::PixyNode() :
		node_handle_(),
		private_node_handle_("~"),
		use_servos_(false),
		rate_(50.0),
		servo_0_(0),
		servo_1_(1)
{

	private_node_handle_.param<std::string>(std::string("frame_id"), frame_id,
			std::string("pixy_frame"));

	double rate;
	private_node_handle_.param("rate", rate, 50.0);
	rate_=ros::Rate(rate);

    private_node_handle_.param("use_servos", use_servos_, false);

    double servo_min, servo_max;
    private_node_handle_.param("servo_0_min", servo_min, -M_PI);
    private_node_handle_.param("servo_0_max", servo_max, M_PI);
    servo_0_.setRange(servo_min, servo_max);

    private_node_handle_.param("servo_1_min", servo_min, -M_PI);
    private_node_handle_.param("servo_1_max", servo_max, M_PI);
    servo_1_.setRange(servo_min, servo_max);

    if(use_servos_)
    {
    	servo_0_.subscribe(node_handle_);
    	servo_1_.subscribe(node_handle_);
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pixy_node");

	ROS_INFO("PixyNode for ROS");

	PixyNode myPixy;
	myPixy.spin();

	return (0);
}

// EOF
