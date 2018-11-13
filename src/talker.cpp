/**
 * @file    talker.cpp
 * @brief Implementation of publisher node to publish to chatter topic with services to change messages
 * @author RajendraMayavan
 * @copyright MIT License
 *
 * Copyright (c) 2018 Mayavan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <sstream>
#include "beginner_tutorials/StringService.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

//! Message variable Initialization
extern std::string messageToPublish = "Hello";

/**
 * @brief Callback function to set the message to be published to chatter topic
 * @param req input request of service
 * @param req input response of service
 * @return returns true if the message has been set successfully
 */
bool setMessage(beginner_tutorials::StringService::Request &req,
                beginner_tutorials::StringService::Response &resp) {
  ROS_INFO_STREAM("The message has been set to "<< req.message);
  if (req.message.compare("") == 0)
    ROS_ERROR_STREAM("The message has been set to empty string");
  messageToPublish = req.message;
  resp.success = true;
  ROS_DEBUG_STREAM("Service callback has been called");
  return true;
}

/**
 * @brief main function to create the node and publish to chatter topic
 * @param argc Standard main function parameter
 * @param argv Standard main function parameter
 * @return 0 if execution completed successfully
 */
int main(int argc, char **argv) {
  // ROS initialization
  ros::init(argc, argv, "talker");

  // Initialization of Node handle
  ros::NodeHandle n;

  // Initialization of publisher
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Advertise service to change messages
  ros::ServiceServer server = n.advertiseService("setMessage", &setMessage);

  int hertz = 10;

  // Setting the frequency
  if (argc == 2) {
    hertz = atoi(argv[1]);
  }

  ROS_INFO_STREAM("Loop frequency set to "<< hertz);

  // Show error if frequency is negative or very low or very high
  if (hertz < 1) {
    hertz = 1;
    ROS_WARN_STREAM(
        "Loop frequency too low or negative. Frequency set to 1");
  } else if (hertz > 10000) {
    ROS_FATAL("Loop frequency too high. Closing node to reduce CPU usage");
    exit(1);
  }

  // Set the publisher rate
  ros::Rate loop_rate(hertz);

  // Counter to count the message number
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << messageToPublish << "   Count# " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    // Publish message to the chatter topic
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
