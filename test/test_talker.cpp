/**
 * @file    test_talker.cpp
 * @brief Test for publisher node "talker"
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED
 * "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/StringService.h"

std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief Test if the service was created
 *
 * @param[in] TESTSuite
 * @param[in] testService
 *
 * @return none
 */
TEST(Service, existence) {
  ros::ServiceClient client =
      nh->serviceClient<beginner_tutorials::StringService>("setMessage");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::StringService srv;
  srv.request.message = "test";
  client.call(srv);

  EXPECT_TRUE(srv.response.success);
}

/**
 * @brief Run all the tests that were declared with TEST()
 *
 * @param[in] argc
 * @param[in] argv
 *
 * @return 0 if executed with no errors
 */

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  nh.reset(new ros::NodeHandle);
  return RUN_ALL_TESTS();
}
