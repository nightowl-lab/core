// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BAIDU_JOY_CONTROLLER__DS4_JOY_CONVERTER_HPP_
#define BAIDU_JOY_CONTROLLER__DS4_JOY_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"

#include <algorithm>

namespace baidu_joy_controller
{
class DS4JoyConverter
{
  public:
    DS4JoyConverter() {}
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) { j_ = *msg; }

    /**
     * 发布LED消息
     *
     * @param 消息发布者
     * @param red
     * @param green
     * @param blue
     */
    void publishLEDMessage(rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr publisher, uint8_t red, uint8_t green, uint8_t blue)
    {
        sensor_msgs::msg::JoyFeedbackArray msg;
        msg.array.resize(3);
        msg.array[0].type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
        msg.array[0].id = 0;
        msg.array[0].intensity = red / 255.0f;

        msg.array[1].type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
        msg.array[1].id = 1;
        msg.array[1].intensity = green / 255.0f;

        msg.array[2].type = sensor_msgs::msg::JoyFeedback::TYPE_LED;
        msg.array[2].id = 2;
        msg.array[2].intensity = blue / 255.0f;
        publisher->publish(msg);
    }

    float LStickLeftRight() const { return j_.axes.at(0); }
    float LStickUpDown() const { return j_.axes.at(1); }
    float LTrigger() const { return j_.axes.at(4) == 0.0 ? 0.0 : (-j_.axes.at(4) / 2.0f + 0.5); }
    float RStickLeftRight() const { return j_.axes.at(2); }
    float RStickUpDown() const { return j_.axes.at(3); }
    float RTrigger() const { return j_.axes.at(5) == 0.0 ? 0.0 : (-j_.axes.at(5) / 2.0f + 0.5); }

    bool Cross() const { return j_.buttons.at(0); }
    bool Circle() const { return j_.buttons.at(1); }
    bool Triangle() const { return j_.buttons.at(3); }
    bool Square() const { return j_.buttons.at(2); }
    bool L1() const { return j_.buttons.at(9); }
    bool R1() const { return j_.buttons.at(10); }
    bool Share() const { return j_.buttons.at(4); }
    bool Options() const { return j_.buttons.at(6); }
    bool PS() const { return j_.buttons.at(5); }
    bool LStickPress() const { return j_.buttons.at(7); }
    bool RStickPress() const { return j_.buttons.at(8); }
    bool CursorTop() const { return j_.buttons.at(11); }
    bool CursorBottom() const { return j_.buttons.at(12); }
    bool CursorLeft() const { return j_.buttons.at(13); }
    bool CursorRight() const { return j_.buttons.at(14); }

  private:
    sensor_msgs::msg::Joy j_;
};
}  // namespace baidu_joy_controller

#endif  // JOY_CONTROLLER__JOY_CONVERTER__DS4_JOY_CONVERTER_HPP_
