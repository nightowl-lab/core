#ifndef BAIDU_JOY_CONTROLLER__TOOL_HPP_
#define BAIDU_JOY_CONTROLLER__TOOL_HPP_

/**
 * LED闪烁工具宏
 * @param lastStatus 上一次LED状态
 * @param lastTime 上一次LED闪烁时间
 * @param nodePtr node指针
 * @param joy 摇杆类
 * @param publisher 摇杆反馈发布者
 * @param R LED R颜色
 * @param G LED G颜色
 * @param B LED B颜色
 * @param duration 闪烁间隔
 */
#define LED_BLINK(lastStatus, lastTime, nodePtr, joy, publisher, R, G, B, duration) { \
    if ((lastTime) + duration < (nodePtr)->get_clock()->now()) { \
        if ((lastStatus) = !(lastStatus)) { \
            (joy).publishLEDMessage((publisher), (R), (G), (B)); \
        } else { \
            (joy).publishLEDMessage((publisher), 0, 0, 0); \
        }\
        (lastTime) = (nodePtr)->get_clock()->now(); \
    }\
}

/**
 * 检查摇杆按钮的上升沿
 * @param joy 当次摇杆信息
 * @param lastJoy 上次摇杆信息
 * @param button 按钮名称
 * 
 */
#define CHECK_JOY_BUTTON_FROM_RELEASE_TO_PRESS(joy, lastJoy, button) ((lastJoy).button() == 0 && (joy).button() == 1)

#endif