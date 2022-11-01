#ifndef SMARTER_EYE_CAMERA__TOOL_H__
#define SMARTER_EYE_CAMERA__TOOL_H__

/**
 * 检查SDK返回值,若出错则抛出runtime_error异常
 * @param status 返回值
 * @param msg 异常提示
 */
#define CHECK_SDK_STATUS_OR_THROW(status,  msg) if (!(status)) throw new std::runtime_error(msg);

/**
 * 检查参数变量的值是否在允许范围内,若出错则抛出invalid_argument异常
 * @param param 变量值
 * @param range 范围,格式类似{1, 2, 3}
 * @param msg 异常提示
 */
#define CHECK_PARAM_IN_ARRAY_OR_THROW(param, range, msg) if (!std::set<decltype(param)>range.count(param)) throw new std::invalid_argument(msg);

/**
 * 角度转弧度
 * @param degree 角度
 * @return 弧度
 */
#define DEGREE_TO_RAD(degree) ((degree) / 180.0f * M_PI)

#endif