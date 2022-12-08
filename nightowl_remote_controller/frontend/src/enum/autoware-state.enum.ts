export enum AutowareState {
    /* 正在初始化 */
    INITIALIZING = 1,

    /* 等待路线 */
    WAITING_FOR_ROUTE = 2,

    /* 正在规划 */
    PLANNING = 3,

    /* 等待出发指令 */
    WAITING_FOR_ENGAGE = 4,

    /* 正在自动驾驶 */
    DRIVING = 5,

    /* 到达目标 */
    ARRIVED_GOAL = 6,

    /* 正在完成 */
    FINALIZING = 7,

    /* 手动模式 */
    MANUAL = 100
}