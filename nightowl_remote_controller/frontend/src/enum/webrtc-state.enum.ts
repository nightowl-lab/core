export enum WebRTCState {
    /* 等待对端发送Offer */
    WAITING_FOR_OFFER = 1,

    /* 等待连接建立 */
    CONNECTING = 2,

    /* 已连接 */
    CONNECTED = 3,

    /* 连接已断开 */
    DISCONNECTED = 4,
}