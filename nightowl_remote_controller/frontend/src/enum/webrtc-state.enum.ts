export enum WebRTCState {
    /* 等待对端发送Offer */
    WAITING_FOR_OFFER = 1,

    /* 等待连接建立 */
    WAITING_FOR_CONNECT = 2,

    /* 等待媒体流建立 */
    WAITING_FOR_MEDIA_TRACK = 3,

    /* 等待数据流建立 */
    WAITING_FOR_DATA_CHANNEL = 4,

    /* 已连接 */
    CONNECTED = 5,

    /* 连接已断开 */
    DISCONNECTED = 6,
}