import { environment } from "../environment";
import { WebSocketException } from "../exception/websocket.exception";
import { ToolService } from "./tool.service";
import { AutowareState } from "../enum/autoware-state.enum";
import { GearState } from "../enum/gear-state.enum";
import { WebRTCState } from "../enum/webrtc-state.enum";
import { WebRTCRPCType } from "../enum/webrtc-rpc-type.enum";
import { createInstance } from "../singleton";
import { InvalidArgumentsException } from "../exception/invalid-arguments.exception";


type onWebRTCRPCMessageCallback = (isCall: boolean, callID: number, code: number, data: Uint8Array) => void;
type OnWebSocketMessageCallback = (type: string, from: string, callID: number, data: any) => void;
type OnWebRTCTrackCallback = (stream: MediaStream) => void;
type OnVehicleReport = (speed: number, steeringAngle: number, autowareState: AutowareState, gearState: GearState, batteryPercent: number, vehicleStopMode: boolean, emergencyStop: boolean, leftTurnIndicator: boolean, rightTurnIndicator: boolean) => void;
type OnWebRTCStatsCallback = (status: WebRTCState, fps: number, delay: number, bitrate: number) => void;

export class WebRTCService {
    static getInstance = createInstance(WebRTCService);
    private initPromise: Promise<void>;
    private toolService: ToolService = ToolService.getInstance();
    private static UNRELIABLE_VHICLE_REPORT_SOF = 0xAA;
    private static UNRELIABLE_JOYSTICK_SOF = 0xBB;
    private static RELIABLE_SOF_RPC_CALL = 0xCC;
    private static RELIABLE_SOF_RPC_RETURN = 0xDD;
    private rtc?: RTCPeerConnection;
    private ws?: WebSocket;
    private mediaTrack?: MediaStreamTrack;
    private reliableDataChannel?: RTCDataChannel;
    private unreliableDataChannel?: RTCDataChannel;
    private onWebRTCRPCMessageCallback: onWebRTCRPCMessageCallback[] = [];
    private onWebSocketMeesageCallback: OnWebSocketMessageCallback[] = [];
    private onWebRTCTrackCallback: OnWebRTCTrackCallback[] = [];
    private onVehicleReportCallback: OnVehicleReport[] = [];
    private onWebRTCStatsCallback: OnWebRTCStatsCallback[] = [];

    constructor() {
        this.initPromise = this.onApplicationBootstrap();
    }

    private async onApplicationBootstrap() {
        this.rtc = new RTCPeerConnection({
            bundlePolicy: "max-bundle",
            iceServers: environment.iceServers.map(url => {
                let regex = RegExp(/^(([^:.@/?#]+):)?(\/{0,2}((([^:@]*)(:([^@]*))?)@)?(([^:\/?#]*)(:([^\/?#]*))?))?([^?#]*)(\\?([^#]*))?(#(.*))?/);
	            var matches =  url.match(regex);
                if (matches === null || matches.length != 18) {
                    throw new InvalidArgumentsException("iceServers");
                }
                let newURL = matches[1] + matches[9], username = matches[6], password = matches[8];
                return {
                    urls: newURL,
                    username,
                    credential: password
                };
            })
        });
        this.rtc.ontrack = this.onWebRTCTrack.bind(this);
        this.rtc.ondatachannel = this.onWebRTCDataChannelOpen.bind(this);
        this.ws = new WebSocket(environment.websocketServer);
        this.ws.onmessage = this.onWebSocketMessage.bind(this);
        await this.waitWebSocketConnect();
        /* 注册回调 */
        this.onWebSocketMeesageCallback.push(this.onOfferReceived.bind(this));
        /* 初始化统计计时器 */
        await this.initWebRTCStatsInterval();
    }

    /**
     * 等待WebRTC Gathering连接
     */
    private waitGatheringComplete() {
        return new Promise<void>((resolve) => {
            if (this.rtc?.iceGatheringState === 'complete') {
                resolve();
            } else {
                let callback: () => void;
                this.rtc?.addEventListener('icegatheringstatechange', callback = () => {
                    if (this.rtc?.iceGatheringState === 'complete') {
                        this.rtc?.removeEventListener('icegatheringstatechange', callback);
                        resolve();
                    }
                });
            }
        });
    }

    /**
     * 等待WebSocket连接
     */
    private waitWebSocketConnect() {
        return new Promise<void>((resolve) => {
            if (this.ws?.readyState === WebSocket.OPEN) {
                resolve();
            } else {
                let callback: () => void;
                this.ws?.addEventListener('open', callback = () => {
                    this.ws?.removeEventListener('open', callback);
                    resolve();
                });
            }
        });
    }

    /**
     * WebRTC媒体Track打开回调
     * @param event rtc事件
     */
    private onWebRTCTrack(event: RTCTrackEvent) {
        console.log(`[WebRTC] Received video track, length: ${event.streams.length}`);
        this.mediaTrack = event.track
        for (const callback of this.onWebRTCTrackCallback) {
            callback(event.streams[0]);
        }
    }

    /**
     * WebRTC数据通道打开回调
     * @param event rtc事件
     */
    private onWebRTCDataChannelOpen(event: RTCDataChannelEvent) {
        event.channel.onmessage = msgEvent => {
            const byteArray = new Uint8Array(msgEvent.data);
            /* 验证长度和SOF合法性 */
            let pass = true;
            if (event.channel.label === 'unreliable') {
                if (!(byteArray[0] === WebRTCService.UNRELIABLE_VHICLE_REPORT_SOF && byteArray.length === 16)) pass = false;
            } else if (event.channel.label === 'reliable') {
                if (!([WebRTCService.RELIABLE_SOF_RPC_CALL, WebRTCService.RELIABLE_SOF_RPC_RETURN].includes(byteArray[0]) && byteArray.length >= 5 )) pass = false;
            } else {
                pass = false;
            }
            /* 验证CRC校验码 */
            if (pass && this.toolService!.calculateCRC8(byteArray.slice(0, byteArray.length - 1)) !== byteArray[byteArray.length - 1]) pass = false;
            /* 错误处理 */
            if (!pass) {
                console.error(`[WebRTC] Failed to parse data channel message, length: ${byteArray.length}`);
                return;
            }
            if (event.channel.label === 'unreliable') {
                this.onWebRTCRawVehicleReport(byteArray.slice(1, byteArray.length - 2));
            } else if (event.channel.label === 'reliable') {
                const callID = (byteArray[1] << 8) | byteArray[2];
                for (const callback of this.onWebRTCRPCMessageCallback) {
                    callback(byteArray[0] === WebRTCService.RELIABLE_SOF_RPC_CALL, callID, byteArray[3], byteArray.slice(4));
                }
            }
        };
        event.channel.onclose = () => {
            if (event.channel.label === 'reliable') {
                this.reliableDataChannel = undefined;
            } else if (event.channel.label === 'unreliable') {
                this.unreliableDataChannel = undefined;
            }
            console.log(`[WebRTC] DataChannel: ${event.channel.label} Closed!`);
        };
        if (event.channel.label === 'reliable') {
            this.reliableDataChannel = event.channel;
        } else if (event.channel.label === 'unreliable') {
            this.unreliableDataChannel = event.channel;
        }
        console.log(`[WebRTC] DataChannel: ${event.channel.label} Open!`);
    }


    /**
     * 初始化WebRTC统计定时器
     */
     private async initWebRTCStatsInterval() {
        let lastRTPStatsTime = -1, lastRTPBytesReceived = 0, lastRTPFramesDecoded = -1;
        setInterval(async () => {
            let delay = 0, bitrate = 0, fps = 0, state = WebRTCState.DISCONNECTED;
            /* 获取当前RTC状态 */
            if (!this.rtc?.remoteDescription) {
                state = WebRTCState.WAITING_FOR_OFFER;
            } else if (this.rtc?.connectionState === 'connecting') {
                state = WebRTCState.CONNECTING;
            }
            if (this.rtc?.connectionState === 'connected' &&
                this.reliableDataChannel?.readyState === 'open' &&
                this.unreliableDataChannel?.readyState === 'open' && 
                this.mediaTrack?.readyState === 'live') {
                state = WebRTCState.CONNECTED;
                /* 测速PING延迟 */
                if (this.reliableDataChannel !== undefined) {
                    const start = new Date().getTime();
                    await this.makeWebRTCRPCCall(WebRTCRPCType.PING, new Uint8Array(0));
                    const stop = new Date().getTime();
                    delay = stop - start;
                }
                /* 获取FPS和波特率 */
                if (this.mediaTrack !== undefined) {
                    const mediaStats = await this.rtc?.getStats(this.mediaTrack);
                    mediaStats.forEach(report => {
                        if (report.type !== 'inbound-rtp') return;
                        const deltaTime = (report.timestamp - lastRTPStatsTime) / 1000;
                        bitrate = (report["bytesReceived"] - lastRTPBytesReceived) / deltaTime * 8;
                        lastRTPBytesReceived = report["bytesReceived"];
                        fps = (report["framesDecoded"] - lastRTPFramesDecoded) / deltaTime;
                        lastRTPFramesDecoded = report["framesDecoded"];
                        lastRTPStatsTime = report.timestamp;
                    });
                }
            }
            /* 调用回调 */
            for (const callback of this.onWebRTCStatsCallback) {
                callback(state, fps, delay, bitrate);
            }
        }, 1000);
    }

    /**
     * 发起WebRTC RPC调用
     * @param type 调用类型
     * @param data 参数
     * @returns 返回值和数据
     */
    private makeWebRTCRPCCall(type: WebRTCRPCType, data: Uint8Array) {
        const sendCallID = Math.floor(Math.random() * 65535);
        const sendData = new Uint8Array(data.length + 5);
        sendData[0] = WebRTCService.RELIABLE_SOF_RPC_CALL;
        sendData[1] = sendCallID >> 8;
        sendData[2] = sendCallID & 0xFF;
        sendData[3] = type;
        sendData.set(data, 4);
        sendData[sendData.length - 1] = this.toolService!.calculateCRC8(sendData.slice(0, sendData.length - 1));
        this.reliableDataChannel?.send(sendData);
        /* 等待RPC调用结束 */
        return new Promise<{ code: number; data: Uint8Array }>(resolve => {
            let index = this.onWebRTCRPCMessageCallback.push((isCall: boolean, callID: number, code: number, data: Uint8Array) => {
                if (isCall || callID !== sendCallID) return;
                delete this.onWebRTCRPCMessageCallback[index];
                resolve({ code, data });
            });
        });
    }

    /**
     * 接受到车辆报告回调
     * @param data 原始字节数据
     */
    private onWebRTCRawVehicleReport(data: Uint8Array) {
        const speed = ((data[0] << 8) | data[1]) / 100;
        let steeringAngle = (data[2] << 8) | data[3];
        /* steeringAngle有符号 */
        steeringAngle = (steeringAngle >= 32768 ? 65536 - steeringAngle : steeringAngle) / 100;
        const autowareState: AutowareState = data[4];
        const gearState: GearState = data[5];
        const batteryPercent = data[6];
        const vehicleStopMode: boolean = (data[7] & 0x01) === 1;
        const emergencyStop: boolean = (data[7] & 0x02) === 1;
        const leftTurnIndicator: boolean = (data[7] & 0x04) === 1;
        const rightTurnIndicator: boolean = (data[7] & 0x08) === 1;
        for (const callback of this.onVehicleReportCallback) {
            callback(speed, steeringAngle, autowareState, gearState, batteryPercent, vehicleStopMode, emergencyStop, leftTurnIndicator, rightTurnIndicator);
        }
    }

    /**
     * WebSocket接受到消息回调
     * @param event 事件
     */
    private onWebSocketMessage(event: MessageEvent) {
        let jsonData: any;
        try {
            jsonData = JSON.parse(event.data);
            if (typeof jsonData['from'] !== 'string' || typeof jsonData['callID'] !== 'number' ||
                typeof jsonData['type'] !== 'string' || typeof jsonData['data'] === 'undefined') {
                throw new Error();
            }
        } catch(e) {
            console.error(`[WebRTC] Failed to parse json from server!`);
            return;
        }
        for (const callback of this.onWebSocketMeesageCallback) {
            callback(jsonData.type, jsonData.from, jsonData.callID, jsonData.data);
        }
    }

    /**
     * 信令服务器处理Offer回调
     * @param from 来源
     * @param type 调用类型
     * @param callID 调用ID
     * @param data 附加参数
     * @returns 返回数据
     */
    private async onOfferReceived(type: string, from: string, callID: number, data: any) {
        if (type !== 'offer') return;
        console.log("[WebSocket] Got offer!");
        await this.rtc?.setRemoteDescription({
            type: 'offer',
            sdp: data[0]
        });
        await this.rtc?.setLocalDescription(await this.rtc?.createAnswer());
        await this.waitGatheringComplete();
        if (this.rtc?.localDescription === null) {
            console.error("[WebRTC] Failed to get answer");
            return;
        }
        const answer = this.rtc?.localDescription.sdp;
        console.log("[WebRTC] Got answer!");
        this.makeWebSocketRPCReturn(from, callID, 0, { sdp: answer });
    }

    /**
     * 发送WebSocket返回值
     * @param to 目标
     * @param type 调用类型
     * @param code 返回值
     * @param data 返回数据
     */
    private makeWebSocketRPCReturn(to: string, callID: number, code: number, data: any) {
        this.ws?.send(JSON.stringify({ to, type: 'response', callID, data: { code, ...data } }));
    }

    /**
     * 发起WebSocket RPC调用
     * @param to 目标
     * @param type 调用类型
     * @param data 附加参数
     * @returns 返回数据
     */
    private makeWebSocketRPCCall(to: string, type: string, data: any) {
        const callID = Math.floor(Math.random() * 2147483647);
        this.ws?.send(JSON.stringify({ to, type, callID, data }));
        return new Promise<any>((resolve, reject) => {
            let callback: (event: MessageEvent) => void;
            this.ws?.addEventListener('message', callback = (event: MessageEvent) => {
                let jsonData;
                try {
                    jsonData = JSON.parse(event.data);
                    if (!jsonData.from || !jsonData.callID || !jsonData.type || (jsonData.type === "response" && jsonData.data.code === undefined)) {
                        throw new Error();
                    }
                } catch(e) {
                    console.error(`[WebSocket] Cannot parse json data`);
                    return;
                }
                if ((jsonData.from !== "server" && jsonData.from !== to) || jsonData.callID !== callID || jsonData.type !== "response") return;
                this.ws?.removeEventListener('message', callback);
                if (jsonData.data.code !== 0) {
                    reject(new WebSocketException(jsonData.data.code));
                } else {
                    delete jsonData.data.code;
                    resolve(jsonData.data);
                }
            });
        });
    }

    /**
     * 登录WebSocket服务器
     * @param username 用户名
     * @param password 密码
     */
    async login(username: string, password: string) {
        await this.initPromise;
        console.log(`[WebSocket] Logging to server`);
        await this.makeWebSocketRPCCall('server', 'login', {
            username,
            password,
            role: 'USER'
        });
        console.log(`[WebSocket] Login successful!`);
    }

    /**
     * 连接车辆
     * @param target 目标车辆
     */
    async connectToVehicle(target: string) {
        await this.initPromise;
        console.log(`[WebSocket] Requesting offer from vehicle`);
        await this.makeWebSocketRPCCall(target, 'request', {});
        console.log(`[WebSocket] Request offer successful!`);
    }

    /**
     * 发送摇杆信息
     * @param channels 摇杆通道
     * @param buttons 按钮
     */
    async sendJoystickMessage(channels: number[], buttons: boolean[]) {
        await this.initPromise;
        const sendData = new Uint8Array(channels.length * 2 + Math.ceil(buttons.length / 8) + 1);
        sendData[0] = WebRTCService.UNRELIABLE_JOYSTICK_SOF;
        sendData[1] = channels.length;
        let index = 2;
        for (const channel of channels) {
            const channelValue = Math.ceil(channel * 65535);
            sendData[index++] = channelValue >> 8;
            sendData[index++] = channelValue & 0xFF;
        }
        let buttonPosition = 0;
        for (const button of buttons) {
            sendData[index + Math.floor(buttonPosition / 8)] = (button ? 1 : 0) << (buttonPosition % 8);
            buttonPosition++;
        }
        sendData[sendData.length] = this.toolService!.calculateCRC8(sendData.slice(0, sendData.length - 2));
        this.unreliableDataChannel?.send(sendData);
    }

    /**
     * 订阅WebRTC媒体Track接受回调
     * @param callback 回调函数
     */
    subscribeWebRTCTrackCallback(callback: OnWebRTCTrackCallback) {
        this.onWebRTCTrackCallback.push(callback);
    }

    /**
     * 订阅车辆报告回调
     * @param callback 回调函数
     */
    subscribeVehicleReportCallback(callback: OnVehicleReport) {
        this.onVehicleReportCallback.push(callback);
    }

    /**
     * 订阅WebRTC统计报告回调
     * @param callback 回调函数
     */
    subscribeWebRTCStatsCallback(callback: OnWebRTCStatsCallback) {
        this.onWebRTCStatsCallback.push(callback);
    }
}