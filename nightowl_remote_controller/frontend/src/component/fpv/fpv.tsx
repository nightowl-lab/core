import React from "react";
import { WebRTCService } from "../../service/webrtc.service";

export class FPV extends React.Component {
    private webrtcService = WebRTCService.getInstance();
    public videoTag: React.RefObject<HTMLVideoElement> = React.createRef();

    async componentDidMount() {
        await this.webrtcService!.login("105", "105105105");
        await this.webrtcService!.connectToVehicle("baidu");
        this.webrtcService?.subscribeWebRTCTrackCallback(stream => {
            this.videoTag.current!.srcObject = stream;
        });
        this.webrtcService?.subscribeWebRTCStatsCallback((status, fps, delay, bitrate) => console.log(status, fps, delay, bitrate));
        this.webrtcService?.subscribeVehicleReportCallback((speed, steeringAngle, autowareState, gearState, batteryPercent, vehicleStopMode, emergencyStop, leftTurnIndicator, rightTurnIndicator) => {
            console.log({ speed, steeringAngle, autowareState, gearState, batteryPercent, vehicleStopMode, emergencyStop, leftTurnIndicator, rightTurnIndicator });
        })
    }

    render() {
        return <video ref={this.videoTag} controls autoPlay/>;
    }
}