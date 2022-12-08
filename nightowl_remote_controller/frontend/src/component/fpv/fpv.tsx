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
    }

    render() {
        return <video ref={this.videoTag} controls autoPlay/>;
    }
}