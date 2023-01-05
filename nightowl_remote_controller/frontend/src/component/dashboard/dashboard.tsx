import React from "react";
import './dashboard.css'
import {ReactComponent as LSL} from './_asset/L_light_cp.svg'
import {ReactComponent as RSL} from './_asset/R_light_cp.svg'


export class DashBoard extends React.Component<any, any> {
    state = {
        status: 'A',
        speedMeterCounter: 0,
        speedMeterUnit: 'KMH',
        gears: ['P', 'R', 'N', 'D'],
        currentGear: 'N'
    }
    render() {
        return (
            <div className='dashboardWrapper'>
                <div className='dashboardStatusWrapper'>
                    <div className='dashboardStatusCircle'>
                        <div className='dashboardStatusTitle'>{this.state.status}</div>
                    </div>
                </div>

                <div className='dashboardSpeedMeterWrapper'>
                    <div className='dashboardSteeringLightWrapper'>
                        <LSL className={'dashboardSteeringLightItem'}/>
                        <div className='dashboardSpeedMeterCounter'>{this.state.speedMeterCounter}</div>
                        <RSL className={'dashboardSteeringLightItem'}/>
                    </div>
                    <div className='dashboardSpeedMeterUnit'>{this.state.speedMeterUnit}</div>
                </div>

                <div className='dashboardGearBoxWrapper'>
                    {
                        this.state.gears.map((value, index) => {
                            return <div key={index} className={value === this.state.currentGear ? 'dashboardGearBoxEngage' : 'dashboardGearBoxDisengage'}>{value}</div>
                        })
                    }
                </div>
            </div>
        )
    }
}
