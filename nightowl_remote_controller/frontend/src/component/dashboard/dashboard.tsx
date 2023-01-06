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
        currentGear: 'N',
        isLSL: false,
        isRSL: false,
    }

    public switchSteeringLight = (which: string = 'L', state: boolean = false) => {
        if (which === 'L') {
            this.setState({
                isLSL: state
            })
        } else if (which === 'R') {
            this.setState({
                isRSL: state
            })
        }
    }

    render() {
        return (
            <div className='dashboard-wrapper'>
                <div className='dashboard-status-wrapper'>
                    <div className='dashboard-status-circle'>
                        <div className='dashboard-status-title'>{this.state.status}</div>
                    </div>
                </div>

                <div className='dashboard-speedMeter-wrapper'>
                    <div className='dashboard-steeringLight-wrapper'>
                        <LSL
                            className={this.state.isLSL ? 'dashboard-steeringLight-item-ON' : 'dashboard-steeringLight-item-OFF'}/>
                        <div className='dashboard-speedMeter-counter'>{this.state.speedMeterCounter}</div>
                        <RSL
                            className={this.state.isRSL ? 'dashboard-steeringLight-item-ON' : 'dashboard-steeringLight-item-OFF'}/>
                    </div>
                    <div className='dashboard-speedMeter-unit'>{this.state.speedMeterUnit}</div>
                </div>

                <div className='dashboard-gearBox-wrapper'>
                    {
                        this.state.gears.map((value, index) => {
                            return <div key={index}
                                        className={value === this.state.currentGear ? 'dashboard-gearBox-engage' : 'dashboard-gearBox-disengage'}>{value}</div>
                        })
                    }
                </div>
            </div>
        )
    }
}
