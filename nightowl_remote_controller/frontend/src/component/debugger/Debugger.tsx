import React from 'react'
import './debugger.css'

export class Debugger extends React.Component<any, any> {
    render() {
        return (
            <div className={'debugger-wrapper'}>
                <div className={'debugger-item-wrapper'}>
                    <div>Debugger</div>
                    <button>turn left</button>
                    <button>turn right</button>
                </div>
            </div>
        )
    }
}