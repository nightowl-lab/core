import React from 'react';
import './App.css';
import { FPV } from './component/fpv/fpv';
import {DashBoard} from './component/dashboard/dashboard';
import {Debugger} from "./component/debugger/Debugger";

function App() {
  return (
      <div className={'app'}>
          <Debugger className={'debugger-wrapper'}/>
          <DashBoard className={'dashboard-wrapper'}/>
          <FPV className={'fpv-wrapper'}/>

      </div>
  );
}

export default App;
