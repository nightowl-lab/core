import React from 'react';
import './App.css';
import { FPV } from './component/fpv/fpv';
import {DashBoard} from './component/dashboard/dashboard';

function App() {
  return (
      <div>
        <DashBoard/>
        <FPV />
      </div>
  );
}

export default App;
