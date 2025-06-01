import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { Toaster } from 'react-hot-toast';
import Login from './pages/Login';
import Signup from './pages/Signup';
import Dashboard from './pages/Dashboard';
import MissionPlanner from './pages/MissionPlanner';
import DroneDetails from './pages/DroneDetails';

import Layout from './components/Layout';

function App() {
  return (
    <Router>
      <Toaster position="top-right" />

      <Routes>
        <Route path="/login" element={<Login />} />
        <Route path="/signup" element={<Signup />} />
        <Route path="/" element={<Layout />}>
          <Route index element={<Dashboard />} />
          <Route path="mission-planner" element={<MissionPlanner />} />
          <Route path="drone-details/:missionId" element={<DroneDetails />} />
        </Route>
      </Routes>
    </Router>
  );
}

export default App;