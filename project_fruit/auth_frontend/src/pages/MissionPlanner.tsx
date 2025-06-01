// MissionPlanner.tsx

import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  Play,
  Trash2,
  Plus,
  MapPin,
  Info,
  Leaf,
  Cpu,
  Video,
  Camera
} from 'lucide-react';
import { Waypoint, FruitType, ModelType } from '../types';

const fruitTypes: FruitType[] = [
  { id: 'apple', name: 'Apple', color: 'bg-red-500' },
  { id: 'orange', name: 'Orange', color: 'bg-orange-500' },
  { id: 'banana', name: 'Banana', color: 'bg-yellow-500' },
  { id: 'grape', name: 'Grape', color: 'bg-purple-500' },
  { id: 'strawberry', name: 'Strawberry', color: 'bg-pink-500' },
  { id: 'blueberry', name: 'Blueberry', color: 'bg-blue-500' },
];

const modelTypes: ModelType[] = [
  { id: 'fruitdetectv1', name: 'FruitDetect v1', description: 'Basic fruit detection model' },
  { id: 'fruitdetectv2', name: 'FruitDetect v2', description: 'Advanced detection with ripeness estimation' },
  { id: 'citrusnet', name: 'CitrusNet', description: 'Specialized for citrus fruits' },
  { id: 'berryvision', name: 'BerryVision', description: 'Optimized for small berries' },
];

const MissionPlanner = () => {
  const navigate = useNavigate();
  const [missionName, setMissionName] = useState('');
  const [location, setLocation] = useState('');
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedFruitType, setSelectedFruitType] = useState<string>('');
  const [selectedModelType, setSelectedModelType] = useState<string>('');
  const [height, setHeight] = useState<number>(10);
  const [videoEnabled, setVideoEnabled] = useState(false);

  const isFormValid = () => {
    return (
      missionName.trim() !== '' &&
      location.trim() !== '' &&
      waypoints.length >= 2 &&
      selectedFruitType !== '' &&
      selectedModelType !== ''
    );
  };

  const handleStartMission = () => {
    if (!isFormValid()) return;
    const missionId = 'new-mission-' + Date.now();
    navigate(`/drone-details/${missionId}`);
  };

  return (
    <div className="space-y-6 animate-fadeIn">
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-800">Mission Planner</h1>
          <p className="text-gray-600 mt-1">Configure your drone mission parameters</p>
        </div>
        <div className="flex items-center space-x-3 mt-4 sm:mt-0">
          <button
            onClick={handleStartMission}
            disabled={!isFormValid()}
            className="flex items-center gap-2 bg-emerald-600 hover:bg-emerald-700 text-white px-4 py-2 rounded-lg disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <Play size={18} />
            <span>Start Mission</span>
          </button>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Left Panel */}
        <div className="lg:col-span-1 space-y-6">
          {/* Mission Details */}
          <div className="bg-white rounded-lg shadow-sm border">
            <div className="p-4 border-b">
              <h2 className="text-lg font-semibold">Mission Details</h2>
            </div>
            <div className="p-4 space-y-4">
              <input type="text" value={missionName} onChange={(e) => setMissionName(e.target.value)} placeholder="Enter mission name" className="w-full px-3 py-2 border rounded-md" />
              <input type="text" value={location} onChange={(e) => setLocation(e.target.value)} placeholder="Enter location" className="w-full px-3 py-2 border rounded-md" />
              <input type="number" value={height} onChange={(e) => setHeight(Number(e.target.value))} placeholder="Enter Altitude" min={1} max={50} className="w-full px-3 py-2 border rounded-md" />
            </div>
          </div>

          {/* Fruit Type */}
          <div className="bg-white rounded-lg shadow-sm border">
            <div className="p-4 border-b flex items-center">
              <Leaf size={18} className="text-emerald-600 mr-2" />
              <h2 className="text-lg font-semibold">Fruit Type</h2>
            </div>
            <div className="p-4">
              <div className="grid grid-cols-3 gap-2">
                {fruitTypes.map(fruit => (
                  <button
                    key={fruit.id}
                    onClick={() => setSelectedFruitType(fruit.id)}
                    className={`flex items-center p-2 rounded-md border 
                      ${selectedFruitType === fruit.id
                        ? 'border-emerald-500 bg-emerald-50 text-emerald-700'
                        : 'border-gray-300 hover:bg-gray-50'}`}
                  >
                    <div className={`w-3 h-3 rounded-full ${fruit.color} mr-2`} />
                    <span className="text-sm">{fruit.name}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* ML Model */}
          <div className="bg-white rounded-lg shadow-sm border">
            <div className="p-4 border-b flex items-center">
              <Cpu size={18} className="text-blue-600 mr-2" />
              <h2 className="text-lg font-semibold">ML Model</h2>
            </div>
            <div className="p-4">
              <div className="grid grid-cols-2 gap-2">
                {modelTypes.map(model => (
                  <button
                    key={model.id}
                    onClick={() => setSelectedModelType(model.id)}
                    className={`flex flex-col items-start p-2 rounded-md border text-left
                      ${selectedModelType === model.id
                        ? 'border-blue-500 bg-blue-50 text-blue-700'
                        : 'border-gray-300 hover:bg-gray-50'}`}
                  >
                    <span className="font-medium text-sm">{model.name}</span>
                  </button>
                ))}
              </div>
              {selectedModelType && (
                <div className="mt-4 p-3 bg-blue-50 text-blue-800 rounded-md flex">
                  <Info size={18} className="text-blue-500 mr-2 mt-0.5" />
                  <div>
                    <p className="font-medium">
                      {modelTypes.find(m => m.id === selectedModelType)?.name}
                    </p>
                    <p className="text-sm">
                      {modelTypes.find(m => m.id === selectedModelType)?.description}
                    </p>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Right Panel: Video Feed */}
        <div className="lg:col-span-2 space-y-6">
          <div className="bg-white rounded-lg shadow-sm border flex flex-col h-full">
            <div className="p-4 border-b flex items-center">
              <Video size={18} className="text-indigo-600 mr-2" />
              <h2 className="text-lg font-semibold">Video Feed</h2>
            </div>
            <div className="flex-grow">
              {videoEnabled ? (
                <video autoPlay muted loop className="w-full h-[500px] object-cover rounded-b-none">
                  <source src="https://www.w3schools.com/html/mov_bbb.mp4" type="video/mp4" />
                  Your browser does not support the video tag.
                </video>
              ) : (
                <div className="h-[500px] bg-black flex items-center justify-center text-gray-400">
                  Video feed is disabled
                </div>
              )}
            </div>
            <div className="p-4 border-t">
              <button
                onClick={() => setVideoEnabled(!videoEnabled)}
                className="flex items-center gap-2 px-4 py-2 text-white bg-indigo-600 hover:bg-indigo-700 rounded-md"
              >
                <Camera size={16} />
                {videoEnabled ? 'Disable Video' : 'Enable Video'}
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default MissionPlanner;
