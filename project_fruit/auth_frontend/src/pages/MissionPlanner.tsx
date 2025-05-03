import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  Save, 
  Play, 
  Trash2, 
  Plus, 
  MapPin, 
  X,
  Info,
  Leaf,
  Cpu,
  Map as MapIcon
} from 'lucide-react';
import MapComponent from '../components/MapComponent';
import { Waypoint, FruitType, ModelType } from '../types';

// Sample data for dropdowns
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
  { id: 'appleinspect', name: 'Apple Inspector', description: 'Apple-specific with disease detection' },
];

const MissionPlanner = () => {
  const navigate = useNavigate();
  const [missionName, setMissionName] = useState('');
  const [location, setLocation] = useState('');
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedFruitType, setSelectedFruitType] = useState<string>('');
  const [selectedModelType, setSelectedModelType] = useState<string>('');
  const [height, setHeight] = useState<number>(10);
  const [speed, setSpeed] = useState<number>(5);
  
  // Form validation
  const isFormValid = () => {
    return (
      missionName.trim() !== '' &&
      location.trim() !== '' &&
      waypoints.length >= 2 &&
      selectedFruitType !== '' &&
      selectedModelType !== ''
    );
  };
  
  const handleAddWaypoint = (waypoint: Waypoint) => {
    setWaypoints([...waypoints, waypoint]);
  };
  
  const handleRemoveWaypoint = (index: number) => {
    const updatedWaypoints = [...waypoints];
    updatedWaypoints.splice(index, 1);
    setWaypoints(updatedWaypoints);
  };
  
  const handleStartMission = () => {
    if (!isFormValid()) return;
    
    // In a real application, you would save the mission data to your backend
    const missionId = 'new-mission-' + Date.now();
    navigate(`/drone-details/${missionId}`);
  };
  
  const handleSaveMission = () => {
    if (!isFormValid()) return;
    
    // Save mission logic would go here
    alert('Mission saved successfully');
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
            onClick={handleSaveMission}
            disabled={!isFormValid()}
            className="flex items-center gap-2 border border-gray-300 px-4 py-2 rounded-lg text-gray-700 hover:bg-gray-50 disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <Save size={18} />
            <span>Save</span>
          </button>
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
        {/* Mission details form */}
        <div className="lg:col-span-1 space-y-6">
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
            <div className="p-4 border-b border-gray-200">
              <h2 className="text-lg font-semibold text-gray-800">Mission Details</h2>
            </div>
            
            <div className="p-4 space-y-4">
              <div>
                <label htmlFor="mission-name" className="block text-sm font-medium text-gray-700 mb-1">
                  Mission Name
                </label>
                <input
                  type="text"
                  id="mission-name"
                  value={missionName}
                  onChange={(e) => setMissionName(e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
                  placeholder="Enter mission name"
                />
              </div>
              
              <div>
                <label htmlFor="location" className="block text-sm font-medium text-gray-700 mb-1">
                  Location
                </label>
                <input
                  type="text"
                  id="location"
                  value={location}
                  onChange={(e) => setLocation(e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
                  placeholder="Enter location name"
                />
              </div>
              
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <label htmlFor="height" className="block text-sm font-medium text-gray-700 mb-1">
                    Flight Height (m)
                  </label>
                  <input
                    type="number"
                    id="height"
                    value={height}
                    onChange={(e) => setHeight(Number(e.target.value))}
                    min="5"
                    max="50"
                    className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
                  />
                </div>
                
                <div>
                  <label htmlFor="speed" className="block text-sm font-medium text-gray-700 mb-1">
                    Flight Speed (m/s)
                  </label>
                  <input
                    type="number"
                    id="speed"
                    value={speed}
                    onChange={(e) => setSpeed(Number(e.target.value))}
                    min="1"
                    max="10"
                    className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
                  />
                </div>
              </div>
            </div>
          </div>
          
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
            <div className="p-4 border-b border-gray-200 flex items-center">
              <Leaf size={18} className="text-emerald-600 mr-2" />
              <h2 className="text-lg font-semibold text-gray-800">Fruit Type</h2>
            </div>
            
            <div className="p-4">
              <label htmlFor="fruit-type" className="block text-sm font-medium text-gray-700 mb-2">
                Select Fruit Type
              </label>
              <select
                id="fruit-type"
                value={selectedFruitType}
                onChange={(e) => setSelectedFruitType(e.target.value)}
                className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
              >
                <option value="">Select a fruit type</option>
                {fruitTypes.map((fruit) => (
                  <option key={fruit.id} value={fruit.id}>
                    {fruit.name}
                  </option>
                ))}
              </select>
              
              <div className="mt-4 grid grid-cols-3 gap-2">
                {fruitTypes.map((fruit) => (
                  <button
                    key={fruit.id}
                    type="button"
                    onClick={() => setSelectedFruitType(fruit.id)}
                    className={`
                      flex items-center justify-center p-2 rounded-md border
                      ${selectedFruitType === fruit.id ? 'border-emerald-500 bg-emerald-50 text-emerald-700' : 'border-gray-300 hover:bg-gray-50'}
                    `}
                  >
                    <div className={`w-3 h-3 rounded-full ${fruit.color} mr-2`}></div>
                    <span className="text-sm">{fruit.name}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>
          
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
            <div className="p-4 border-b border-gray-200 flex items-center">
              <Cpu size={18} className="text-blue-600 mr-2" />
              <h2 className="text-lg font-semibold text-gray-800">ML Model</h2>
            </div>
            
            <div className="p-4">
              <label htmlFor="model-type" className="block text-sm font-medium text-gray-700 mb-2">
                Select Detection Model
              </label>
              <select
                id="model-type"
                value={selectedModelType}
                onChange={(e) => setSelectedModelType(e.target.value)}
                className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-emerald-500"
              >
                <option value="">Select a model</option>
                {modelTypes.map((model) => (
                  <option key={model.id} value={model.id}>
                    {model.name}
                  </option>
                ))}
              </select>
              
              {selectedModelType && (
                <div className="mt-4 p-3 bg-blue-50 text-blue-800 rounded-md flex items-start">
                  <Info size={18} className="text-blue-500 mr-2 mt-0.5 flex-shrink-0" />
                  <div>
                    <p className="font-medium mb-1">
                      {modelTypes.find(m => m.id === selectedModelType)?.name}
                    </p>
                    <p className="text-sm text-blue-700">
                      {modelTypes.find(m => m.id === selectedModelType)?.description}
                    </p>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>
        
        {/* Map and waypoints */}
        <div className="lg:col-span-2 space-y-6">
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
            <div className="p-4 border-b border-gray-200 flex items-center justify-between">
              <div className="flex items-center">
                <MapIcon size={18} className="text-indigo-600 mr-2" />
                <h2 className="text-lg font-semibold text-gray-800">Flight Path</h2>
              </div>
              
              <button
                type="button"
                className="flex items-center text-sm text-indigo-600 hover:text-indigo-800"
              >
                <Plus size={16} className="mr-1" />
                Add Waypoint
              </button>
            </div>
            
            <div className="h-80 bg-gray-100 relative">
              <MapComponent onAddWaypoint={handleAddWaypoint} waypoints={waypoints} />
            </div>
            
            <div className="p-4 border-t border-gray-200">
              <h3 className="text-sm font-medium text-gray-700 mb-3">Waypoints</h3>
              
              {waypoints.length === 0 ? (
                <div className="text-center py-6 border border-dashed border-gray-300 rounded-md bg-gray-50">
                  <MapPin size={24} className="text-gray-400 mx-auto mb-2" />
                  <p className="text-gray-500">Click on the map to add waypoints</p>
                </div>
              ) : (
                <div className="space-y-2 max-h-60 overflow-y-auto">
                  {waypoints.map((waypoint, index) => (
                    <div 
                      key={`waypoint-${index}`}
                      className="flex items-center justify-between p-3 bg-gray-50 rounded-md border border-gray-200"
                    >
                      <div>
                        <div className="flex items-center">
                          <div className="w-6 h-6 rounded-full bg-indigo-100 text-indigo-800 flex items-center justify-center font-medium text-xs">
                            {index + 1}
                          </div>
                          <span className="ml-2 font-medium text-gray-700">Waypoint {index + 1}</span>
                        </div>
                        <div className="text-xs text-gray-500 mt-1">
                          Lat: {waypoint.lat.toFixed(6)}, Lng: {waypoint.lng.toFixed(6)}
                        </div>
                      </div>
                      
                      <button
                        type="button"
                        onClick={() => handleRemoveWaypoint(index)}
                        className="p-1.5 text-gray-500 hover:text-red-500 rounded-full hover:bg-gray-100"
                      >
                        <Trash2 size={16} />
                      </button>
                    </div>
                  ))}
                </div>
              )}
              
              {waypoints.length > 0 && (
                <div className="flex justify-between items-center mt-4 pt-4 border-t border-gray-200">
                  <div className="text-sm text-gray-700">
                    <span className="font-medium">{waypoints.length}</span> waypoints
                  </div>
                  <button
                    type="button"
                    onClick={() => setWaypoints([])}
                    className="flex items-center text-sm text-red-600 hover:text-red-800"
                  >
                    <Trash2 size={16} className="mr-1" />
                    Clear All
                  </button>
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default MissionPlanner;