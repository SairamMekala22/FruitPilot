import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  Plus, 
  Activity, 
  Clock, 
  Map as MapIcon, 
  Leaf,
  ArrowRight, 
  Filter, 
  Search
} from 'lucide-react';
import MissionCard from '../components/MissionCard';
import StatsCard from '../components/StatsCard';
import { Mission } from '../types';

// Sample mission data
const sampleMissions: Mission[] = [
  {
    id: 'mission-001',
    name: 'Apple Orchard Scan',
    date: '2025-05-15',
    location: 'North Field',
    fruitType: 'Apple',
    modelType: 'FruitDetectV2',
    status: 'completed',
    detectedFruits: 387,
    duration: '45 min',
    waypoints: [
      { lat: 37.7749, lng: -122.4194 },
      { lat: 37.7750, lng: -122.4180 },
      { lat: 37.7735, lng: -122.4173 },
    ],
    coverageArea: '4.5 acres'
  },
  {
    id: 'mission-002',
    name: 'Orange Grove Survey',
    date: '2025-05-12',
    location: 'East Grove',
    fruitType: 'Orange',
    modelType: 'CitrusNet',
    status: 'completed',
    detectedFruits: 542,
    duration: '52 min',
    waypoints: [
      { lat: 37.7749, lng: -122.4194 },
      { lat: 37.7750, lng: -122.4180 },
      { lat: 37.7735, lng: -122.4173 },
    ],
    coverageArea: '5.2 acres'
  },
  {
    id: 'mission-003',
    name: 'Berry Field Inspection',
    date: '2025-05-08',
    location: 'West Field',
    fruitType: 'Strawberry',
    modelType: 'BerryVision',
    status: 'failed',
    detectedFruits: 203,
    duration: '18 min',
    waypoints: [
      { lat: 37.7749, lng: -122.4194 },
      { lat: 37.7750, lng: -122.4180 },
    ],
    coverageArea: '2.1 acres'
  },
  {
    id: 'mission-004',
    name: 'Vineyard Mapping',
    date: '2025-05-05',
    location: 'South Vineyard',
    fruitType: 'Grape',
    modelType: 'GrapeClusterNet',
    status: 'completed',
    detectedFruits: 1240,
    duration: '67 min',
    waypoints: [
      { lat: 37.7749, lng: -122.4194 },
      { lat: 37.7750, lng: -122.4180 },
      { lat: 37.7735, lng: -122.4173 },
      { lat: 37.7730, lng: -122.4195 },
    ],
    coverageArea: '7.8 acres'
  },
];

const Dashboard = () => {
  const navigate = useNavigate();
  const [searchQuery, setSearchQuery] = useState('');
  const [filteredMissions, setFilteredMissions] = useState<Mission[]>(sampleMissions);
  
  // Calculate statistics
  const totalMissions = sampleMissions.length;
  const totalFruitsDetected = sampleMissions.reduce((sum, mission) => sum + mission.detectedFruits, 0);
  const totalCoverage = sampleMissions.reduce((sum, mission) => sum + parseFloat(mission.coverageArea.split(' ')[0]), 0);
  const successRate = (sampleMissions.filter(m => m.status === 'completed').length / totalMissions) * 100;
  
  const handleSearch = (e: React.ChangeEvent<HTMLInputElement>) => {
    const query = e.target.value.toLowerCase();
    setSearchQuery(query);
    
    if (query.trim() === '') {
      setFilteredMissions(sampleMissions);
    } else {
      const filtered = sampleMissions.filter(mission => 
        mission.name.toLowerCase().includes(query) ||
        mission.location.toLowerCase().includes(query) ||
        mission.fruitType.toLowerCase().includes(query)
      );
      setFilteredMissions(filtered);
    }
  };
  
  const goToMissionPlanner = () => {
    navigate('/mission-planner');
  };

  return (
    <div className="space-y-6 animate-fadeIn">
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-800">Mission Dashboard</h1>
          <p className="text-gray-600 mt-1">Manage and monitor your fruit detection missions</p>
        </div>
        <button 
          onClick={goToMissionPlanner}
          className="mt-4 sm:mt-0 flex items-center gap-2 bg-emerald-600 hover:bg-emerald-700 text-white px-4 py-2 rounded-lg transition-colors shadow-sm"
        >
          <Plus size={18} />
          <span>New Mission</span>
        </button>
      </div>
      
      {/* Stats Overview */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
        <StatsCard 
          title="Total Missions" 
          value={totalMissions.toString()} 
          icon={<MapIcon size={20} className="text-blue-500" />} 
          trend="+2 this week"
          trendDirection="up"
        />
        <StatsCard 
          title="Fruits Detected" 
          value={totalFruitsDetected.toLocaleString()} 
          icon={<Leaf size={20} className="text-green-500" />} 
          trend="+450 this week"
          trendDirection="up"
        />
        <StatsCard 
          title="Area Covered" 
          value={`${totalCoverage.toFixed(1)} acres`} 
          icon={<Activity size={20} className="text-indigo-500" />} 
          trend="+5.3 acres"
          trendDirection="up"
        />
        <StatsCard 
          title="Success Rate" 
          value={`${successRate.toFixed(0)}%`} 
          icon={<Clock size={20} className="text-amber-500" />} 
          trend="+5% improvement"
          trendDirection="up"
        />
      </div>
      
      {/* Mission List */}
      <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
        <div className="p-4 border-b border-gray-200 flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
          <h2 className="text-lg font-semibold text-gray-800">Recent Missions</h2>
          
          {/* Search and filters */}
          <div className="flex flex-col sm:flex-row gap-3">
            <div className="relative">
              <input
                type="text"
                placeholder="Search missions..."
                className="pl-10 pr-4 py-2 border border-gray-300 rounded-lg w-full focus:outline-none focus:ring-2 focus:ring-emerald-500 focus:border-transparent"
                value={searchQuery}
                onChange={handleSearch}
              />
              <Search size={18} className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" />
            </div>
            
            <button className="flex items-center gap-2 border border-gray-300 rounded-lg px-3 py-2 text-gray-600 hover:bg-gray-50">
              <Filter size={18} />
              <span>Filter</span>
            </button>
          </div>
        </div>
        
        <div className="divide-y divide-gray-200">
          {filteredMissions.length > 0 ? (
            filteredMissions.map(mission => (
              <MissionCard key={mission.id} mission={mission} />
            ))
          ) : (
            <div className="p-8 text-center text-gray-500">
              No missions found matching your search criteria.
            </div>
          )}
        </div>
        
        {/* View all link */}
        <div className="p-4 border-t border-gray-200 bg-gray-50">
          <button className="w-full text-center text-emerald-600 hover:text-emerald-700 font-medium flex items-center justify-center gap-1">
            <span>View All Missions</span>
            <ArrowRight size={16} />
          </button>
        </div>
      </div>
    </div>
  );
};

export default Dashboard;