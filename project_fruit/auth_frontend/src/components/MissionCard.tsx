import React from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  ChevronRight, 
  MapPin, 
  Calendar, 
  Clock, 
  Leaf,
  Cpu,
  CheckCircle,
  XCircle,
  AlertCircle
} from 'lucide-react';
import { Mission } from '../types';

interface MissionCardProps {
  mission: Mission;
}

const MissionCard: React.FC<MissionCardProps> = ({ mission }) => {
  const navigate = useNavigate();
  
  const handleViewMission = () => {
    navigate(`/drone-details/${mission.id}`);
  };
  
  const getStatusBadge = () => {
    switch (mission.status) {
      case 'completed':
        return (
          <div className="flex items-center text-green-600 bg-green-50 px-2 py-1 rounded-md">
            <CheckCircle size={14} className="mr-1" />
            <span className="text-xs font-medium">Completed</span>
          </div>
        );
      case 'failed':
        return (
          <div className="flex items-center text-red-600 bg-red-50 px-2 py-1 rounded-md">
            <XCircle size={14} className="mr-1" />
            <span className="text-xs font-medium">Failed</span>
          </div>
        );
      case 'in-progress':
        return (
          <div className="flex items-center text-blue-600 bg-blue-50 px-2 py-1 rounded-md">
            <div className="w-2 h-2 bg-blue-600 rounded-full animate-pulse mr-1.5"></div>
            <span className="text-xs font-medium">In Progress</span>
          </div>
        );
      default:
        return (
          <div className="flex items-center text-amber-600 bg-amber-50 px-2 py-1 rounded-md">
            <AlertCircle size={14} className="mr-1" />
            <span className="text-xs font-medium">Pending</span>
          </div>
        );
    }
  };
  
  return (
    <div className="p-4 hover:bg-gray-50 transition-colors">
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div className="flex-1">
          <div className="flex flex-col sm:flex-row sm:items-center gap-3">
            <h3 className="font-semibold text-gray-800">{mission.name}</h3>
            {getStatusBadge()}
          </div>
          
          <div className="mt-2 grid grid-cols-1 sm:grid-cols-2 gap-y-2 gap-x-4">
            <div className="flex items-center text-gray-600">
              <MapPin size={16} className="text-gray-400 mr-2" />
              <span className="text-sm">{mission.location}</span>
            </div>
            <div className="flex items-center text-gray-600">
              <Calendar size={16} className="text-gray-400 mr-2" />
              <span className="text-sm">{mission.date}</span>
            </div>
            <div className="flex items-center text-gray-600">
              <Leaf size={16} className="text-gray-400 mr-2" />
              <span className="text-sm">{mission.fruitType}</span>
            </div>
            <div className="flex items-center text-gray-600">
              <Cpu size={16} className="text-gray-400 mr-2" />
              <span className="text-sm">{mission.modelType}</span>
            </div>
          </div>
        </div>
        
        <div className="flex flex-col sm:items-end gap-3">
          <div className="flex flex-col sm:items-end">
            <div className="text-2xl font-semibold text-gray-800">{mission.detectedFruits}</div>
            <div className="text-sm text-gray-500">fruits detected</div>
          </div>
          
          <div className="flex items-center gap-4">
            <div className="flex items-center text-gray-600 text-sm">
              <Clock size={14} className="text-gray-400 mr-1.5" />
              <span>{mission.duration}</span>
            </div>
            
            <button
              onClick={handleViewMission}
              className="flex items-center gap-1 text-emerald-600 hover:text-emerald-700 font-medium text-sm"
            >
              <span>View</span>
              <ChevronRight size={16} />
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default MissionCard;