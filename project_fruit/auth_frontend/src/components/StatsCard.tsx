import React, { ReactNode } from 'react';
import { TrendingUp, TrendingDown } from 'lucide-react';

interface StatsCardProps {
  title: string;
  value: string;
  icon: ReactNode;
  trend?: string;
  trendDirection?: 'up' | 'down' | 'neutral';
}

const StatsCard: React.FC<StatsCardProps> = ({ 
  title, 
  value, 
  icon,
  trend,
  trendDirection = 'neutral'
}) => {
  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-4 flex flex-col">
      <div className="flex items-center mb-2">
        <div className="mr-2">{icon}</div>
        <span className="text-sm font-medium text-gray-600">{title}</span>
      </div>
      
      <div className="text-2xl font-bold text-gray-800 mb-1">{value}</div>
      
      {trend && (
        <div className={`flex items-center text-xs
          ${trendDirection === 'up' ? 'text-green-600' : 
            trendDirection === 'down' ? 'text-red-600' : 
            'text-gray-500'}
        `}>
          {trendDirection === 'up' && <TrendingUp size={14} className="mr-1" />}
          {trendDirection === 'down' && <TrendingDown size={14} className="mr-1" />}
          <span>{trend}</span>
        </div>
      )}
    </div>
  );
};

export default StatsCard;