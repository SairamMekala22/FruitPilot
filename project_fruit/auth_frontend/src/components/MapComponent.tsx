import React, { useEffect, useRef, useState } from 'react';
import { Waypoint } from '../types';

interface MapComponentProps {
  onAddWaypoint: (waypoint: Waypoint) => void;
  waypoints: Waypoint[];
}

const MapComponent: React.FC<MapComponentProps> = ({ onAddWaypoint, waypoints }) => {
  const mapRef = useRef<HTMLDivElement>(null);
  
  // In a real application, you would use a proper map library like Leaflet or Google Maps
  // This is a simplified mock version for demonstration purposes
  
  const handleMapClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (!mapRef.current) return;
    
    const rect = mapRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    // Convert pixel coordinates to simulated lat/lng
    // This is just a mock calculation
    const lat = 37.7749 + (y / rect.height) * 0.02;
    const lng = -122.4194 + (x / rect.width) * 0.02;
    
    onAddWaypoint({ lat, lng });
  };
  
  return (
    <div 
      ref={mapRef}
      className="w-full h-full bg-blue-100 relative cursor-crosshair"
      onClick={handleMapClick}
      style={{
        backgroundImage: "url('https://images.pexels.com/photos/440731/pexels-photo-440731.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=1')",
        backgroundSize: 'cover',
        backgroundPosition: 'center',
      }}
    >
      {/* Map UI elements */}
      <div className="absolute top-2 right-2 bg-white shadow-md rounded-md p-2 text-xs">
        Click on the map to add waypoints
      </div>
      
      {/* Render waypoints */}
      {waypoints.map((waypoint, index) => {
        // Convert lat/lng to pixel coordinates (mock calculation)
        const x = ((waypoint.lng + 122.4194) / 0.02) * (mapRef.current?.offsetWidth || 0);
        const y = ((waypoint.lat - 37.7749) / 0.02) * (mapRef.current?.offsetHeight || 0);
        
        return (
          <div 
            key={`waypoint-marker-${index}`}
            className="absolute transform -translate-x-1/2 -translate-y-1/2 z-10"
            style={{ left: `${x}px`, top: `${y}px` }}
          >
            <div className="flex flex-col items-center">
              <div className="w-6 h-6 rounded-full bg-indigo-600 text-white flex items-center justify-center text-xs font-bold border-2 border-white shadow-md">
                {index + 1}
              </div>
              {index > 0 && index < waypoints.length && (
                <div className="mt-1 px-1.5 py-0.5 bg-white/80 rounded text-xs font-medium shadow-sm">
                  WP {index + 1}
                </div>
              )}
            </div>
          </div>
        );
      })}
      
      {/* Draw lines between waypoints */}
      {waypoints.length >= 2 && (
        <svg className="absolute inset-0 pointer-events-none">
          <g>
            {waypoints.map((waypoint, index) => {
              if (index === 0) return null;
              
              const prevWaypoint = waypoints[index - 1];
              
              // Convert lat/lng to pixel coordinates
              const x1 = ((prevWaypoint.lng + 122.4194) / 0.02) * (mapRef.current?.offsetWidth || 0);
              const y1 = ((prevWaypoint.lat - 37.7749) / 0.02) * (mapRef.current?.offsetHeight || 0);
              const x2 = ((waypoint.lng + 122.4194) / 0.02) * (mapRef.current?.offsetWidth || 0);
              const y2 = ((waypoint.lat - 37.7749) / 0.02) * (mapRef.current?.offsetHeight || 0);
              
              return (
                <line 
                  key={`line-${index}`}
                  x1={x1}
                  y1={y1}
                  x2={x2}
                  y2={y2}
                  stroke="#4f46e5"
                  strokeWidth="2"
                  strokeDasharray="4 2"
                />
              );
            })}
          </g>
        </svg>
      )}
    </div>
  );
};

export default MapComponent;