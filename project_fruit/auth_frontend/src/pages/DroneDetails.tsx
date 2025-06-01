import React, { useState, useEffect } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import { 
  Battery, 
  Wifi, 
  AlertTriangle, 
  Camera, 
  ArrowUp, 
  Navigation, 
  Thermometer,
  Play,
  Pause,
  Home,
  X,
  RefreshCcw,
  Eye
} from 'lucide-react';
import { formatDistanceToNow } from 'date-fns';

const DroneDetails = () => {
  const { missionId } = useParams();
  const navigate = useNavigate();
  const [missionStatus, setMissionStatus] = useState<'ready' | 'in-progress' | 'paused' | 'completed'>('ready');
  const [batteryLevel, setBatteryLevel] = useState(98);
  const [signalStrength, setSignalStrength] = useState(87);
  const [temperature, setTemperature] = useState(26.5);
  const [altitude, setAltitude] = useState(0);
  const [speed, setSpeed] = useState(0);
  const [detectedFruits, setDetectedFruits] = useState(0);
  const [elapsedTime, setElapsedTime] = useState(0);
  const [startTime, setStartTime] = useState<Date | null>(null);
  
  // Simulate real-time data updates
  useEffect(() => {
    let interval: ReturnType<typeof setInterval>;
    
    if (missionStatus === 'in-progress') {
      interval = setInterval(() => {
        // Update battery (slowly decreasing)
        setBatteryLevel(prev => Math.max(prev - 0.1, 0));
        
        // Fluctuate signal strength
        setSignalStrength(prev => Math.max(Math.min(prev + (Math.random() * 4 - 2), 100), 60));
        
        // Update temperature (slight fluctuations)
        setTemperature(prev => prev + (Math.random() * 0.4 - 0.2));
        
        // Update altitude and speed based on mission phase
        if (altitude < 10) {
          setAltitude(prev => prev + 0.5);
          setSpeed(prev => Math.min(prev + 0.2, 5));
        } else if (elapsedTime > 60 && altitude > 2) {
          setAltitude(prev => Math.max(prev - 0.3, 0));
          setSpeed(prev => Math.max(prev - 0.1, 0));
        }
        
        // Randomly detect fruits
        if (Math.random() > 0.7 && altitude > 5) {
          setDetectedFruits(prev => prev + Math.floor(Math.random() * 3) + 1);
        }
        
        // Update elapsed time
        if (startTime) {
          const now = new Date();
          setElapsedTime(Math.floor((now.getTime() - startTime.getTime()) / 1000));
        }
      }, 1000);
    }
    
    return () => clearInterval(interval);
  }, [missionStatus, altitude, elapsedTime, startTime]);
  
  const startMission = () => {
    setMissionStatus('in-progress');
    setStartTime(new Date());
  };
  
  const pauseMission = () => {
    setMissionStatus('paused');
  };
  
  const resumeMission = () => {
    setMissionStatus('in-progress');
  };
  
  const returnToHome = () => {
    if (window.confirm('Are you sure you want to abort the mission and return to home?')) {
      // Simulate return to home logic
      setMissionStatus('completed');
      navigate('/');
    }
  };
  
  // Format elapsed time as mm:ss
  const formatElapsedTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };
  
  return (
    <div className="space-y-6 animate-fadeIn">
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-800">Drone Control</h1>
          <p className="text-gray-600 mt-1">
            Mission {missionId === 'current' ? 'Preparation' : `#${missionId}`} • {missionStatus === 'ready' ? 'Ready to Start' : (
              missionStatus === 'in-progress' ? 'In Progress' : (
                missionStatus === 'paused' ? 'Paused' : 'Completed'
              )
            )}
          </p>
        </div>
        
        <div className="flex items-center space-x-3 mt-4 sm:mt-0">
          {missionStatus === 'ready' && (
            <button 
              onClick={startMission}
              className="flex items-center gap-2 bg-emerald-600 hover:bg-emerald-700 text-white px-4 py-2 rounded-lg"
            >
              <Play size={18} />
              <span>Start Mission</span>
            </button>
          )}
          
          {missionStatus === 'in-progress' && (
            <button 
              onClick={pauseMission}
              className="flex items-center gap-2 bg-amber-500 hover:bg-amber-600 text-white px-4 py-2 rounded-lg"
            >
              <Pause size={18} />
              <span>Pause</span>
            </button>
          )}
          
          {missionStatus === 'paused' && (
            <button 
              onClick={resumeMission}
              className="flex items-center gap-2 bg-emerald-600 hover:bg-emerald-700 text-white px-4 py-2 rounded-lg"
            >
              <Play size={18} />
              <span>Resume</span>
            </button>
          )}
          
          {missionStatus !== 'ready' && (
            <button
              onClick={returnToHome}
              className="flex items-center gap-2 bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded-lg"
            >
              <Home size={18} />
              <span>Return Home</span>
            </button>
          )}
        </div>
      </div>
      
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Live feed section */}
        <div className="lg:col-span-2 bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
          <div className="p-4 border-b border-gray-200 flex items-center justify-between">
            <div className="flex items-center">
              <Camera size={18} className="text-indigo-600 mr-2" />
              <h2 className="text-lg font-semibold text-gray-800">Live Camera Feed</h2>
            </div>
            
            <div className="flex items-center space-x-2">
              <div className={`flex items-center ${missionStatus === 'in-progress' ? 'text-green-600' : 'text-gray-500'}`}>
                <span className={`inline-block w-2 h-2 rounded-full mr-1.5 ${missionStatus === 'in-progress' ? 'bg-green-600 animate-pulse' : 'bg-gray-500'}`}></span>
                <span className="text-sm">{missionStatus === 'in-progress' ? 'Live' : 'Standby'}</span>
              </div>
              
              <button className="p-1.5 text-gray-500 hover:text-indigo-600 rounded-full hover:bg-gray-100">
                <RefreshCcw size={16} />
              </button>
              
              {/* <button className="p-1.5 text-gray-500 hover:text-indigo-600 rounded-full hover:bg-gray-100">
                <Eye size={16} />
              </button> */}
            </div>
          </div>
          
          <div className="relative">
            <div className="bg-gray-900 w-full h-96 flex items-center justify-center">
              {missionStatus === 'ready' ? (
                <div className="text-center">
                  <Camera size={48} className="text-gray-600 mx-auto mb-3" />
                  <p className="text-gray-400 mb-2">Camera feed will be available when the mission starts</p>
                  <button 
                    onClick={startMission}
                    className="mt-2 bg-emerald-600 hover:bg-emerald-700 text-white px-4 py-2 rounded-lg"
                  >
                    Start Mission
                  </button>
                </div>
              ) : (
                <>
                  {/* Simulated drone camera feed */}
                  <div className="w-full h-full bg-gradient-to-b from-sky-600 to-sky-400 relative">
                    {/* Ground elements would be here in a real implementation */}
                    <div className="absolute bottom-0 left-0 right-0 h-24 bg-gradient-to-t from-green-800 to-transparent"></div>
                    
                    {/* Detection overlays */}
                    {missionStatus === 'in-progress' && detectedFruits > 0 && (
                      <>
                        {Array.from({ length: Math.min(detectedFruits, 15) }).map((_, idx) => (
                          <div 
                            key={idx}
                            className="absolute rounded-md border-2 border-green-400 bg-green-400/20 flex items-center justify-center"
                            style={{
                              width: Math.random() * 40 + 30,
                              height: Math.random() * 40 + 30,
                              left: Math.random() * 80 + 10 + '%',
                              top: Math.random() * 80 + 10 + '%',
                              transform: 'translate(-50%, -50%)'
                            }}
                          >
                            <span className="text-xs text-white font-bold bg-green-600 px-1 rounded">
                              {Math.random() > 0.7 ? 'Ripe' : 'Unripe'}
                            </span>
                          </div>
                        ))}
                      </>
                    )}
                    
                    {/* HUD Overlay */}
                    <div className="absolute inset-0 pointer-events-none">
                      {/* <div className="absolute top-4 left-4 text-white text-shadow">
                        <div className="text-sm">Alt: {altitude.toFixed(1)}m</div>
                        <div className="text-sm">Speed: {speed.toFixed(1)}m/s</div>
                      </div> */}
                      
                      {/* <div className="absolute top-4 right-4 text-white text-shadow">
                        <div className="text-sm">Battery: {batteryLevel.toFixed(0)}%</div>
                        <div className="text-sm">Signal: {signalStrength.toFixed(0)}%</div>
                      </div> */}
                      
                      {/* <div className="absolute bottom-4 left-4 text-white text-shadow">
                        <div className="text-sm">Fruits Detected: {detectedFruits}</div>
                        <div className="text-sm">Elapsed: {formatElapsedTime(elapsedTime)}</div>
                      </div> */}
                      
                      {/* Center reticle */}
                      <div className="absolute top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2">
                        <div className="w-20 h-20 border-2 border-white/40 rounded-full flex items-center justify-center">
                          <div className="w-1 h-1 bg-white/80 rounded-full"></div>
                        </div>
                      </div>
                      
                      {/* Compass */}
                      {/* <div className="absolute top-4 left-1/2 transform -translate-x-1/2 text-white bg-black/30 rounded px-2 py-1">
                        { <div className="flex items-center text-sm">
                          <Navigation size={12} className="mr-1" />
                          <span>Heading: 315° NW</span>
                        </div> }
                      </div> */}
                    </div>
                  </div>
                </>
              )}
            </div>
            
            {/* Mission status overlay */}
            {/* {missionStatus !== 'ready' && (
              <div className="absolute top-0 right-0 m-4 bg-black/60 text-white px-3 py-1.5 rounded-md flex items-center space-x-2">
                {missionStatus === 'in-progress' && (
                  <>
                    <span className="w-2 h-2 bg-green-500 rounded-full animate-pulse"></span>
                    <span>LIVE</span>
                  </>
                )}
                {missionStatus === 'paused' && (
                  <>
                    <span className="w-2 h-2 bg-amber-500 rounded-full"></span>
                    <span>PAUSED</span>
                  </>
                )}
                {missionStatus === 'completed' && (
                  <>
                    <span className="w-2 h-2 bg-blue-500 rounded-full"></span>
                    <span>COMPLETED</span>
                  </>
                )}
              </div>
            )} */}
          </div>
        </div>
        
        {/* Drone stats and controls */}
        <div className="lg:col-span-1 space-y-6">
          <div className="bg-white rounded-lg shadow-sm border border-gray-200 overflow-hidden">
            <div className="p-4 border-b border-gray-200">
              <h2 className="text-lg font-semibold text-gray-800">Drone Status</h2>
            </div>
            
            <div className="p-4 space-y-4">
              <div className="flex justify-between items-center">
                <div className="flex items-center text-gray-700">
                  <Battery size={18} className="mr-2" />
                  <span>Battery</span>
                </div>
                <div className="flex items-center">
                  <div className="w-24 h-3 bg-gray-200 rounded-full overflow-hidden mr-2">
                    <div 
                      className={`h-full rounded-full ${
                        batteryLevel > 50 ? 'bg-green-500' : 
                        batteryLevel > 20 ? 'bg-amber-500' : 
                        'bg-red-500'
                      }`}
                      style={{ width: `${batteryLevel}%` }}
                    ></div>
                  </div>
                  <span className="text-sm font-medium">{batteryLevel.toFixed(0)}%</span>
                </div>
              </div>
              
              <div className="flex justify-between items-center">
                <div className="flex items-center text-gray-700">
                  <Wifi size={18} className="mr-2" />
                  <span>Signal</span>
                </div>
                <div className="flex items-center">
                  <div className="w-24 h-3 bg-gray-200 rounded-full overflow-hidden mr-2">
                    <div 
                      className={`h-full rounded-full ${
                        signalStrength > 70 ? 'bg-green-500' : 
                        signalStrength > 30 ? 'bg-amber-500' : 
                        'bg-red-500'
                      }`}
                      style={{ width: `${signalStrength}%` }}
                    ></div>
                  </div>
                  <span className="text-sm font-medium">{signalStrength.toFixed(0)}%</span>
                </div>
              </div>
              
              <div className="flex justify-between items-center">
                <div className="flex items-center text-gray-700">
                  <ArrowUp size={18} className="mr-2" />
                  <span>Altitude</span>
                </div>
                <span className="text-sm font-medium">{altitude.toFixed(1)} meters</span>
              </div>
              
              <div className="flex justify-between items-center">
                <div className="flex items-center text-gray-700">
                  <Navigation size={18} className="mr-2" />
                  <span>Speed</span>
                </div>
                <span className="text-sm font-medium">{speed.toFixed(1)} m/s</span>
              </div>
              
              <div className="flex justify-between items-center">
                <div className="flex items-center text-gray-700">
                  <Thermometer size={18} className="mr-2" />
                  <span>Temperature</span>
                </div>
                <span className="text-sm font-medium">{temperature.toFixed(1)}°C</span>
              </div>
              
              <div className="border-t border-gray-200 my-2 pt-2">
                <div className="flex justify-between items-center mb-2">
                  <div className="flex items-center text-gray-700">
                    <span className="font-medium">Mission Stats</span>
                  </div>
                  <span className="text-xs text-gray-500">
                    {startTime ? formatDistanceToNow(startTime, { addSuffix: true }) : 'Not started'}
                  </span>
                </div>
                
                <div className="grid grid-cols-2 gap-3">
                  <div className="bg-indigo-50 p-3 rounded-lg">
                    <div className="text-indigo-800 text-xs mb-1">Fruits Detected</div>
                    <div className="text-2xl font-bold text-indigo-700">{detectedFruits}</div>
                  </div>
                  
                  <div className="bg-emerald-50 p-3 rounded-lg">
                    <div className="text-emerald-800 text-xs mb-1">Mission Time</div>
                    <div className="text-2xl font-bold text-emerald-700">{formatElapsedTime(elapsedTime)}</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
          
          {/* Alerts */}
          {batteryLevel < 20 && (
            <div className="bg-red-50 border border-red-200 rounded-lg p-4 flex items-start">
              <AlertTriangle size={20} className="text-red-500 mr-3 flex-shrink-0 mt-0.5" />
              <div>
                <h3 className="font-semibold text-red-800">Low Battery Warning</h3>
                <p className="text-sm text-red-700 mt-1">
                  Battery level below 20%. Drone will automatically return to home soon.
                </p>
              </div>
              <button className="ml-auto p-1 text-red-500 hover:text-red-700">
                <X size={18} />
              </button>
            </div>
          )}
          
          {signalStrength < 40 && (
            <div className="bg-amber-50 border border-amber-200 rounded-lg p-4 flex items-start">
              <AlertTriangle size={20} className="text-amber-500 mr-3 flex-shrink-0 mt-0.5" />
              <div>
                <h3 className="font-semibold text-amber-800">Weak Signal</h3>
                <p className="text-sm text-amber-700 mt-1">
                  Signal strength is low. Video quality may be reduced.
                </p>
              </div>
              <button className="ml-auto p-1 text-amber-500 hover:text-amber-700">
                <X size={18} />
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default DroneDetails;