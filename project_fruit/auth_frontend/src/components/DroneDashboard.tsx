import React, { useEffect, useState } from 'react';
import { io, Socket } from 'socket.io-client';
import axios from 'axios';

const BACKEND_URL = 'http://localhost:5000';

interface DroneData {
    type: 'info' | 'error' | 'status';
    message: string;
}

export const DroneDashboard: React.FC = () => {
    const [socket, setSocket] = useState<Socket | null>(null);
    const [droneStatus, setDroneStatus] = useState<{ isRunning: boolean }>({ isRunning: false });
    const [logs, setLogs] = useState<DroneData[]>([]);

    useEffect(() => {
        const newSocket = io(BACKEND_URL);
        setSocket(newSocket);

        newSocket.on('droneData', (data: DroneData) => {
            setLogs(prev => [...prev, data]);
        });

        // Fetch initial drone status
        fetchDroneStatus();

        return () => {
            newSocket.close();
        };
    }, []);

    const fetchDroneStatus = async () => {
        try {
            const response = await axios.get(`${BACKEND_URL}/api/drone/status`);
            setDroneStatus(response.data.data);
        } catch (error) {
            console.error('Error fetching drone status:', error);
        }
    };

    const startDrone = async () => {
        try {
            await axios.post(`${BACKEND_URL}/api/drone/start`);
            fetchDroneStatus();
        } catch (error) {
            console.error('Error starting drone:', error);
        }
    };

    const stopDrone = async () => {
        try {
            await axios.post(`${BACKEND_URL}/api/drone/stop`);
            fetchDroneStatus();
        } catch (error) {
            console.error('Error stopping drone:', error);
        }
    };

    return (
        <div className="p-6">
            <div className="mb-6">
                <h1 className="text-2xl font-bold mb-4">Drone Control Dashboard</h1>
                <div className="flex gap-4 mb-4">
                    <button
                        onClick={startDrone}
                        disabled={droneStatus.isRunning}
                        className={`px-4 py-2 rounded ${
                            droneStatus.isRunning
                                ? 'bg-gray-400'
                                : 'bg-green-500 hover:bg-green-600'
                        } text-white`}
                    >
                        Start Drone
                    </button>
                    <button
                        onClick={stopDrone}
                        disabled={!droneStatus.isRunning}
                        className={`px-4 py-2 rounded ${
                            !droneStatus.isRunning
                                ? 'bg-gray-400'
                                : 'bg-red-500 hover:bg-red-600'
                        } text-white`}
                    >
                        Stop Drone
                    </button>
                </div>
                <div className="mb-4">
                    <span className="font-semibold">Status: </span>
                    <span className={droneStatus.isRunning ? 'text-green-500' : 'text-red-500'}>
                        {droneStatus.isRunning ? 'Running' : 'Stopped'}
                    </span>
                </div>
            </div>

            <div>
                <h2 className="text-xl font-bold mb-2">Drone Logs</h2>
                <div className="bg-gray-100 p-4 rounded h-96 overflow-y-auto">
                    {logs.map((log, index) => (
                        <div
                            key={index}
                            className={`mb-2 ${
                                log.type === 'error'
                                    ? 'text-red-600'
                                    : log.type === 'info'
                                    ? 'text-blue-600'
                                    : 'text-gray-600'
                            }`}
                        >
                            {log.message}
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
};

export default DroneDashboard; 