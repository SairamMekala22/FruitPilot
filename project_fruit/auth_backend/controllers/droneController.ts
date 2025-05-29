import { spawn } from 'child_process';
import { EventEmitter } from 'events';

class DroneController extends EventEmitter {
    private process: any;
    private isRunning: boolean = false;

    constructor() {
        super();
    }

    startDrone = async () => {
        if (this.isRunning) {
            throw new Error('Drone process is already running');
        }

        this.process = spawn('python', ['Flightcode.py'], {
            cwd: process.cwd()
        });

        this.isRunning = true;

        this.process.stdout.on('data', (data: Buffer) => {
            const message = data.toString();
            this.emit('droneData', { type: 'info', message });
        });

        this.process.stderr.on('data', (data: Buffer) => {
            const message = data.toString();
            this.emit('droneData', { type: 'error', message });
        });

        this.process.on('close', (code: number) => {
            this.isRunning = false;
            this.emit('droneData', { type: 'status', message: `Drone process exited with code ${code}` });
        });
    }

    stopDrone = async () => {
        if (!this.isRunning) {
            throw new Error('Drone is not running');
        }

        this.process.kill();
        this.isRunning = false;
    }

    getDroneStatus = () => {
        return {
            isRunning: this.isRunning
        };
    }
}

export const droneController = new DroneController(); 