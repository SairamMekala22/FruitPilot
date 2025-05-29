import express from 'express';
import { droneController } from '../controllers/droneController';

const router = express.Router();

// Start the drone
router.post('/start', async (req, res) => {
    try {
        await droneController.startDrone();
        res.json({ success: true, message: 'Drone started successfully' });
    } catch (error: any) {
        res.status(500).json({ success: false, message: error.message });
    }
});

// Stop the drone
router.post('/stop', async (req, res) => {
    try {
        await droneController.stopDrone();
        res.json({ success: true, message: 'Drone stopped successfully' });
    } catch (error: any) {
        res.status(500).json({ success: false, message: error.message });
    }
});

// Get drone status
router.get('/status', (req, res) => {
    try {
        const status = droneController.getDroneStatus();
        res.json({ success: true, data: status });
    } catch (error: any) {
        res.status(500).json({ success: false, message: error.message });
    }
});

export default router; 