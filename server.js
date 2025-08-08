// server.js
const express = require('express');
const cors = require('cors');
const app = express();

app.use(cors());
app.use(express.json());

// Serve static files
app.use(express.static('public'));

// Device API endpoints
app.post('/api/device/:deviceId/connect', (req, res) => {
    const { deviceId } = req.params;
    
    // Check if device exists and is reachable
    // Implement device discovery logic
    
    res.json({ 
        success: true, 
        deviceId: deviceId,
        status: 'connected' 
    });
});

app.get('/api/device/:deviceId/config', (req, res) => {
    const { deviceId } = req.params;
    
    // Get config from database
    const config = {
        deviceName: `Device ${deviceId}`,
        wifiSsid: '',
        serverIp: '192.168.1.100',
        // ... other config
    };
    
    res.json(config);
});

app.post('/api/device/:deviceId/config', (req, res) => {
    const { deviceId } = req.params;
    const config = req.body;
    
    // Save to database
    // Send config to ESP32 device
    
    res.json({ 
        success: true, 
        message: 'Configuration saved' 
    });
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
    console.log(`Server running on port ${PORT}`);
});