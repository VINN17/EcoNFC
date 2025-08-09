const express = require('express');
const app = express();
const bodyParser = require('body-parser');

app.use(bodyParser.json());

// Endpoint untuk menyimpan konfigurasi perangkat
app.post('/save-config', (req, res) => {
    const { nfcUID, ssid, password, serverUrl } = req.body;
    
    console.log('Menerima Konfigurasi:');
    console.log(`NFC UID: ${nfcUID}`);
    console.log(`SSID: ${ssid}`);
    console.log(`Password: ${password}`);
    console.log(`Server URL: ${serverUrl}`);

    // Logika untuk menyimpan pengaturan ini ke database atau langsung kirim ke perangkat

    res.status(200).json({ message: 'Konfigurasi berhasil disimpan' });
});

// Jalankan server
app.listen(3000, () => {
    console.log('Server berjalan di http://localhost:3000');
});
