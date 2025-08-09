document.addEventListener('DOMContentLoaded', () => {
    const form = document.getElementById('device-config-form');
    const saveButton = document.getElementById('save-config-btn');
    const saveButtonText = document.getElementById('save-btn-text');
    const loadingSpinner = document.getElementById('loading-spinner');
    const statusMessage = document.getElementById('status-message');

    // RFID Writing Function
    async function writeRFIDTag(tagData) {
        try {
            // Implement RFID writing logic
            // This will depend on your specific RFID hardware and library
            const response = await fetch('/api/rfid/write', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ 
                    tagUID: tagData.rfidTag,
                    deviceName: tagData.deviceName
                })
            });

            if (!response.ok) {
                throw new Error('RFID Write Failed');
            }

            return await response.json();
        } catch (error) {
            console.error('RFID Write Error:', error);
            throw error;
        }
    }

    // Status Message Function
    function showStatus(message, type = 'info') {
        statusMessage.textContent = message;
        statusMessage.className = `text-${type === 'success' ? 'green' : 'red'}-500`;
    }

    // Form Submission Handler
    form.addEventListener('submit', async (e) => {
        e.preventDefault();
        
        // Collect form data
        const formData = new FormData(e.target);
        const config = {
            rfidTag: formData.get('rfid-tag'),
            deviceName: formData.get('device-name')
        };

        // Disable save button and show loading
        saveButton.disabled = true;
        saveButtonText.textContent = 'Saving...';
        loadingSpinner.classList.remove('hidden');

        try {
            // Write RFID Tag
            await writeRFIDTag(config);

            // Show success message
            showStatus('Configuration saved and RFID tag written successfully!', 'success');

            // Optional: Prompt for another device
            setTimeout(() => {
                if (confirm('Configuration saved! Configure another device?')) {
                    form.reset();
                }
            }, 2000);

        } catch (error) {
            // Handle errors
            showStatus(`Configuration failed: ${error.message}`, 'error');
        } finally {
            // Reset button state
            saveButton.disabled = false;
            saveButtonText.textContent = 'Save Configuration';
            loadingSpinner.classList.add('hidden');
        }
    });

    // NFC/RFID Scanning Support (Example - needs actual implementation)
    if ('NDEFReader' in window) {
        const nfcScanButton = document.createElement('button');
        nfcScanButton.textContent = 'Scan RFID/NFC Tag';
        nfcScanButton.classList.add('w-full', 'bg-green-600', 'text-white', 'py-2', 'rounded-lg', 'mt-4');
        
        nfcScanButton.addEventListener('click', async () => {
            try {
                const ndef = new NDEFReader();
                await ndef.scan();
                
                ndef.addEventListener("reading", event => {
                    const rfidTag = event.message.records[0].data;
                    document.getElementById('rfid-tag').value = rfidTag;
                });
            } catch (error) {
                console.error('NFC Scan Error:', error);
            }
        });

        form.appendChild(nfcScanButton);
    }
});
