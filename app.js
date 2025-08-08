const scanButton = document.getElementById('scanButton');
const log = document.getElementById('log');

scanButton.addEventListener('click', async () => {
  log.textContent = 'Memindai...\n';

  if ('NDEFReader' in window) {
    try {
      const ndef = new NDEFReader();
      await ndef.scan();
      log.textContent += 'Pemindaian berhasil dimulai.\n';

      ndef.addEventListener('readingerror', () => {
        log.textContent += 'Ups! Tidak dapat membaca data dari tag NFC. Coba yang lain?\n';
      });

      ndef.addEventListener('reading', ({ message, serialNumber }) => {
        log.textContent += `> Nomor Seri: ${serialNumber}\n`;
        log.textContent += `> Rekaman (${message.records.length}):\n`;

        for (const record of message.records) {
          log.textContent += `  > Jenis Rekaman: ${record.recordType}\n`;
          log.textContent += `  > Jenis MIME:   ${record.mediaType}\n`;
          log.textContent += `  > ID Rekaman:   ${record.id}\n`;

          switch (record.recordType) {
            case "text":
              const textDecoder = new TextDecoder(record.encoding);
              log.textContent += `  > Data:        ${textDecoder.decode(record.data)}\n`;
              break;
            case "url":
              const urlDecoder = new TextDecoder();
              log.textContent += `  > URL:         ${urlDecoder.decode(record.data)}\n`;
              break;
            default:
              // Untuk jenis rekaman lain, Anda mungkin memerlukan logika khusus untuk mengurai data
              log.textContent += "  > Data:        (tidak dapat menampilkan jenis rekaman ini)\n";
          }
        }
      });
    } catch (error) {
      log.textContent += `Ups! ${error}\n`;
    }
  } else {
    log.textContent = 'Web NFC tidak didukung di browser ini.';
  }
});