This folder contains compiled OpenGarage firmware files.

To use OTA (over-the-air) firmware update:

* Download the specific firmware file (e.g. og_x.x.x.bin)

* Before uploading, close the Blynk and OpenGarage app if you use either of them, to avoid them interfering with the upload process.
  
* If your OpenGarage is already connected to your WiFi network, go to its homepage, click 'Update' at the bottom of the homepage. Select the firmware og_x.x.x.bin file, type in device key, and 'Submit'. Wait till it finishes. If the upload fails for any reason, you can try again. If the device hangs, unplug and plug back power and try again.

* If your OpenGarage is in Access Point (AP) mode (such as immediately after a factory reset), you can use your laptop or computer to connect to its AP SSID (which looks like OG_ followed by 6 letters), then once connected, open a browser and type in http://192.168.4.1/update, then select the firmware .bin file, type in device key, and 'Submit'. Wait till it finishes.
  
* If the firmware upload is corrupted for any reason, causing the device to fail to boot, you will need to re-upload the firmware using a USB cable and USB-serial adapter (such as the popular CH340-based serial adapter).
