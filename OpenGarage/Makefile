SKETCH = ./examples/ogMainArduino/ogMainArduino.ino
LIBS = $(ESP_LIBS)/Wire \
$(ESP_LIBS)/ESP8266WiFi \
$(ESP_LIBS)/ESP8266mDNS \
$(ESP_LIBS)/DNSServer \
$(ESP_LIBS)/ESP8266WebServer \
$(ESP_LIBS)/ESP8266HTTPClient \
$(ESP_LIBS)/Ticker \
~/Arduino/libraries/Blynk \
~/Arduino/libraries/PubSubClient/src \
~/Arduino/libraries/AM2320 \
~/Arduino/libraries/OneWire \
~/Arduino/libraries/DHTsensor \
~/Arduino/libraries/DallasTemperature \
~/Arduino/libraries/ArduinoJson/src \
.

ESP_ROOT = $(HOME)/Arduino/hardware/esp8266com/esp8266
BUILD_ROOT = /tmp/$(MAIN_NAME)

#UPLOAD_SPEED = 921600
UPLOAD_SPEED = 230400
UPLOAD_VERB = -v
UPLOAD_RESET = nodemcu
# Uncomment the line below for revision 1: reset mode is ck
# UPLOAD_RESET = ck

FLASH_DEF = 4M1M
# FLASH_DEF = 2M
FLASH_MODE = dio
FLASH_SPEED = 40  
F_CPU = 80000000L

BOARD = generic

include ./makeEspArduino.mk
