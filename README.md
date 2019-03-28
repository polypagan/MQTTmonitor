#MQTT monitor

Hardware: SBC (supports ESP8266, ESP32, originally for Arduino Uno with CC3000 Wifi, but I never got that to work).

Library support:
  Uses Arduino core for selected processor - WiFi, WiFiclient, DNS, Webserver, OTAupdate,
  
  Also Time, TimeZone, Wire, SPI, Streaming output, ArduinoJSON, PubSubClient.
    
  Plus this uses my proprietary myESPEEPROM, prettyPlotPoint and FSM code (see github repos).

Current implementation plots one selected field from one selected topic.
