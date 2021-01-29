# Systemprogrammierung
Projekt zum Modul Systemprogrammierung. 
Das Projekt befasst sich mit dem Erstellen einiger Kernelmodule für den Raspberry Pi, um eine "Wetterstation" mit Anzeige zu realisieren. Dabei geht es weniger um eine "sinnvolle" Realisierung einer Wetterstation, als vielmehr um einen Einblick in systemnahe Programmierung im Linux Kernelspace. 

Konkret umfasst das Projekt folgende Teile:
- Kernelmodul als Gerätetreiber des BME280 Temperatur/Druck Sensors
- Kernelmodul als Gerätetreiber eines MCP23S17 PortExpanders für ein vierstelliges 7-Segment-Display
- Kernelmodul als Gerätetreiber für einen über UART angeschlossenen Microcontroller (ATTINY 2313A), welcher als "Treiber" für einen WS2812 LED-Streifen fungiert
- Userspace Programm, welches alle Gerätetreiber zu einer "Wetterstation" verbindet

Ein Schaltplan, sowie ein Bild vom Versuchsaufbau ist im Dokument ![aufbau.pdf](aufbau.pdf) zu finden.

## Demo Video
[![Video zur Demonstration](https://img.youtube.com/vi/Bq1tRMwmxJQ/0.jpg)](https://www.youtube.com/watch?v=Bq1tRMwmxJQ)

Ressourcen, Quellen und Startpunkte:
- Dokumentation zu Linux-Kernel-API für SPI: https://www.kernel.org/doc/html/v4.15/driver-api/spi.html
- Dokumentation zu Linux-Kernel-API für I2C: https://www.kernel.org/doc/html/v4.15/driver-api/i2c.html
- Zugriff auf UART aus dem Kernelspace: https://github.com/eBUS/ttyebus
