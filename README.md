# Beaglebone-black-Adafruit-TTL-serial-camera
Capture image with Adafruit TTL serial camera
It was designed to be used in security systems and does two main things - it outputs NTSC monochrome video and can take snapshots of that video (in color) and transmit them over the TTL serial link.
You can snap pictures at 640x480, 320x240 or 160x120 and they're pre-compressed JPEG images which makes them nice and small and easy to store on an SD card

Connection:

Camera  Beaglebone Black
+5V     P9 7  (SYS_5V)
GND     P9 1  (GND)
RX      P9 24 (uart1_txd)
TX      P9 26 (uart1_rxd)
