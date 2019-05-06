# growbook
First version for used temp only

--------
## Library used
DHT11/DHT22 https://github.com/beegee-tokyo/DHTesp
  
  Datasheets:
  - http://www.micro4you.com/files/sensor/DHT11.pdf
  - http://www.adafruit.com/datasheets/DHT22.pdf
  - http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/RHT03.pdf
  - http://meteobox.tk/files/AM2302.pdf
--------
### DHT11 vs DHT22

#### DHT11
 * 3 to 5V power
 * 2.5mA max current use during conversion (while requesting data)
 * Good for **20-80%** humidity readings with **5% accuracy**
 * Good for 0-50°C temperature readings ±2°C accuracy
 * No more than 1 Hz sampling rate (once every second)
#### DHT22
 * 3 to 5V power
 * 2.5mA max current use during conversion (while requesting data)
 * Good for **0-100%** humidity readings with **2-5% accuracy**
 * Good for -40 to 80°C temperature readings ±0.5°C accuracy
 * No more than 0.5 Hz sampling rate (once every 2 seconds)
 --------