# LoRa Rover

The Lora Rover is a standard RC car equipped with an lora transmission module with which large distances can be overcome.
The range comes with one big disadvantage which is the low bandwith (300 kbps), which is also the challange since the controller only has limited information to operate, and barely no visual information such as a video feed.
The system consists of multiple units, the rover itself, a [basestation](https://github.com/cy8berpunk/lora_rover_basestation) tranceiver and the web interface(int the basestation repo).

## LoRa Protocol

LoRa is a low freq. high range protocol, which is designed to be used with IOT devices which don't require high bandwith.

## LoRa Rover control

The rover can be controlled by sending control strings containing coordinates to the rover, the rover the drives to the given location.
The driving behaviour is linear and as such needs to be driven with a very short distance between waypoints.

## LoRa Rover Image transmission

The image is transmitted in a very low resolution since the LMIC lib limits the maximum data rate to [49 bits per msg](https://www.thethingsnetwork.org/forum/t/lmic-fails-to-send-application-payload-larger-than-51-bytes/8923/19). The image is downsampled and grey scaled and then transmitted in chunks of 49 bytes which can take up to 1 minute or more. There are no parity checks implemented since that would dramaticly increase the transmission time and bitflips simply don't matter at such a low resolution.

# ROS packages

- cc_node	<br>
  command & control node
- comp_node <br>
  compass node, handling and publishing incoming i2C compass data
- driving_node <br>
  providing services for basic robot movements
- gps_nav_node <br>
  providing services for gps & compass dependent movements
- gps_node <br>
  handling & proccessing, publishing incoming serial gps data
- lora_node <br>
  providing and interface for communication and houses img transmission
