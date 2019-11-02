# LoRa Rover 

The Lora Rover is a standard RC car equipped with an lora transmission module with which large distances can be overcome.
The range comes with one big disadvantage which is the low bandwith (300 kbps), which is also the challange since the controller only has limited information to operate, and barely no visual information such as a video feed.
The system consists of multiple units, the rover itself, a [basestation](https://github.com/cy8berpunk/lora_rover_basestation) tranceiver and the web interface(int the basestation repo).

## LoRa Protocol

LoRa is a low freq. high range protocol, which is designed to be used with IOT devices which don't require high bandwith.

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
  providing and interface for communication
