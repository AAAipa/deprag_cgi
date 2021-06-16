# deprag_cgi_interface
This repo contains a ros package, which handle the cgi interface for sequence Controller AST5, AST6, AST10, AST11, AST40 and for Base Station ComCenter 10.


##  Used ROS Version
ROS Melodic 

## Requiered Packages
Python3 package requests 2.18.4

## Implemented Function Blocks

| Number        | Name           | Required parameters  |
| ------------- |-------------|-------------|
|   1   |Tightening to Torque without Angle supervision |Maximum Screwdriving Time, Shut-off Torque, Minimum Torque, Maximum Torque, Screwdriver Speed, rpm, Torque Hold Time |
|   6   |Loosen to Torque without Angle supervision     | |
|   6   |Loosen to Torque with Angle supervision      | |
|   4   |Tightening to External Signal      | |
|   5   |Loosen to External Signal     | |
|   24  |Display Values      |- |
|   25  |Save Values      |- |
|   26  |Statistic      |-|

## Building from Source
$ git clone <br/>
$ catkin build <br/>

## Usage
### setup

The ip address of screwdriver controller is configurable at
deprag_cgi_interface/src/deprag_cgi_node.py 

### Programm Transfer to Sequence Controller
Example 1:<br/>
$ rosservice call /create_pgm<br/>
$ rosservice call /loosen_angle_super<br/>
$ rosservice call /publish<br/>


## Error Coding 
• 0: No error
• 4: The program data could not be written
• 9: Invalid parameter
• 10: The internal conversion program failed
• 11: The program contains no steps
• 15: The EC screwdriver is invalid or not connected (only for ComCenter)
## Used ROS Version
ROS Melodic 

## References
Implemented according to the manual Operating manual: External Program Creation, 013777XX


## Acknowledgement
Sponsored by the Ministry of the Environment Baden-Württemberg, in the context of the Strategic Dialogue Automotive Industry, and supervised by the Project Management Agency Karlsruhe (PTKA). Funding number: L7520101

https://www.ipa.fraunhofer.de/de/referenzprojekte/DeMoBat.html
