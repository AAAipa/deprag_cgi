# deprag_cgi_interface
This repo contains a ros package, which handle the cgi interface for sequence Controller AST5, AST6, AST10, AST11, AST40 and for Base Station ComCenter 10


##  Used ROS Version
ROS Melodic 

## Requiered Packages
Python3 package requests 2.18.4

## Implemented function Blocks

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

