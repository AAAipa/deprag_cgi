# deprag_cgi
This repo contains a ros package, which handle the cgi interface for sequence Controller AST5, AST6, AST10, AST11, AST40 and for Base Station ComCenter 10.


##  Used ROS Version
ROS Melodic 

## Requiered Packages
Python3 package requests 2.18.4

## Implemented Function Blocks

| Number        | Name           | Required parameters  |
| ------------- |-------------|-------------|
|   1   |Tightening to Torque without Angle supervision |Maximum Screwdriving Time, Shut-off Torque, Minimum Torque, Maximum Torque, Screwdriver Speed, rpm, Torque Hold Time |
|   6   |Loosen to Torque without Angle supervision     |Maximum Screwdriving Time, Shut-off Torque, Minimum Torque, Maximum Torque, Screwdriver Speed, Torque Hold Time|
|   6   |Loosen to Torque with Angle supervision      |Maximum Screwdriving Time, Shut-off Torque, Minimum Torque, Maximum Torque, Screwdriver Speed, Torque Hold Time, Threshold Torque, Minimum Angle, Maximum Angle|
|   8   |Loosening to Angle with Torque supervision     |Maximum Screwdriving Time, Shut-off Angle, Minimum Angle, Maximum Angle, Minimum Torque, Maximum Torque, Screwdriver Speed, Type of Final Values|
|   4   |Tightening to External Signal      |Maximum Screwdriving Time, Minimum Angle, Maximum Angle, Minimum Torque, Maximum Torque, Screwdriver Speed, Type of Final Values|
|   5   |Loosen to External Signal     |Maximum Screwdriving Time, Minimum Angle, Maximum Angle, Minimum Torque, Maximum Torque, Screwdriver Speed, Type of Final Values|
|   24  |Display Values      |- |
|   25  |Save Values      |- |
|   26  |Statistic      |-|

## Building from Source
$ git clone <br/>
$ catkin build <br/>

## Usage
### setup

The ip address of screwdriver controller is configurable at
deprag_cgi_interface/src/deprag_cgi_node.py. Further parameters can
also be set there.

### Programm Transfer to Sequence Controller

Firstly open a terminal windows and build your workspace:
```
$ catkin_make
```
Then change your current directory to the deprag_cgi source and register the python script:
```
$ roscd deprag_cgi/src

$ chmod +x deprag_cgi_node.py
```
Now run the deprag cgi script:
```
$ rosrun deprag_cgi deprag_cgi_node.py
```
To create a new program we first must call the deprag_create service, establishing basic program settings like its name, program number, etc.
```
$ rosservice call /deprag_create "{pgm_number: 32, head_title: 'Example', head_dir: 0, head_feed: 0, torque_unit: 0}"
```
The service will return an unique id that we must hold on to because it is needed to add further steps to the program and later pubish it.

id: 11

Now to add a loosen on extern step we call the deprag_loosen_on_extern service, making sure to set cid to the value returned by the deprag_create call.
```
$ rosservice call /deprag_loosen_on_extern "{cid: 11, max_time: 32, angle_min: 60, angle_max: 320, torque_min: 5.0, torque_max: 15.0, rpm: 30, create_val: 1}"
```
Again the id is returned.

id:11

If id happens to be a negative value an error occured, therefore any software using these services should check if id is smaler than zero. Each value smaler than zero coresponds to a line of code where the error occured, therfore a text search of deprag_cgi_node.py will yield this line.

To chain steps we simply call their services in order.

$ rosservice call /deprag_tighten_on_extern "{cid: 11, max_time: 36, angle_min: 20, angle_max: 630, torque_min: 5.0, torque_max: 15.0, rpm: 30, create_val: 1}"

Finaly to publish the program to the deprag device we call the deprag_publish service

$ rosservice call /deprag_publish "cid: 11"

The program is now avaliable on the deprag device and the id is invalid, do not use it anymore.



## Error Coding 
When an error occures while a service is called the service will return a negative id. To find the corresponding error and line of code
simply conduct a text search in the deprag_cgi_node.py script.

## References
Implemented according to the manual Operating manual: External Program Creation, 013777XX


## Acknowledgement
Sponsored by the Ministry of the Environment Baden-Württemberg, in the context of the Strategic Dialogue Automotive Industry, and supervised by the Project Management Agency Karlsruhe (PTKA). Funding number: L7520101

https://www.ipa.fraunhofer.de/de/referenzprojekte/DeMoBat.html

## Contact
For more information please feel free to contact: <br />
M.Sc. Anwar Al Assadi<br />
Wissenschaftlicher Mitarbeiter<br />
Fraunhofer‐Institut für<br />
Produktionstechnik und Automatisierung IPA<br />
Abteilung Roboter- und Assistenzsysteme<br />
Nobelstraße 12 │ 70569 Stuttgart <br />
Telefon +49 711 970-1264 <br />
anwar.alassadi@ipa.fraunhofer.de<br />
