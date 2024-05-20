# Load Carriers Tracker
## Additional libraries
### Unique ID (For UUID generation)
```
sudo apt-get install ros-noetic-unique-id
```
## Usage
First, run the transform_detections node to transform the detections from robot frame to the global CS.
```
rosrun load_carriers_tracker transform_detections_node
```
Then, launch the carrier_trackers.launch file.
```
roslaunch load_carriers_tracker carrier_trackers.launch
```
