# RSDS ROS Client Node

## How-To Run
The main CarMaker ROS node (CMNode) will run a separate RSDS ROS client node for the first camera of every CameraRSI cluster in CarMaker. A CameraRSI cluster is a collection of CameraRSI sensors that executes on the same GPU and transmits data on the same TCP/IP port.

The RSDS nodes currently do not support additional cameras per cluster. If a cluster has two or more cameras, only one will properly transmit data over ROS, and it may not be the one originally intended for the RSDS node. This is a limitation inside CMNode that we hope to alleviate in the future. Please put only once CameraRSI per cluster for the time being.

## Performance Suggestions
For better performance, run Movie instances in Performance mode (needs to be activated before running RSDS client, if ExclusiceMode is on):
`IPGMovie -> View -> Qualitiy Settings... -> Mode`

