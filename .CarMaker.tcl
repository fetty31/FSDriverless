##
## .CarMaker.tcl 
##
## FSAI project startup script
## Loads the provided basic test run at startup.

Project::CheckProject
LoadTestRun "FS_autonomous_TrackDrive"
::CMRosIF::ROS_Launch -mode launch
Application connect
Application start
