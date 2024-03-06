proc addLargeOrangeCone {fileID n s t} {
	# Add large organe cone to the opened infofile with handle fileID
	# The cone is added as a traffic object number $n
	# The cone is added to the main route at offset ($s %t)
    puts $fileID "Traffic.$n.ObjectKind = StatWithName"
	puts $fileID "Traffic.$n.ObjectClass = Unknown"
	puts $fileID "Traffic.$n.Name = CN$n"
	puts $fileID "Traffic.$n.Info = Large Orange Start Cone"
	puts $fileID "Traffic.$n.Movie.Geometry = TrafficCones/TrafficCone_Large_Orange.obj"
	puts $fileID "Traffic.$n.Color = 1.0 0.65 0.0"
	puts $fileID "Traffic.$n.Basics.Dimension = 0.285 0.285 0.505"
	puts $fileID "Traffic.$n.Basics.Offset = 0.0 0.0"
	puts $fileID "Traffic.$n.Basics.Fr12CoM = 0.143"
	puts $fileID "Traffic.$n.Init.Orientation = 0.0 0.0 0.0"
	puts $fileID "Traffic.$n.RCSClass = RCS_GuardRailPost"
	puts $fileID "Traffic.$n.DetectMask = 1 1"
	puts $fileID "Traffic.$n.Route = 0 1"
	puts $fileID "Traffic.$n.Init.Road = $s $t"
	puts $fileID "Traffic.$n.Init.v = 0.0"
	puts $fileID "Traffic.$n.FreeMotion = 0"
	puts $fileID "Traffic.$n.Lighting = 0"
	puts $fileID "Traffic.$n.UpdRate = 200"
	puts $fileID "Traffic.$n.Motion.Kind = Course"
	puts $fileID "Traffic.$n.Motion.mass = 1.05"
	puts $fileID "Traffic.$n.Man.TreatAtEnd = FreezePos"
	puts $fileID "Traffic.$n.Man.N = 0"
}

proc addOrangeCone {fileID n s t} {
	# Add small organe cone to the opened infofile with handle fileID
	# The cone is added as a traffic object number $n
	# The cone is added to the main route at offset ($s %t)
    puts $fileID "Traffic.$n.ObjectKind = StatWithName"
	puts $fileID "Traffic.$n.ObjectClass = Unknown"
	puts $fileID "Traffic.$n.Name = CN$n"
	puts $fileID "Traffic.$n.Info = Small Orange Exit/Entry Lane Cone"
	puts $fileID "Traffic.$n.Movie.Geometry = TrafficCones/TrafficCone_Small_Orange.obj"
	puts $fileID "Traffic.$n.Color = 1.0 0.65 0.0"
	puts $fileID "Traffic.$n.Basics.Dimension = 0.228 0.228 0.325"
	puts $fileID "Traffic.$n.Basics.Offset = 0.0 0.0"
	puts $fileID "Traffic.$n.Basics.Fr12CoM = 0.114"
	puts $fileID "Traffic.$n.Init.Orientation = 0.0 0.0 0.0"
	puts $fileID "Traffic.$n.RCSClass = RCS_GuardRailPost"
	puts $fileID "Traffic.$n.DetectMask = 1 1"
	puts $fileID "Traffic.$n.Route = 0 1"
	puts $fileID "Traffic.$n.Init.Road = $s $t"
	puts $fileID "Traffic.$n.Init.v = 0.0"
	puts $fileID "Traffic.$n.FreeMotion = 0"
	puts $fileID "Traffic.$n.Lighting = 0"
	puts $fileID "Traffic.$n.UpdRate = 200"
	puts $fileID "Traffic.$n.Motion.Kind = Course"
	puts $fileID "Traffic.$n.Motion.mass = 0.45"
	puts $fileID "Traffic.$n.Man.TreatAtEnd = FreezePos"
	puts $fileID "Traffic.$n.Man.N = 0"
}

proc addBlueCone {fileID n s t} {
	# Add small blue cone to the opened infofile with handle fileID
	# The cone is added as a traffic object number $n
	# The cone is added to the main route at offset ($s %t)
    puts $fileID "Traffic.$n.ObjectKind = StatWithName"
	puts $fileID "Traffic.$n.ObjectClass = Unknown"
	puts $fileID "Traffic.$n.Name = CN$n"
	puts $fileID "Traffic.$n.Info = Left Blue Cone"
	puts $fileID "Traffic.$n.Movie.Geometry = TrafficCones/TrafficCone_Small_Blue.obj"
	puts $fileID "Traffic.$n.Color = 0 0 1"
	puts $fileID "Traffic.$n.Basics.Dimension = 0.228 0.228 0.325"
	puts $fileID "Traffic.$n.Basics.Offset = 0.0 0.0"
	puts $fileID "Traffic.$n.Basics.Fr12CoM = 0.114"
	puts $fileID "Traffic.$n.Init.Orientation = 0.0 0.0 0.0"
	puts $fileID "Traffic.$n.RCSClass = RCS_GuardRailPost"
	puts $fileID "Traffic.$n.DetectMask = 1 1"
	puts $fileID "Traffic.$n.Route = 0 1"
	puts $fileID "Traffic.$n.Init.Road = $s $t"
	puts $fileID "Traffic.$n.Init.v = 0.0"
	puts $fileID "Traffic.$n.FreeMotion = 0"
	puts $fileID "Traffic.$n.Lighting = 0"
	puts $fileID "Traffic.$n.UpdRate = 200"
	puts $fileID "Traffic.$n.Motion.Kind = Course"
	puts $fileID "Traffic.$n.Motion.mass = 0.45"
	puts $fileID "Traffic.$n.Man.TreatAtEnd = FreezePos"
	puts $fileID "Traffic.$n.Man.N = 0"
}

proc addYellowCone {fileID n s t} {
	# Add small yellow cone to the opened infofile with handle fileID
	# The cone is added as a traffic object number $n
	# The cone is added to the main route at offset ($s %t)
    puts $fileID "Traffic.$n.ObjectKind = StatWithName"
	puts $fileID "Traffic.$n.ObjectClass = Unknown"
	puts $fileID "Traffic.$n.Name = CN$n"
	puts $fileID "Traffic.$n.Info = Right Yellow Cone"
	puts $fileID "Traffic.$n.Movie.Geometry = TrafficCones/TrafficCone_Small_Yellow.obj"
	puts $fileID "Traffic.$n.Color = 1 1 0"
	puts $fileID "Traffic.$n.Basics.Dimension = 0.228 0.228 0.325"
	puts $fileID "Traffic.$n.Basics.Offset = 0.0 0.0"
	puts $fileID "Traffic.$n.Basics.Fr12CoM = 0.114"
	puts $fileID "Traffic.$n.Init.Orientation = 0.0 0.0 0.0"
	puts $fileID "Traffic.$n.RCSClass = RCS_GuardRailPost"
	puts $fileID "Traffic.$n.DetectMask = 1 1"
	puts $fileID "Traffic.$n.Route = 0 1"
	puts $fileID "Traffic.$n.Init.Road = $s $t"
	puts $fileID "Traffic.$n.Init.v = 0.0"
	puts $fileID "Traffic.$n.FreeMotion = 0"
	puts $fileID "Traffic.$n.Lighting = 0"
	puts $fileID "Traffic.$n.UpdRate = 200"
	puts $fileID "Traffic.$n.Motion.Kind = Course"
	puts $fileID "Traffic.$n.Motion.mass = 0.45"
	puts $fileID "Traffic.$n.Man.TreatAtEnd = FreezePos"
	puts $fileID "Traffic.$n.Man.N = 0"
}


proc cleanUp {fileName} {
	# Programtically remove all traffic objects from the TestRun infofile
	
	# Read all lines from the infofile
	set fileID [open Data/TestRun/$fileName r]
	set fc [read $fileID]
	
	# Seek all Traffic.NUM lines and delete them
	regsub -all -line {Traffic\.\d.+\n} $fc {""} fc
	
	# Re-open file and clear its contents
	set fileID [open Data/TestRun/$fileName w]
	
	# Re-insert the old TestRun minus all the traffic data
	puts -nonewline $fileID $fc
	close $fileID
}

# Get the relative name of the current TestRun
set fileName [SimInfo testrun]
set fileNameBak Data/TestRun/
append fileNameBak $fileName _bak
file copy -force Data/TestRun/$fileName $fileNameBak

# Clean ALL traffic objects from the TestRun
cleanUp $fileName

# Get the route length and set the number of cones accordingly
set routeLen [IFileValue TestRun Road.Route.0.Length]
set coneDist 5
set nCones [expr "int($routeLen/$coneDist)"]

# Write the number of cones to the TestRun infofile
IFileModify TestRun Traffic.N [expr "2*$nCones+4"]

# Open TestRun infofile and get a handle to it
set fileID [open Data/TestRun/$fileName a]

# Add 2x2 large orange cones at the start
addLargeOrangeCone $fileID 0 0 1.5
addLargeOrangeCone $fileID 1 0 -1.5
addLargeOrangeCone $fileID 2 0.5 1.5
addLargeOrangeCone $fileID 3 0.5 -1.5

# Add blue/yellow cones on the left/right side of the track
for {set i 0} {$i < $nCones} {incr i} {
    addBlueCone $fileID [expr "2*$i+4"] [expr "5*$i+5"] 1.5
	addYellowCone $fileID [expr "2*$i+5"] [expr "5*$i+5"] -1.5
}

# Close and flush the IFile to apply changes
close $fileID
IFileFlush

