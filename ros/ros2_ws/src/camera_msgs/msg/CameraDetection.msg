#~~~~~~~~~~ Custom ROS message for HiFi Camera Sensor ~~~~~~~~~~

#~~~~~ Quantities match UAQs for HiFi Camera Sensor in CM
#~~~~~ See 'Reference Manual --> User Accessible Quantities --> Sensors --> Camera Sensor' for full descriptions.

int32           objid                   # global ID to identify object (integer) 

int8            objecttype                    # camera object type (integer)
                                        #   0 - Car
                                        #   1 - Truck
                                        #   2 - Bicycle
                                        #   3 - Pedestrian
                                        #   4 - Traffic sign
                                        #   5 - Traffic light

int64           nvispixels              # number of visible pixels (integer)

float64         confidence              # indicates how much of the object is visible

float64         mbr_bl_x                # minimum bounding rectangle - bottom left and top right points
float64         mbr_bl_y                
float64         mbr_bl_z                
float64         mbr_tr_x                
float64         mbr_tr_y                
float64         mbr_tr_z                

int8            facing                  # 0: not facing sensor, 1: facing sensor

int8            lightstate              # Traffic light state/phase (integer):
                                        #   0: All lights off
                                        #   1: Green light on
                                        #   2: Yellow light on
                                        #   3: Red light on
                                        #   4: Red-Yellow light on

float64         signmain_val0           # User defined sign attribute values
float64         signmain_val1           
float64         signsuppl1_val0         
float64         signsuppl1_val1         
float64         signsuppl2_val0         
float64         signsuppl2_val1        