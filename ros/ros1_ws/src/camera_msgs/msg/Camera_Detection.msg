#~~~~~~~~~~ Custom ROS message for HiFi Camera Sensor ~~~~~~~~~~

#~~~~~ Quantities match UAQs for HiFi Camera Sensor in CM
#~~~~~ See 'Reference Manual --> User Accessible Quantities --> Sensors --> Camera Sensor' for full descriptions.

int32           ObjID                   # global ID to identify object (integer) 

int8            ObjectType                    # camera object type (integer)
                                        #   0 - Car
                                        #   1 - Truck
                                        #   2 - Bicycle
                                        #   3 - Pedestrian
                                        #   4 - Traffic sign
                                        #   5 - Traffic light

int64           nVisPixels              # number of visible pixels (integer)

float64         Confidence              # indicates how much of the object is visible

float64         MBR_BL_X                # minimum bounding rectangle - bottom left and top right points
float64         MBR_BL_Y                
float64         MBR_BL_Z                
float64         MBR_TR_X                
float64         MBR_TR_Y                
float64         MBR_TR_Z                

int8            Facing                  # 0: not facing sensor, 1: facing sensor

int8            LightState              # Traffic light state/phase (integer):
                                        #   0: All lights off
                                        #   1: Green light on
                                        #   2: Yellow light on
                                        #   3: Red light on
                                        #   4: Red-Yellow light on

float64         SignMain_Val0           # User defined sign attribute values
float64         SignMain_Val1           
float64         SignSuppl1_Val0         
float64         SignSuppl1_Val1         
float64         SignSuppl2_Val0         
float64         SignSuppl2_Val1        