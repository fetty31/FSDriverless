#---------- ROS Message to map external signals to CarMaker Vehicle Control Quantities ---------- 
#*** Most of VC Interface outlined here...uncomment relevant quantities and assign in CMNode as demonstrated. ***

#----- Flag to Use VC Input -----
bool use_vc             # Set this flag to 1 in order to start writing to CarMaker VC quantities in this message.
                        #   --> Used so that zeros are not sent over topic when no node is publishing to steer, brake, gas, etc.
                        #   --> Allows driver or DiL to drive the vehicle via DM quantities and not be overwritten by VC if not needed.

#----- Key / StartStop Button -----
#int8 key                # Vehicle key position (integer):
                        #   0: Key out
                        #   1: Key in, power off
                        #   2: Key in, power accessory
                        #   3: Key in, power on
                        #   4: Key in, starter active

#bool sst                # Powertrain start-stop button (boolean): 
                        #   0=off
                        #   1=on

#----- Transmission -----
int8 selector_ctrl      # PRNDL Position
                        #   -9 = Park
                        #   -1 = Reverse
                        #    0 = Neutral
                        #    1 = Drive
                        #    2 = Manual gear selection (requires Manumatic transmission in CM model)

#int8 gear               # Gear number for manual transmission...uncomment for use with manual transmission model.

#----- Pedals -----
float64 gas             # Gas pedal position [0 - 1]
float64 brake           # Brake pedal position [0 - 1]
#float64 clutch         # Clutch pedal position [0 - 1]...uncomment for use with manual transmission model.

#----- Steering -----
float64 steer_ang       # Steer angle [rad]
float64 steer_ang_vel   # Steer angle velocity [rad/s]
float64 steer_ang_acc   # Steer angle acceleration [rad/s^2]
#float64 steer_trq      # Steering torque at steering wheel [N*m]...uncomment for use with GenTorque steering model (steer by torque).

#----- Lights -----
#bool brakelight         # Brake light on (boolean)
#bool daytimelights      # Daytime running light on (boolean)
#bool foglights_left     # Front fog left/right light on (boolean)
#bool foglights_right
#bool foglights_rear     # Rear fog light on (boolean)
#bool highbeams          # High beam/full headlight on (boolean)
#int8 indicator_left     # Indicator left/right (integer):
#int8 indicator_right    #   0=Off; 1=Indicator on, light off; 2=Indicator on, light on
#bool lowbeams           # Low beam/dipped headlight on (boolean)
#bool parklight_left     # Parking left/right light on (boolean)
#bool parklight_right    
#bool reverselights      # Reversing light on (boolean)
