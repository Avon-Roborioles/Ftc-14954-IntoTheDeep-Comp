Hardware map:

USB:
"limelight"

Control Hub:
M0: "backRight"
M1: "frontRight"
M2: "frontLeft"
M3: "backLeft"

S1: "extension"
S2: "blinkin"


I2C0: "odometry"
I2C1: inop

Expansion Hub:
M0: "Intake"
M1: "climb"
M2: "liftMotor"


S0: "swingArm" 0 with preloaded spring toward switch
S1: "allianceColor"


S5: "wrist"


I2C1: "RampColor"
I2C2: "intakeColor"
I2C3: "boxDistance"

D7:"liftDown"
D5: "extensionIn"
D3: "swingArmDown"

TeleOp (Gamepad Driving)
Driver:
Right Bumper = Slow Drive
Left Bumper= Score
Y = Heading Reset
UP DPAD = SpecimenIntakeGrab
Down DPAD = Specimen score
Left DPAD = In Bucket recover

Operator: 
Left Bumper = Extend Extension 
Right Bumper = Retract Extension
A = Intake Bottom Bucket
B = Cancel Motors command (Intake, Passthrough and Lift)
X = Wrist toggle between low and handoff
Y = Intake Top Bucket
DPAD UP = Specimen Bar Command
DPAD DOWN = Lift Down Command
DPAD RIGHT = Eject PassThrough
DPAD LEFT = Intake to Pass End 
BACK = Intake Reject Command
RIGHT STICK BUTTON = Lift Reset Command
Left Joystick = Hang

 