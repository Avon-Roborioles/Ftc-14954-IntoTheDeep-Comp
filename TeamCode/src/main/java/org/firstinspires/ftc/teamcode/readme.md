Hardware map:

USB:
"limelight"

Control Hub:
M0: "backRight"
M1: "frontRight"
M2: "frontLeft"
M3: "backLeft"

S0: "extension"
S1: "wrist"
S2: "blinkin"


I2C0: "odometry"
I2C1: "intakeDistance"
I2C2: "intakeColor1"
I2C3: "intakeColor2"

Expansion Hub:
M0: "Intake"
M1: "climb"
M2: "liftMotor"
M3: "Pass"

S0: "swingArm" 0 with preloaded spring toward switch
S2: "allianceColor"

I2C0: "boxDistance"

D7:"liftDown"
D5: "extensionIn"
D3: "swingArmDown"

TeleOp (Gamepad Driving)
Driver:
Right Bumper = Slow Drive

Operator: 
Left Bumper = Extend Extension 
Right Bumper = Retract Extension
A = Intake Command
B = Cancel Motors command (Intake, Passthrough and Lift)
X = Wrist toggle between low and handoff
Y = Score Command
DPAD UP = Specimen Bar Command
DPAD DOWN = Lift Down Command
DPAD RIGHT = Bottom Bucket Command
DPAD LEFT = Toggle Alliance Command (Will Be Removed Soon)
BACK = Intake Reject Command
RIGHT STICK BUTTON = Lift Reset Command

 