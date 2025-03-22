# 8767 Code Explanation

[Constants](Constants.md)

## Constants

### CAN ID's
We have various CAN IDs to each Spark Max on our robot to accureatly control everything and condense the robot.java code.
#### Drive Motors
Our drive motor CAN ids include...
- Front Left Motor: CAN ID 11
- Front Right Motor: CAN ID 12
- Rear Left Motor:  CAN ID 1
- Rear Right Motor: CAN ID 
#### Subsystem Motors (Season Specific)
Our subsystem motor CAN ids include...
- Shooter Motor 1: CAN ID 10
- Shooter Motor 2: CAN ID 13
- Intake Motor: CAN ID 8
- Conveyer Motor: CAN ID 5
#### Electronic Components
We have multiple electronical componenets within our robot to control certain elements
- Power Distribution Hub: CAN ID 7
- Pnuematic Hub: CAN ID 12
### Controller Definitions (Buttonmap)
We have all of our buttons set to special IDs
#### Logitech F310 Controller
##### D-Mode
- X Button: ID 1
- A Button: ID 2
- B Button: ID 3
- Y Button: ID 4
- LB Button: ID 5
- RB Button: ID 6
- LT Button: ID 7
- RT Button: ID 8
- Back Button: ID 9
- Start Button: ID 10
- Left Stick Y Axis: ID 1
- Left Stick X Axis: ID 0
- Right Stick Y Axis: ID 3
- Right Stick X Axis: ID 2
##### X Mode
- X Button: ID 3
- A Button: ID 1
- B Button: ID 2
- Y Button: ID 4
- LB Button: ID 5
- RB Button: ID 6
- Back Button: ID 7
- Start Button: ID 8
- Left Stick Y Axis: ID 1
- Left Stick Y Axis: ID 0
- Right Stick Y Axis: ID 5
- Right Stick X Axis: ID 4
#### Xbox Controller
- X Button: ID 3
- A Button: ID 1
- B Button: ID 2
- Y Button: ID 4
- LB Button: ID 5
- RB Button: ID 6
- Back Button: ID 7
- Start Button: ID 8
- LT Axis: ID 2
- RT Axis: ID 3
- Left Stick Y Axis: ID 1
- Left Stick X Axis: ID 0
- Right Stick Y Axis: ID 5
- Right Stick X Axis: ID 4
#### Logitech Attack 3 Joystick
- Trigger Button: ID 1
- Button 2-11: ID 2-11
- Y Axis: ID 0
- X Axis: ID 1
#### Logitech Extreme 3D Pro Joystick
- Trigger Button: ID 1
- Thumb Button: ID 2
- Button 3-12: ID 3-12
- X Axis: ID 1
- Y Axis: ID 2
- rotationZ Axis: ID 3
- Bottom Slider Axis: ID 4
- Hat Y Axis: ID 5
- Hat X Axis: ID 6
### Pnuematic IDs
We have multiple IDs for the forward and reverse on our solenoids
#### Drivetrain (Butterfly Specific)
- Drivetrain Solenoid Forward: ID 0
- Drivetrian Solenoid Reverse: ID 1
- Pnuematic Module Solenoid: ID 0
#### Subsystem (Season Specific)
- None as of now.
### Math Equations
We have various math equations to help with distance travelled and others in Autonomous and Teleop
#### countsPerInchMecanum
- This calculates the amount of encoder counts per inch of the mecanum wheels
- (encoderPerRev x gearRatioMecanum) / (wheelDiameterMecanum x pi)
- All of these dependences are listed below
### countsPerInchTraction
- This calculates the amount of encoder counts per inch of the traction wheels
- (encoderPerRev x gearRatioTraction) / (wheelDiameterTraction x pi)
- All of these dependencies are listed below
##### encoderPerRev
- encoderPerRev is the amount of encoders read on each rotation of a NEO motor
- 42 Encoder Count
##### Gear Ratio (Mecanum, Butterfly Specific)
- This is the gear ratio the mecanum wheels on the butterfly modules
- 10.71/1.0 ratio
##### Gear Ratio (Traction, Butterfly Specific)
- This is the gear ratio of the traction wheels on the butterfly modules
- 10.71/1.0
##### Wheel Diameter (Andymark SR Mecanum Wheels
- 6 inches
#### Wheel Diameter (Andymark Traction Wheels)
- 4 inches

## Robot.java
Our main robot file has lots of code to ensure our robot functions to the best of its ability during competition.
### Drivetrain Code (Arcade)
- We use an arcade drive at the moment to drive our tank drive
`m_robotDrive.arcadeDrive(js_Driver.getRawAxis(constants.axisD_lUpDown), js_Driver.getRawAxis(constants.axisD_rLeftRight));`
- Left Stick Y runs the forward/backward movement on the robot.
- Right Stick X runs the left/right turning movement on the robot.
### Supersystem Code
#### Color Sensor
- We use a color sensor to control the intake and conveyer system to limit driver error
- We use the ADC value (Red) to check if the orange note is inside of our robot
`if (m_colorSensor.getRed() > 200)`
- Then, when this is true, we stop all the intake motors to keep the note away from the flywheels
- Now, if the flywheel shooter button is pressed, it overrides the color sensor to allow for a shot in the AMP or SPEAKER
  ` if (js_Driver.getRawButton(constants.btnD_X) ||         js_Driver.getRawButton(constants.btnD_B)) {`
- This checks if the either buttons are pressed
- Else, if the color sensor does not see anything, the normal intake and conveyer system works 
      if (js_Driver.getRawButton(constants.btnD_LB)) {
        // Bring note to shooter
        m_Conveyor.set(0.6);
      } else if (js_Driver.getRawButton(constants.btnD_LT)) {
        // Kick the note back to intake
        m_Conveyor.set(-1);
      } else {
        m_Conveyor.set(0);
      }
      if (js_Driver.getRawButton(constants.btnD_RB)) {
        // Bring note to conveyor
        m_Intake.set(0.7);
      } else if (js_Driver.getRawButton(constants.btnD_RT)) {
        // Kick the note out of robot
        m_Intake.set(-0.7);
      } else {
        m_Intake.set(0);
      }
#### Shooter Code
- We use our trigger axis on an xbox controller to control variable speed of the shooting motors
` m_Shooter1.set(js_Driver.getRawAxis(constants.axisXB_RTrigger));
    m_Shooter2.set(js_Driver.getRawAxis(constants.axisXB_RTrigger));`




## Anything Else
