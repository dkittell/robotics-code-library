2021 Competition Robot Code
===========================

This repo is for the code that goes on the robot and driver station to assist robot controls.

Driver Station Code
-------------------

Driver Station code is located in the "DriverStation" code directory.

*   PowerShell
    *   Firewall.ps1 - This file assists with Firewall changes to the drive station computer.
    *   FRCTest.ps1 - This file assists with the testing of network connection between the robot and the drive station.
*   ShuffleBoard
    *   shuffleboard.json - This file is a work in progress for the shuffleboard.
*   SmartDashboard
    *   2020_FRC3535_SmartDashboard.xml - This is a view of the smart dashboard
    *   2020_Robot.xml - This is a view of the smart dashboard

* * *

Robot Code
----------

Robot code (written in Java) is located in the "RobotCode" directory.

### Prerequisites:

*   Laptop Setup for programming
    *   Latest Java JDK - [Java JDK Download](https://www.oracle.com/java/technologies/javase-jdk15-downloads.html)
    *   Latest WPILib Release - [WPILib Download](https://github.com/wpilibsuite/allwpilib/releases)  
        Instructions at [https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html)
    *   PlayingWithFusion Library Installed - [Library Download](https://www.playingwithfusion.com/docview.php?docid=1205&catid=9012)

### Robot Code - Description

The robot this year has:
*   5 [PlayingWithFusion Venom](https://www.playingwithfusion.com/productview.php?pdid=99) motors
    *   4 Driving Motors - This allows for easier programming for encoder counts
    *   1 Shooter Motor - This allows for easier programming for PID Loop


Links
----------
*   roboRio
    *   http://169.254.208.241/#!/SystemConfig
    *   http://10.35.35.2/#!/SystemConfig
*   Limelight - Ethernet Port is PPOE
    *   172.16.128.72:5809 - Camera Data Stream
    *   limelight.local:5801 - Configuration Link
