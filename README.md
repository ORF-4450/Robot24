### Robot24 Base Code
----------------------------------------------------------------------------
FRC Team 4450 2024 Robot Control program base code.

![The USS ProtoStar](resources/bot.png)

This is the 2024 base robot control program created by the Olympia Robotics Federation (FRC Team 4450). 
It is intended as a starting point to develop complete season specific versions to actually operate the
2024 robot, USS Protostar for FRC game CRESCENDO. It can also be used as a starting point for future
robot control programs. 

This code contains the minimal support for drive base, vision, Path Planning and AdvantageScope from which 
code can be developed for any robot. There are some season specific items, particularly the Constants and Assignments files, which reflect the actual 2024 robot conventions, which would be needed to write a complete program for the 2024 robot. See PR#10 for more details.

----------------------------------------------------------------------------
## Instructions to setup development environment for VS Code
1) Follow the instructions [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html) to setup the JDK, Visual Studio Code, the FRC plugins and tools. Do not install the C++ portion. You do not need the FRC Update Suite to compile code.
2) Clone this repository to local folder.
3) Open that folder in Visual Studio Code.
4) Build the project using the build project command from the WPILib commands list.

### If RobotLib gets an update:
1) download the RobotLib.json file from the RobotLib Github repo and drop it into the vendordeps folder inside the project folder. Build the project.
****************************************************************************************************************
Version 24.bc.1

*   Post-season update of base code for 2024.

R. Corn, April 30 2024

Version 24.bc.0

*   First release of 2024 Base Code.
*   Note that there was extensive development done between end of 2023 and this release. The history of
    that development is in the Robot24B project MaxSwerve2 branch.
 
R. Corn, February 6 2024
