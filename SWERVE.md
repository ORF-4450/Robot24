### Swerve code notes 2024.

Robot23 used the Swerve Drive Specialties swerve modules. It used a highly modified version
of the sample SDS code. SDS code had a detailed configuration procedure to align the wheels
correctly and obtain the CanCoder offsets needed by that code. That procedure was documented
in this file.

For 2024, it was decided to move to the REV MaxSwerve modules. These modules and the SparkMax
motor controllers used by the Neo and Neo550 motors have a built in configuration scheme which
is documented in the MaxSwerve manual. As such, it not documented here.

In anticipation of MaxSwerve, the SDS based code, partially located in RobotLib and Robot23
DriveBase class, was modified to support MaxSwerve. SDS code supports Neos as drive and steer
motors via the SparkMax controller. However, the MaxSwerve uses the REV through bore encoders
instead of SDS's CanCoders. This encoder difference was the main modification to the SDS code.

In parallel to that effort, the team also took the MaxSwerve sample code and got it ready to
test as a back up for the SDS modifications.

When the MaxSwerve drive base prototype was finished and testing began, the MaxSwerve sample
code worked first time but the SDS code did not. Development of the MaxSwerve code proceeded
at a rapid pace. A fix was applied to the SDS code, but due to the rapid advancement of the
MaxSwerve code, not only for swerve driving, but support for April Tag vision and advanced
navigation, the SDS fix was never tested. Things quickly reached a point where it made sense 
to just adopt the MaxSwerve based code into Robot24 and leave the SDS code for a later date 
or perhaps never. In all honesty, the SDS code is very complex where the MaxSwerve is simpler.
