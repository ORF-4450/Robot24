package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_INNER;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_LEFT;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_RIGHT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private CANSparkMax motorMain = new CANSparkMax(ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax motorFollower = new CANSparkMax(ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);
    private CANSparkMax motorInner = new CANSparkMax(ELEVATOR_MOTOR_INNER, MotorType.kBrushless);

    private SparkLimitSwitch lowerLimitSwitch;
    private SparkLimitSwitch upperLimitSwitch;
  
    

    public Elevator() {
        Util.consoleLog();

        motorFollower.follow(motorMain);
        lowerLimitSwitch = motorFollower.getReverseLimitSwitch(Type.kNormallyOpen);
        upperLimitSwitch = motorFollower.getForwardLimitSwitch(Type.kNormallyOpen);

        lowerLimitSwitch.enableLimitSwitch(true);
        upperLimitSwitch.enableLimitSwitch(true);
    }
}
