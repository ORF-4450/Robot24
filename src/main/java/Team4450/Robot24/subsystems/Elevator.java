package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_INNER;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_LEFT;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_RIGHT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import Team4450.Lib.Util;
import Team4450.Robot24.AdvantageScope;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private CANSparkMax motorMain = new CANSparkMax(ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
    private CANSparkMax motorFollower = new CANSparkMax(ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);
    private CANSparkMax motorCenterstage = new CANSparkMax(ELEVATOR_MOTOR_INNER, MotorType.kBrushless);

    private SparkLimitSwitch lowerLimitSwitch;
    private SparkLimitSwitch upperLimitSwitch;
  
    private RelativeEncoder mainEncoder;
    private RelativeEncoder followEncoder;
    private RelativeEncoder centerstageEncoder;

    public Elevator() {
        Util.consoleLog();

        motorFollower.follow(motorMain, true);
        motorFollower.setInverted(true);

        motorFollower.setIdleMode(IdleMode.kBrake);
        motorMain.setIdleMode(IdleMode.kBrake);
        motorCenterstage.setIdleMode(IdleMode.kBrake);

        lowerLimitSwitch = motorFollower.getReverseLimitSwitch(Type.kNormallyOpen);
        upperLimitSwitch = motorFollower.getForwardLimitSwitch(Type.kNormallyOpen);

        lowerLimitSwitch.enableLimitSwitch(true);
        upperLimitSwitch.enableLimitSwitch(true);

        mainEncoder = motorMain.getEncoder();
        followEncoder = motorFollower.getEncoder();
        centerstageEncoder = motorCenterstage.getEncoder();

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motorMain, DCMotor.getNEO(1));
        }
    }

    @Override
    public void periodic() {
        if (lowerLimitSwitch.isPressed())
            mainEncoder.setPosition(0);
        
        AdvantageScope.getInstance().setElevatorHeight(0.01*mainEncoder.getPosition());
        AdvantageScope.getInstance().setCarriageHeight(0.01*centerstageEncoder.getPosition());

        SmartDashboard.putNumber("Elevator1 Encoder", mainEncoder.getPosition());
        SmartDashboard.putNumber("Elevator2 Encoder", followEncoder.getPosition());
        SmartDashboard.putNumber("Centerstage Encoder", centerstageEncoder.getPosition());

    }

    // @Override
    // public void simulationPeriodic() {
    //     REVPhysicsSim.getInstance().run();
    // }

    /**
     * move elevator in direction based on speed
     * @param speed (such as from a joystick value)
     */
    public void move(double speed) {
        motorMain.set(speed);
        if (RobotBase.isSimulation() && 0.01*(mainEncoder.getPosition()+speed) > 0 && 0.01*(mainEncoder.getPosition()+speed) < 0.5)
            mainEncoder.setPosition(mainEncoder.getPosition() + speed);
        // Util.consoleLog("%f", mainEncoder.getPosition());
    }

    public void moveInner(double speed) {
        motorCenterstage.set(speed);
        if (RobotBase.isSimulation() && 0.01*(centerstageEncoder.getPosition()+speed) > 0 && 0.01*(centerstageEncoder.getPosition()+speed) < 0.5)
            centerstageEncoder.setPosition(centerstageEncoder.getPosition() + speed);
    }

    /**
     * The height of the elevator (measured at shooter pivot)
     * above the ground
     * @return the height in meters of pivot above ground
     */
    public double getHeight() {
        return 1; // TODO: fix this because its just made up.
    }

}
