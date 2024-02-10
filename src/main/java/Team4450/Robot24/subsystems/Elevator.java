package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_INNER;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_LEFT;
import static Team4450.Robot24.Constants.ELEVATOR_MOTOR_RIGHT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import Team4450.Lib.Util;
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
    private CANSparkMax motorInner = new CANSparkMax(ELEVATOR_MOTOR_INNER, MotorType.kBrushless);

    private SparkLimitSwitch lowerLimitSwitch;
    private SparkLimitSwitch upperLimitSwitch;
  
    private RelativeEncoder mainEncoder;
    private RelativeEncoder innerEncoder;

    private Mechanism2d mechanism = new Mechanism2d(100,100);
    private MechanismRoot2d mechRoot = mechanism.getRoot("root", 5, 0);
    private MechanismLigament2d elevatorLigament = mechRoot.append(new MechanismLigament2d("elevator", 50, 90));

    public Elevator() {
        Util.consoleLog();

        motorFollower.follow(motorMain);
        lowerLimitSwitch = motorFollower.getReverseLimitSwitch(Type.kNormallyOpen);
        upperLimitSwitch = motorFollower.getForwardLimitSwitch(Type.kNormallyOpen);

        lowerLimitSwitch.enableLimitSwitch(true);
        upperLimitSwitch.enableLimitSwitch(true);

        mainEncoder = motorMain.getEncoder();
        innerEncoder = motorInner.getEncoder();

        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motorMain, DCMotor.getNEO(1));
        }

        SmartDashboard.putData("Elevator", mechanism);
    }

    @Override
    public void periodic() {
        if (lowerLimitSwitch.isPressed())
            mainEncoder.setPosition(0);
        elevatorLigament.setLength(mainEncoder.getPosition());
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
        // mainEncoder.setPosition(mainEncoder.getPosition() + speed);
        // Util.consoleLog("%f", mainEncoder.getPosition());
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
