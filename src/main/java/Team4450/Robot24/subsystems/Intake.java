package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.INTAKE_MOTOR_1;
import static Team4450.Robot24.Constants.INTAKE_MOTOR_2;
import static Team4450.Robot24.Constants.INTAKE_SPEED;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake for the 2024 robot the USS ProtoStar
 */
public class Intake extends SubsystemBase {
    private CANSparkMax motor1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushless);

    private double  motorSpeed = INTAKE_SPEED;
    private boolean isrunning = false;

    public Intake() {
        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
        
        motor2.follow(motor1); // they need to do the same things
        // we don't reverse one because they are physically mounted reversed

        Util.consoleLog("Intake created!");
    }

    /**
     * run the intake at INTAKE_SPEED * speedfactor
     * @param speedfactor from -1 to 0 to 1
     */
    public void start(double speedfactor) {
        // Util.consoleLog();

        isrunning = true;
        updateDS();

        SmartDashboard.putNumber("intake_speedfactor", speedfactor);

        motor1.set(Util.clampValue(speedfactor, 1) * motorSpeed);
        // motor2.set(Util.clampValue(speedfactor, 1) * motorSpeed);
    }

    /**
     * start the intake running at full speed (INTAKE_SPEED)
     */
    public void start() {
        start(1);
    }


    /**
     * stop the intake
     */
    public void stop() {
        Util.consoleLog();

        motor1.stopMotor();
        motor2.stopMotor();

        isrunning = false;
        updateDS();
    }

    /**
     * update DriverStation status
     */
    private void updateDS()
    {
        SmartDashboard.putBoolean("Intake", isrunning);
    }
}