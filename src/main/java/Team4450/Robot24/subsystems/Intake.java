package Team4450.Robot24.subsystems;

import static Team4450.Robot24.Constants.INTAKE_MOTOR_1;
import static Team4450.Robot24.Constants.INTAKE_MOTOR_2;
import static Team4450.Robot24.Constants.INTAKE_SPEED;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax motor1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushless);

    private double  motorSpeed = INTAKE_SPEED;
    private boolean isrunning = false;

    public Intake() {
        motor2.follow(motor1);

        // note: we should probably use the `subsystem_property` format in
        // NetworkTables because of how AdvantageScope parses it. - Cole W.
        SmartDashboard.putNumber("intake_speed", motorSpeed);

        updateDS();
        
        Util.consoleLog("Intake created!");
    }

    @Override
    public void periodic() {
        motorSpeed = SmartDashboard.getNumber("intake_speed", motorSpeed);

        if (isrunning) motor1.set(motorSpeed);
    }

    public void start() {
        Util.consoleLog();

        isrunning = true;

        motor1.set(motorSpeed);

        updateDS();
    }

    public void stop() {
        Util.consoleLog();

        isrunning = false;

        motor1.stopMotor();

        updateDS();
    }

    private void updateDS()
    {
        SmartDashboard.putBoolean("Intake", isrunning);
    }
}