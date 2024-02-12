package Team4450.Robot24.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;

public class Candle extends SubsystemBase {
    private final CANdle candle = new CANdle(20);
    public Candle() {
        candle.setLEDs(0, 0, 255);
    }
}
