package Team4450.Robot24.utility;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * AprilTag IDs for the Crescendo 2024 season.
 * 
 * Note that the BLUE/RED designations refer to which alliance
 * the tag "belongs" to rather than its physical location.
 */
public class AprilTagNames {
    public final int SOURCE_RIGHT;
    public final int SOURCE_LEFT;
    public final int AMP;
    public final int SPEAKER_MAIN;
    public final int SPEAKER_OFFSET;
    public final int TRAP_BACK;
    public final int TRAP_LEFT;
    public final int TRAP_RIGHT;

    public AprilTagNames(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            SOURCE_RIGHT = 1;
            SOURCE_LEFT = 2;
            AMP = 6;
            SPEAKER_MAIN = 7;
            SPEAKER_OFFSET = 8;
            TRAP_BACK = 14;
            TRAP_LEFT = 15;
            TRAP_RIGHT = 16;
        } else {
            SPEAKER_OFFSET = 3;
            SPEAKER_MAIN = 4;
            AMP = 5;
            SOURCE_RIGHT = 9;
            SOURCE_LEFT = 10;
            TRAP_LEFT = 11;
            TRAP_RIGHT = 12;
            TRAP_BACK = 13;
        }
    }
}
