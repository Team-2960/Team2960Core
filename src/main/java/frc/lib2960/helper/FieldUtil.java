package frc.lib2960.helper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtil {
    
    /**
     * Checks if the robot is currently on the red alliance. Defaults to blue if no alliance is currently available
     * @return  True if the the robot is currently on the red alliance. False if blue or alliance is not available
     */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
}