package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public final class Utils {
    
    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

}
