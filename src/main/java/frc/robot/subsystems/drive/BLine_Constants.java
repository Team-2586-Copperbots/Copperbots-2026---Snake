package frc.robot.subsystems.drive;

import frc.robot.lib.BLine.Path.PathConstraints;

public final class BLine_Constants {
    public static final double tkP = 10;
    public static final double tkI = 0;
    public static final double tkD = 1.55;

    public static final double rkP = 3;
    public static final double rkI = 0;
    public static final double rkD = 0;

    public static final double CTkP = 3;
    public static final double CTkI = 0;
    public static final double CTkD = 0;

    public static final double highToleranceRot = 0.5;
    public static final double highToleranceTranslation = 0.02;
    public static final PathConstraints highTolerance = new PathConstraints()
            .setEndRotationToleranceDeg(highToleranceRot).setEndTranslationToleranceMeters(highToleranceTranslation);
}
