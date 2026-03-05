package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;

public class ShooterConstants {
    public static List<ShooterDataPoint> shooterDataPoints = List.of(
        // TODO: Collect successful data points and add them here
        new ShooterDataPoint(0, 0, 0)
    );

    // --- Shooter configuration and tuning fields ---

    /** Entry angle to the hub in degrees (TODO: calculate actual value). */
    // public double hubEnterAngle = -70;

    /** Speed for pitching the shooter (open-loop, 0..1). */
    // public double shooterPitchSpeed = 0.1;
    /** Hold speed for maintaining shooter pitch (open-loop, 0..1). */
    public double shooterPitchHoldSpeed = 0.0;

    /** Maximum allowed shooter pitch (units depend on mechanism, e.g., rotations or percent). */
    public double shooterPitchMax = 0.6;
    /** Minimum allowed shooter pitch. */
    public double shooterPitchMin = 0.0;

    /** Threshold for position manager to consider the shooter "at position". */
    public double positionThreshold = 0.02;

    /** Height of the hub (target) in meters. */
    // public double hubHeight = 2;
    /** Height of the shooter in meters. */
    // public double shooterHeight = 1;
}
