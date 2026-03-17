package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;

public class ShooterConstants {
    public static final List<ShooterDataPoint> shooterDataPoints = List.of(
        // TODO: Collect successful data points and add them here
        new ShooterDataPoint(0.0, 0, 0),
        new ShooterDataPoint(0.5, 0, 0),
        new ShooterDataPoint(1.0, 0, 0),
        new ShooterDataPoint(1.5, 0, 0),
        new ShooterDataPoint(2.0, 0, 0),
        new ShooterDataPoint(2.5, 0, 0),
        new ShooterDataPoint(3.0, 0, 0),
        new ShooterDataPoint(3.5, 0, 0),
        new ShooterDataPoint(4.0, 0, 0),
        new ShooterDataPoint(4.5, 0, 0),
        new ShooterDataPoint(5.0, 0, 0),
        new ShooterDataPoint(5.5, 0, 0),
        new ShooterDataPoint(6.0, 0, 0)
    );

    // --- Shooter configuration and tuning fields ---

    /** Entry angle to the hub in degrees (TODO: calculate actual value). */
    // public double hubEnterAngle = -70;

    /** Speed for pitching the shooter (open-loop, 0..1). */
    // public double shooterPitchSpeed = 0.1;
    /** Hold speed for maintaining shooter pitch (open-loop, 0..1). */
    public static final double SHOOTER_PITCH_HOLD_SPEED = 0.0;

    /** Maximum allowed shooter pitch (units depend on mechanism, e.g., rotations or percent). */
    public static final double SHOOTER_PITCH_MAX_SPEED = 0.6;
    /** Minimum allowed shooter pitch. */
    public static final double SHOOTER_PITCH_MIN_SPEED = 0.0;

    /** Threshold for position manager to consider the shooter "at position". */
    public static final double POSITION_THRESHOLD = 0.02;

    public static final double FEEDER_IN_SPEED = 0.2;
    public static final double FEEDER_OUT_SPEED = -0.05;
    



    /** Height of the hub (target) in meters. */
    public static final double HUB_HEIGHT_METRES = 6 / 3.281; // 6 feet to meters
    /** Aim above hub in meters */
    public static final double AIM_ABOVE = 1 / 3.281; // 1 foot to meters
    /** Height of the shooter in meters. */
    public static final double SHOOTER_HEIGHT = 20 / 12 / 3.281; // 26 inches to meters
}
