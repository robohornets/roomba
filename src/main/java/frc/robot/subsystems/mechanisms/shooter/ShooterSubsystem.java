package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import com.btwrobotics.WhatTime.frc.FlywheelPair;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.MotorManagers.PositionManager;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.motor.MotorSubsystem;



/**
 * ShooterSubsystem manages the shooter mechanism, including pitch and firing motors, and provides
 * commands for aiming and controlling the shooter. It integrates with the drivetrain and uses
 * sensor feedback for precise control.
 *
 * Key features:
 * <ul>
 *   <li>Controls shooter flywheel and pitch motors via MotorWrapper.</li>
 *   <li>Uses a Pigeon2 IMU for shooter orientation feedback.</li>
 *   <li>Provides commands for aiming and pitching to a specific angle.</li>
 *   <li>Exposes configuration fields for shooter geometry and tuning.</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {
    /** Reference to the drivetrain for coordinated aiming. */
    Drive drivetrain;

    TreeMap<Double, ShooterDataPoint> dataPoints = new TreeMap<>();

    List<ShooterDataPoint> shooterDataPoints = List.of(
        // TODO: Collect successful data points and add them here
        new ShooterDataPoint(0, 0, 0)
    );

    /**
     * Constructs the ShooterSubsystem.
     * @param drivetrain the swerve drivetrain subsystem (for aiming/coordination)
     */
    public ShooterSubsystem(Drive drivetrain) {
        this.drivetrain = drivetrain;
        
        for (ShooterDataPoint point : shooterDataPoints) {
            dataPoints.put(point.distance, point);
        }
    }

    /** Utility for motion calculations (e.g., trajectory, angles). */
    MotorSubsystem motorSubsystem = new MotorSubsystem();
    
    /** Motor controlling the shooter pitch (angle). */
    public final MotorWrapper shooterPitchMotor = new MotorWrapper(
        new TalonFX(11),
        false
    );

    public FlywheelPair shooterMotors = new FlywheelPair(
        new MotorWrapper(new TalonFX(12), false), // Left shooter motor
        new MotorWrapper(new TalonFX(13), true), // Right shooter motor
        0.4
    );

    /** IMU sensor for shooter orientation feedback. */
    public final Pigeon2 shooterPigeon = new Pigeon2(34);

    // --- Shooter configuration and tuning fields ---

    /** Entry angle to the hub in degrees (TODO: calculate actual value). */
    public double hubEnterAngle = -70;

    /** Speed for pitching the shooter (open-loop, 0..1). */
    public double shooterPitchSpeed = 0.1;
    /** Hold speed for maintaining shooter pitch (open-loop, 0..1). */
    public double shooterPitchHoldSpeed = 0.02;

    /** Maximum allowed shooter pitch (units depend on mechanism, e.g., rotations or percent). */
    public double shooterPitchMax = 0.3;
    /** Minimum allowed shooter pitch. */
    public double shooterPitchMin = 0.0;

    /** Threshold for position manager to consider the shooter "at position". */
    public double positionThreshold = 1.0;

    /** Height of the hub (target) in meters. */
    public double hubHeight = 2;
    /** Height of the shooter in meters. */
    public double shooterHeight = 1;

    /**
     * PositionManager for controlling the shooter pitch motor to a target angle.
     * Uses feedback from the shooter IMU.
     */
    public PositionManager shooterPositionManager = new PositionManager(
        shooterPitchMin,
        shooterPitchMax,
        List.of(shooterPitchMotor),
        0.2,
        0.0,
        0.05,
        positionThreshold, 
        () -> getShooterPitchDeg()
    );

    // --- Commands ---

    /**
     * Returns a command to pitch the shooter to the specified angle (degrees).
     * @param angle target pitch angle in degrees
     */
    public void pitchToAngleDeg(double angle) {
        shooterPositionManager.setTarget(angle);
    }
    
    /**
     * Returns a command to aim the shooter at the hub.
     * Currently a placeholder: should calculate robot position, distance to hub,
     * required rotation, and pitch, then command the shooter and drivetrain.
     * @return a command that aims the shooter at the hub
     */
    public Command aimAtHub() {
        return Commands.run(
            () -> {
                // Calculate the angle and speed needed
                ShooterDataPoint hubCalculateDataPoint = calculateShooterValues(
                    shooterUpperLower(), 
                    drivetrain.getDistanceToHub()
                );

                pitchToAngleDeg(hubCalculateDataPoint.angle);
            }
        );
    }

    // --- Sensor feedback ---

    /**
     * Gets the shooter pitch motor's position in degrees.
     * @return shooter pitch (motor) position in degrees
     */
    public double getShooterMotorPitchDeg() {
        return shooterPitchMotor.getPosition() * 360;
    }

    /**
     * Gets the current shooter pitch in degrees from the Pigeon2 IMU.
     * @return shooter pitch in degrees (roll axis)
     */
    public double getShooterPitchDeg() {
        return shooterPigeon.getRoll().getValueAsDouble();
    }

    public UpperLowerPoint shooterUpperLower() {
        double currentDistance = drivetrain.getDistanceToHub();
        Map.Entry<Double, ShooterDataPoint> lowerEntry = dataPoints.floorEntry(currentDistance);
        Map.Entry<Double, ShooterDataPoint> upperEntry = dataPoints.ceilingEntry(currentDistance);

        // Handle out-of-range cases by clamping to the nearest point
        ShooterDataPoint lower = (lowerEntry != null) ? lowerEntry.getValue() : upperEntry.getValue();
        ShooterDataPoint upper = (upperEntry != null) ? upperEntry.getValue() : lowerEntry.getValue();

        return new UpperLowerPoint(upper, lower);
    }

    public ShooterDataPoint calculateShooterValues(UpperLowerPoint upperLowerPoint, double currentDistance) {
        double valueRange = Math.abs(upperLowerPoint.getUpperDistance() - upperLowerPoint.getLowerDistance());
        double scaledValue = currentDistance - Math.min(upperLowerPoint.getUpperDistance(), upperLowerPoint.getLowerDistance());

        double interpolationFactor = scaledValue/valueRange;

        // Interpolate the angle and speed between the data points
        double estimatedAngle = upperLowerPoint.getLowerAngle() + interpolationFactor * (upperLowerPoint.getUpperAngle() - upperLowerPoint.getLowerAngle());
        double estimatedSpeed = upperLowerPoint.getLowerSpeed() + interpolationFactor * (upperLowerPoint.getUpperSpeed() - upperLowerPoint.getLowerSpeed());
        
        return new ShooterDataPoint(currentDistance, estimatedAngle, estimatedSpeed);
    }
}
