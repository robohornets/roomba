package frc.robot.subsystems.mechanisms.shooter;

import java.util.List;

import com.btwrobotics.WhatTime.frc.MotorManagers.MotorWrapper;
import com.btwrobotics.WhatTime.frc.MotorManagers.PositionManager;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    CommandSwerveDrivetrain drivetrain;

    /**
     * Constructs the ShooterSubsystem.
     * @param drivetrain the swerve drivetrain subsystem (for aiming/coordination)
     */
    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /** Utility for motion calculations (e.g., trajectory, angles). */
    MotorSubsystem motorSubsystem = new MotorSubsystem();
    
    /** Motor controlling the shooter flywheel. */
    public final MotorWrapper shooterMotor = new MotorWrapper(
        new TalonFX(10),
        false
    );
    /** Motor controlling the shooter pitch (angle). */
    public final MotorWrapper shooterPitchMotor = new MotorWrapper(
        new TalonFX(9),
        false
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
        positionThreshold, 
        () -> getShooterPitchDeg()
    );

    // --- Commands ---

    /**
     * Returns a command to pitch the shooter to the specified angle (degrees).
     * @param angle target pitch angle in degrees
     * @return a command that moves the shooter pitch to the given angle
     */
    public Command pitchToAngleDeg(double angle) {
        return shooterPositionManager.move(angle);
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
                // Get robot position (Should be implemented into WhatTime)
                // Calculate distance to hub (Should be implemented into WhatTime)

                // Calculate required rotations to face hub
                // Rotate to face hub

                // Calculate required shooter pitch angle to hit hub
                // Pitch shooter to required angle
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
        // MARK: Should be roll?
        return shooterPigeon.getRoll().getValueAsDouble();
    }

}
