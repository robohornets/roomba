package frc.robot.subsystems.mechanisms.shooter;

import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorGroup;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.math.MathSubsystem;



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

    // public ShooterConstants shooterConstants = new ShooterConstants();

    // MARK: Motors
    /** Motor controlling the shooter angle */
    public final Motor shooterPitchMotor = new Motor(11, "Mechanisms")
        .setFree(false)
        .setRange(ShooterConstants.SHOOTER_MIN_ANGLE, ShooterConstants.SHOOTER_MAX_ANGLE)
        .setMotorSpeed(0.4)
        .setHoldSpeed(0.0)
        .setPG(0.01)
        .setThreshold(ShooterConstants.POSITION_THRESHOLD)
        .setPositionSupplier(() -> getPigeonPosition());

    public final Motor leftShooterMotor = new Motor(13, "Mechanisms");
    public final Motor rightShooterMotor = new Motor(14, "Mechanisms", true);

    public final MotorGroup shooterMotors = new MotorGroup(Arrays.asList(leftShooterMotor, rightShooterMotor))
        .setMotorSpeed(0.4)
        .setAccelerationSteps(100);
    /** IMU sensor for shooter orientation feedback. */
    public final Pigeon2 shooterPigeon = new Pigeon2(34, "Mechanisms");

    // MARK: Constructor
    /**
     * Constructs the ShooterSubsystem.
     * @param drivetrain the swerve drivetrain subsystem (for aiming/coordination)
     */
    public ShooterSubsystem(Drive drivetrain) {
        this.drivetrain = drivetrain;

        shooterMotors.toggleEnabled(true);
        // shooterPitchMotor.toggleEnabled(true);

        for (ShooterDataPoint point : ShooterConstants.shooterDataPoints) {
            dataPoints.put(point.distance, point);
        }

        setDefaultCommand(
            Commands.run(() -> {
                if (pitchTarget != lastSentPitchTarget) {
                    shooterPitchMotor.goTo(pitchTarget);
                    lastSentPitchTarget = pitchTarget;
                }
                if (flywheelSpeed != lastSentFlywheelSpeed) {
                    shooterMotors.drive(flywheelSpeed);
                    lastSentFlywheelSpeed = flywheelSpeed;
                }
            }, this)
        );
    }

    public double shooterAngleTarget = 65;

    /** Shared pitch target used by manual joystick control and button bindings. */
    public double pitchTarget = ShooterConstants.SHOOTER_MAX_ANGLE;
    private double lastSentPitchTarget = Double.NaN;

    // MARK: Set Pitch Target
    public void setPitchTarget(double pitch) {
        // pitchTarget = MathUtil.clamp(pitch, ShooterConstants.SHOOTER_MIN_ANGLE, ShooterConstants.SHOOTER_MAX_ANGLE);
        shooterPitchMotor.goTo(pitchTarget / 180);
    }

    /** Shared flywheel speed target used by manual joystick control and button bindings. */
    public double flywheelSpeed = 0.0;
    private double lastSentFlywheelSpeed = Double.NaN;

    // MARK: Set Flywheel
    public void setFlywheelSpeed(double speed) {
        shooterMotors.drive(speed);
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        // if (drivetrain.isLockedToHub()) {
        //     double currentDistance = drivetrain.getDistanceToHub();
        //     ShooterDataPoint values = calculateShooterValues(shooterUpperLower(), currentDistance);
        //     setPitchTarget(values.angle);
        //     setFlywheelSpeed(values.speed);
        // }

        logValues();
    }

    // MARK: Get Pigeon
    public double getPigeonPosition() {
        return shooterPigeon.getRoll().refresh().getValueAsDouble() * -1;
    }

    // MARK: Increment Shooter
    /** For testing shooter angle manually */
    public void incrementShooterAngle(double incrementValue) {
        shooterAngleTarget += incrementValue;
        
        shooterPitchMotor.goTo(shooterAngleTarget);
    }



    public ShooterDataPoint shooterCalculateTrajectory() {
        double currentDistance = drivetrain.getDistanceToHub();
        double aimHeight = (6 - 20 / 12) / 3.281;

        double[] trajectory = (new MathSubsystem()).calculateTrajectoryFromExitAngle(currentDistance, aimHeight, 65);


        return new ShooterDataPoint(currentDistance, trajectory[1], trajectory[0]);
    }

    // MARK: UpperLowerPoint
    public UpperLowerPoint shooterUpperLower() {
        // MARK: NEEDS REFACTORING

        
        double currentDistance = drivetrain.getDistanceToHub();

        Map.Entry<Double, ShooterDataPoint> lowerEntry = dataPoints.floorEntry(currentDistance);
        Map.Entry<Double, ShooterDataPoint> upperEntry = dataPoints.ceilingEntry(currentDistance);

        // Handle out-of-range cases by clamping to the nearest point
        ShooterDataPoint lower = (lowerEntry != null) ? lowerEntry.getValue() : upperEntry.getValue();
        ShooterDataPoint upper = (upperEntry != null) ? upperEntry.getValue() : lowerEntry.getValue();

        return new UpperLowerPoint(upper, lower);
    }

    public ShooterDataPoint calculateShooterValues(UpperLowerPoint upperLowerPoint, double currentDistance) {
        double lowerDist = upperLowerPoint.getLowerDistance();
        double upperDist = upperLowerPoint.getUpperDistance();
        double valueRange = upperDist - lowerDist;

        if (valueRange == 0) {
            return new ShooterDataPoint(currentDistance, upperLowerPoint.getLowerAngle(), upperLowerPoint.getLowerSpeed());
        }

        double interpolationFactor = (currentDistance - lowerDist) / valueRange;

        double estimatedAngle = upperLowerPoint.getLowerAngle() + interpolationFactor * (upperLowerPoint.getUpperAngle() - upperLowerPoint.getLowerAngle());
        double estimatedSpeed = upperLowerPoint.getLowerSpeed() + interpolationFactor * (upperLowerPoint.getUpperSpeed() - upperLowerPoint.getLowerSpeed());

        return new ShooterDataPoint(currentDistance, estimatedAngle, estimatedSpeed);
    }

    public double getRequiredRPM(ShooterDataPoint shooterDataPoint){
        double vWheel = 2 * shooterDataPoint.speed;
        return vWheel * 60 / (Math.PI * 4 * 0.0254); // get rpm required for wheel with diameter of 4 inches
    }

    // MARK: Logging
    private void logValues() {
        Logger.recordOutput("ShooterSubsystem/MotorConnected", shooterPitchMotor.getMotor().isConnected());
        Logger.recordOutput("ShooterSubsystem/PigeonAngle", getPigeonPosition());
        Logger.recordOutput("ShooterSubsystem/PitchMotorOutput", shooterPitchMotor.getMotor().get());
        Logger.recordOutput("ShooterSubsystem/ShooterSpeed", flywheelSpeed);
        Logger.recordOutput("ShooterSubsystem/TargetAngle", shooterAngleTarget);
    }
}
