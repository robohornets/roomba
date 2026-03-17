package frc.robot.subsystems.mechanisms.shooter;

import java.util.Arrays;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorGroup;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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

    public ShooterConstants shooterConstants = new ShooterConstants();

    // MARK: Motors
    /** Motor controlling the shooter pitch (angle). */
    // this is reversed in real life so its inverted in the code
    public final Motor shooterPitchMotor = new Motor(11, "Mechanisms", true)
        .setFree(false)
        .setRange(-0.5, 0)
        .setMotorSpeed(0.1);

    public final Motor leftShooterMotor = new Motor(13, "Mechanisms");
    public final Motor rightShooterMotor = new Motor(14, "Mechanisms", true);

    public final MotorGroup shooterMotors = new MotorGroup(
        Arrays.asList(leftShooterMotor, rightShooterMotor)
    ).setMotorSpeed(0.4);

    // MARK: Constructor
    /**
     * Constructs the ShooterSubsystem.
     * @param drivetrain the swerve drivetrain subsystem (for aiming/coordination)
     */
    public ShooterSubsystem(Drive drivetrain) {
        this.drivetrain = drivetrain;

        shooterPitchMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        leftShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        rightShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration);
        
        for (ShooterDataPoint point : ShooterConstants.shooterDataPoints) {
            dataPoints.put(point.distance, point);
        }
    }

    /** IMU sensor for shooter orientation feedback. */
    // public final Pigeon2 shooterPigeon = new Pigeon2(34);

    public final CANcoder shooterThroughBore = new CANcoder(36);

    @Override
    public void periodic() {
        Logger.recordOutput("ShooterSubsystem/ThroughBoreAngle", getThroughBorePosition());
    }

    // --- Commands --- \\

    // MARK: Get Through Bore
    public double getThroughBorePosition() {
        double offset = 0.4;
        return shooterThroughBore.getAbsolutePosition().getValueAsDouble() + offset;
    }

    // For testing shooter angle manually
    private double shooterAngleTargetTesting = 0.0;
    public void incrementShooterAngle(double incrementValue) {
        shooterAngleTargetTesting += incrementValue;
        
        Logger.recordOutput("ShooterSubsystem/TestingAngleTarget", shooterAngleTargetTesting);
        
        shooterPitchMotor.goTo(shooterAngleTargetTesting);
    }

    public UpperLowerPoint shooterUpperLower() {
        // MARK: NEEDS REFACTORING
        // if (dataPoints.isEmpty()) {

            double currentDistance = drivetrain.getDistanceToHub();
            double aimHeight = (6 - 20 / 12) / 3.281;

            double[] trajectory = (new MathSubsystem()).calculateTrajectoryFromExitAngle(currentDistance, aimHeight, 70);


            return new UpperLowerPoint(
                new ShooterDataPoint(currentDistance, trajectory[1], trajectory[0]),
                new ShooterDataPoint(currentDistance, trajectory[1], trajectory[0])
            );
        // }
        // double currentDistance = drivetrain.getDistanceToHub();


        // Map.Entry<Double, ShooterDataPoint> lowerEntry = dataPoints.floorEntry(currentDistance);
        // Map.Entry<Double, ShooterDataPoint> upperEntry = dataPoints.ceilingEntry(currentDistance);

        // // Handle out-of-range cases by clamping to the nearest point
        // ShooterDataPoint lower = (lowerEntry != null) ? lowerEntry.getValue() : upperEntry.getValue();
        // ShooterDataPoint upper = (upperEntry != null) ? upperEntry.getValue() : lowerEntry.getValue();

        // return new UpperLowerPoint(upper, lower);
    }

    public ShooterDataPoint calculateShooterValues(UpperLowerPoint upperLowerPoint, double currentDistance) {
        // double valueRange = Math.abs(upperLowerPoint.getUpperDistance() - upperLowerPoint.getLowerDistance());
        // if (valueRange == 0) {
            return new ShooterDataPoint(currentDistance, upperLowerPoint.getUpperAngle(), upperLowerPoint.getUpperSpeed());
        // }

        // double scaledValue = currentDistance - Math.min(upperLowerPoint.getUpperDistance(), upperLowerPoint.getLowerDistance());

        // double interpolationFactor = scaledValue/valueRange;

        // // Interpolate the angle and speed between the data points
        // double estimatedAngle = upperLowerPoint.getLowerAngle() + interpolationFactor * (upperLowerPoint.getUpperAngle() - upperLowerPoint.getLowerAngle());
        // double estimatedSpeed = upperLowerPoint.getLowerSpeed() + interpolationFactor * (upperLowerPoint.getUpperSpeed() - upperLowerPoint.getLowerSpeed());
        
        // return new ShooterDataPoint(currentDistance, estimatedAngle, estimatedSpeed);
    }

    public double getRequiredRPM(ShooterDataPoint shooterDataPoint){
        double vWheel = 2 * shooterDataPoint.speed;
        return vWheel * 60 / (Math.PI * 4 * 0.0254); // get rpm required for wheel with diameter of 4 inches
    }
}
