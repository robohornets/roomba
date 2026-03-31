package frc.robot.subsystems.mechanisms.shooter;

import java.util.Arrays;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.MotorManagers.Motor;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorGroup;

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

    // MARK: Motors
    public final Motor leftShooterMotor = new Motor(13, "Mechanisms");
    public final Motor rightShooterMotor = new Motor(14, "Mechanisms", true);

    public final MotorGroup shooterMotors = new MotorGroup(Arrays.asList(leftShooterMotor, rightShooterMotor))
        .setMotorSpeed(0.4)
        .setAccelerationSteps(50);

    // MARK: Constructor
    /**
     * Constructs the ShooterSubsystem.
     * @param drivetrain the swerve drivetrain subsystem (for aiming/coordination)
     */
    public ShooterSubsystem(Drive drivetrain) {
        this.drivetrain = drivetrain;

        shooterMotors.toggleEnabled(true);

        for (ShooterDataPoint point : ShooterConstants.shooterDataPoints) {
            dataPoints.put(point.distance, point);
        }
    }

    public double shooterAngleTarget = 65;

    /** Shared flywheel speed target used by manual joystick control and button bindings. */
    public double flywheelSpeed = 0.0;

    public double lastSetFlywheelSpeed = 0.0;

    // MARK: Set Flywheel
    public void setFlywheelSpeed(double speed) {
        lastSetFlywheelSpeed = speed;
        shooterMotors.drive(speed);
    }

    // MARK: Periodic Loop
    @Override
    public void periodic() {
        logValues();
        logMotors();
    }

    // MARK: CanShoot
    public Boolean canShoot() {
        if (leftShooterMotor.getMotor().get() >= 0.1) {
            return false;
        }

        return true;
    }

    // MARK: IsShooting
    public Boolean isShooting() {
        return lastSetFlywheelSpeed > 0.0;
    }

    // MARK: CalculateTrajectory
    public ShooterDataPoint shooterCalculateTrajectory() {
        double currentDistance = drivetrain.getDistanceToHub();
        double aimHeight = (6 - 20 / 12) / 3.281;

        double[] trajectory = (new MathSubsystem()).calculateTrajectoryFromExitAngle(currentDistance, aimHeight, 65);

        return new ShooterDataPoint(currentDistance, trajectory[1], trajectory[0]);
    }

    // MARK: GetRequiredRPM
    public double getRequiredRPM(ShooterDataPoint shooterDataPoint){
        double vWheel = 2 * shooterDataPoint.speed;
         // get rpm required for wheel with diameter of 4 inches
        return vWheel * 60 / (Math.PI * 4 * 0.0254);
    }

    // MARK: Logging
    private void logValues() {
        Logger.recordOutput("ShooterSubsystem/ShooterSpeed", leftShooterMotor.getMotor().get());
        Logger.recordOutput("ShooterSubsystem/TargetAngle", shooterAngleTarget);
        Logger.recordOutput("ShooterSubsystem/CanShoot", canShoot() || isShooting());
        Logger.recordOutput("ShooterSubsystem/CanShootRaw", canShoot());
        Logger.recordOutput("ShooterSubsystem/IsShooting", isShooting());
    }

    // MARK: Log Motors
    private void logMotors() {
        logBasicMotorInformation(leftShooterMotor, "LeftShooterMotor");
        logBasicMotorInformation(rightShooterMotor, "RightShooterMotor");
    }

    private void logBasicMotorInformation(Motor motor, String name) {
        // Log connection
        Logger.recordOutput(
            "MotorStatus/ShooterSubsystem/MotorConnections/" + name, 
            motor.getMotor().isConnected()
        );

        // Log voltage
         Logger.recordOutput(
            "MotorStatus/ShooterSubsystem/Voltage/Output/" + name, 
            motor.getMotor().getMotorVoltage().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/ShooterSubsystem/Voltage/Supply/" + name, 
            motor.getMotor().getSupplyVoltage().getValueAsDouble()
        );

        // Log current
        Logger.recordOutput(
            "MotorStatus/ShooterSubsystem/Current/Stator/" + name, 
            motor.getMotor().getStatorCurrent().getValueAsDouble()
        );
        Logger.recordOutput(
            "MotorStatus/ShooterSubsystem/Current/Supply/" + name, 
            motor.getMotor().getSupplyCurrent().getValueAsDouble()
        );
    }
}
