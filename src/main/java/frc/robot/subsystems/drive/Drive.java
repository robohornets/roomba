package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends SubsystemBase {
    public final CommandSwerveDrivetrain drivetrain;

    public Drive(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public static final double ODOMETRY_FREQUENCY = 250.0;
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // MARK: Field Centric Drive
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    // MARK: Robot Centric Drive
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void periodic() {
        drivetrain.periodic();

        Logger.recordOutput("SwerveDrive/Pose", getPose2d());
        
        // Log module states for AdvantageScope swerve visualization
        Logger.recordOutput("SwerveDrive/ModuleStates", drivetrain.getState().ModuleStates);
        Logger.recordOutput("SwerveDrive/ModuleTargets", drivetrain.getState().ModuleTargets);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds", drivetrain.getState().Speeds);
        Logger.recordOutput("SwerveDrive/Rotation", getPose2d().getRotation());
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return drivetrain.applyRequest(request);
    }

    public Pose2d getPose2d() {
        return drivetrain.getState().Pose;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        Logger.recordOutput("SwerveDrive/VisionMeasurement", visionRobotPoseMeters);
        Logger.recordOutput("SwerveDrive/VisionTimestamp", timestampSeconds);

        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3,N1> visionMeasurementStdDevs) {
        Logger.recordOutput("SwerveDrive/VisionMeasurement", visionRobotPoseMeters);
        Logger.recordOutput("SwerveDrive/VisionTimestamp", timestampSeconds);

        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public Pigeon2 getPigeon2() {
        return drivetrain.getPigeon2();
    }

    public void resetPose(Pose2d pose) {
        Logger.recordOutput("SwerveDrive/ResetPose", pose);

        drivetrain.resetPose(pose);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        drivetrain.registerTelemetry(telemetryFunction);
    }
}
