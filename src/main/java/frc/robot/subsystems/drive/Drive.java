package frc.robot.subsystems.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive extends SubsystemBase {
    public final CommandSwerveDrivetrain drivetrain;

    public Drive(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public static final double ODOMETRY_FREQUENCY = 250.0;

    @Override
    public void periodic() {
        drivetrain.periodic();

        Logger.recordOutput("Swerve Drive Pose", getPose2d());
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return drivetrain.applyRequest(request);
    }

    public Pose2d getPose2d() {
        return drivetrain.getState().Pose;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3,N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public Pigeon2 getPigeon2() {
        return drivetrain.getPigeon2();
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        drivetrain.registerTelemetry(telemetryFunction);
    }
}
