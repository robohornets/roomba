package frc.robot.subsystems.vision.limelight;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final StatusSignal<AngularVelocity> angularVelocityZ;


    private static final Matrix<N3, N1> VISION_STD_DEVS = 
        VecBuilder.fill(0.7, 0.7, 9999999);
    
    private static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;

    public LimelightSubsystem(
        CommandSwerveDrivetrain drivetrain, 
        String limelightName
    ) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.angularVelocityZ = drivetrain.getPigeon2().getAngularVelocityZWorld();
    }

    @Override
    public void periodic() {
        addOdometryMeasurement();
    }

    public void addOdometryMeasurement() {
        // Give limelight current estimated rotation from pose for MegaTag 2
        Pose2d currentPose = drivetrain.getState().Pose;

        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            currentPose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate.tagCount == 0) {
            return;
        }
        
        double angularVelDegPerSec = Math.abs(angularVelocityZ.refresh().getValueAsDouble());
        if (angularVelDegPerSec > MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return;
        }

        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VISION_STD_DEVS);
    }
}
