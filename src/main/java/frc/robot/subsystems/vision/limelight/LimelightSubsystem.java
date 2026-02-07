package frc.robot.subsystems.vision.limelight;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final StatusSignal<AngularVelocity> angularVelocityZ;

    public LimelightSubsystem(
        CommandSwerveDrivetrain drivetrain, 
        String limelightName
    ) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.angularVelocityZ = drivetrain.getPigeon2().getAngularVelocityZWorld();
    }

    Pose2d mostRecentPose2d = new Pose2d();

    @Override
    public void periodic() {
        addOdometryMeasurement();
    }

    public void addOdometryMeasurement() {
        // Give limelight current estimated rotation from pose for MegaTag 2
        double currentYaw = drivetrain.getState().Pose.getRotation().getDegrees();

        // Set the current yaw of the robot for increased accuracy
        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            currentYaw,
            0, 0, 0, 0, 0
        );

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null || estimate.tagCount == 0) {
            return;
        }
        
        // Discard result if the angular velocity is too high
        double angularVelDegPerSec = Math.abs(angularVelocityZ.refresh().getValueAsDouble());
        if (angularVelDegPerSec > LimelightConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return;
        }

        // Translate the pose by its offset from the centre of the robot
        Pose2d transformedPose = estimate.pose.transformBy(LimelightConstants.getTransformForLimelight(limelightName).inverse());

        mostRecentPose2d = transformedPose;
        NetworkTablesUtil.put("Vision Systems", limelightName + "Pose", transformedPose);

        drivetrain.addVisionMeasurement(transformedPose, estimate.timestampSeconds, LimelightConstants.VISION_STD_DEVS);
    }

    public Pose2d getMostRecentPose2d() {
        return mostRecentPose2d;
    }
}
