package frc.robot.subsystems.vision.limelight;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;

/**
 * Subsystem that integrates a Limelight camera with the drivetrain's odometry.
 *
 * <p>This class periodically reads pose estimates from a configured Limelight
 * instance (using preset tag/field logic), filters them, and submits valid
 * vision-based pose measurements to the drivetrain's odometry system.
 *
 * <p>High-level behaviour of {@link #addOdometryMeasurement()}:
 * <ol>
 *   <li>Provide the Limelight with the current robot yaw to improve its pose estimates.</li>
 *   <li>Request a pose estimate for a known tag (MegaTag 2 in this project).</li>
 *   <li>Ignore null/empty results or estimates captured while the robot is rotating too quickly.</li>
 *   <li>Publish the estimate to NetworkTables for debugging/visualisation.
 *   <li>Translate the estimated pose from the Limelight frame to the robot centre frame
 *       using {@code LIMELIGHT_TRANSFORM_FROM_CENTRE} and then hand it to the drivetrain
 *       via {@code drivetrain.addVisionMeasurement(...)} with configured standard deviations.
 * </ol>
 *
 * <p>Notes:
 * <ul>
 *   <li>The subsystem expects {@link LimelightHelpers} and constants in {@code LimelightConstants}
 *       to be configured for this robot (tag IDs, transforms, and acceptable angular velocity).
 *   <li>Measurements taken while angular velocity exceeds
 *       {@code LimelightConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC} are discarded to avoid
 *       corrupting odometry with motion-blurred or otherwise invalid estimates.
 * </ul>
 */
public class LimelightSubsystem extends SubsystemBase {
    /** Local reference to the drivetrain used for pose/state and adding vision measurements. */
    private CommandSwerveDrivetrain drivetrain;
    /** The configured Limelight instance name/key (NetworkTables entry name). */
    private final String limelightName;
    /** Cached signal that provides the robot's angular velocity around Z in world frame. */
    private final StatusSignal<AngularVelocity> angularVelocityZ;

    /**
     * Create a LimelightSubsystem.
     *
     * @param drivetrain   drivetrain used to query current pose and add vision measurements
     * @param limelightName the NetworkTables name for the Limelight instance (e.g. "limelight")
     */
    public LimelightSubsystem(
        CommandSwerveDrivetrain drivetrain,
        String limelightName
    ) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        this.angularVelocityZ = drivetrain.getPigeon2().getAngularVelocityZWorld();
    }

    /**
     * Periodic update called by the scheduler. Adds a vision odometry measurement each cycle.
     * <p>Delegates to {@link #addOdometryMeasurement()} to perform the actual read/filter/submit
     * sequence.
     */
    @Override
    public void periodic() {
        addOdometryMeasurement();
    }

    /**
     * Reads a pose estimate from the Limelight and, when valid, submits it to the drivetrain's
     * odometry.
     *
     * <p>Behavior and side-effects:
     * <ul>
     *   <li>Writes the raw pose returned by the Limelight to NetworkTables under "Limelight Pose".
     *   <li>Filters out null/empty estimates and estimates taken while the robot's angular velocity
     *       is above {@code LimelightConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC}.
     *   <li>Transforms the Limelight pose into the robot-centred frame using
     *       {@code LimelightConstants.LIMELIGHT_TRANSFORM_FROM_CENTRE.inverse()} and calls
     *     {@code drivetrain.addVisionMeasurement(transformedPose, timestamp, VISION_STD_DEVS)}.
     * </ul>
     *
     * <p>Implementation details:
     * <ul>
     *   <li>The current robot yaw is pulled from {@code drivetrain.getState().Pose.getRotation()} and
     *       provided to the Limelight via {@code LimelightHelpers.SetRobotOrientation(...)} so the
     *       camera can use the best available heading when generating pose estimates.
     *   <li>The specific pose method used is {@code LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(...)}
     *       — change this if a different tag or pipeline is required.
     * </ul>
     */
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

        NetworkTablesUtil.put("Limelight Pose", estimate.pose);

        // Translate the pose by its offset from the centre of the robot
        Pose2d transformedPose = estimate.pose.transformBy(LimelightConstants.LIMELIGHT_TRANSFORM_FROM_CENTRE.inverse());

        drivetrain.addVisionMeasurement(transformedPose, estimate.timestampSeconds, LimelightConstants.VISION_STD_DEVS);
    }
}
