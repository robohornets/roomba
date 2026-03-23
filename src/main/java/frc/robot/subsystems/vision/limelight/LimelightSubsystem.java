package frc.robot.subsystems.vision.limelight;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.questnav.QuestNavConstants;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

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
    private Drive drivetrain;
    /** Local reference to the QuestNav system to add vision measurements */
    private QuestNavSubsystem questNavSubsystem;
    /** The configured Limelight instance name/key (NetworkTables entry name). */
    private final String limelightName;
    /** Cached signal that provides the robot's angular velocity around Z in world frame. */
    private final StatusSignal<AngularVelocity> angularVelocityZ;

    // MARK: Constructor
    /**
     * Create a LimelightSubsystem.
     *
     * @param drivetrain   drivetrain used to query current pose and add vision measurements
     * @param limelightName the NetworkTables name for the Limelight instance (e.g. "limelight")
     */
    public LimelightSubsystem(
        Drive drivetrain,
        QuestNavSubsystem questNavSubsystem,
        String limelightName
    ) {
        this.drivetrain = drivetrain;
        this.questNavSubsystem = questNavSubsystem;
        this.limelightName = limelightName;
        this.angularVelocityZ = drivetrain.getPigeon2().getAngularVelocityZWorld();
    }

    Field2d limelightField2d = new Field2d();

    private int totalLimelightEstimates = 0;
    private int estimatesAddedToQuest = 0;

    // MARK: Periodic Loop
    /**
     * Periodic update called by the scheduler. Adds a vision odometry measurement each cycle.
     * <p>Delegates to {@link #addOdometryMeasurement()} to perform the actual read/filter/submit
     * sequence.
     */
    @Override
    public void periodic() {
        addOdometryMeasurement();
    }

    // MARK: Add Odometry
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

        // Translate the pose by its offset from the centre of the robot
        Pose2d transformedPose = estimate.pose.transformBy(LimelightConstants.LIMELIGHT_4_TRANSFORM_FROM_CENTRE.inverse());
        Logger.recordOutput("Limelight/" + limelightName + "/Pose", transformedPose);

        Matrix<N3, N1> calculatedStdDevs = LimelightConstants.calculateDynamicStdDevs(estimate);

        double xDifference = Math.abs(transformedPose.getX() - drivetrain.getState().Pose.getX());
        double yDifference = Math.abs(transformedPose.getY() - drivetrain.getState().Pose.getY());

        double distanceError = Math.sqrt(xDifference * xDifference + yDifference * yDifference);

        if (distanceError >= 2.0) {
            return;
        }

        // Add measurement to drivetrain pose estimator
        drivetrain.addVisionMeasurement(transformedPose, estimate.timestampSeconds, calculatedStdDevs);

        totalLimelightEstimates++;
        Logger.recordOutput("Limelight/" + limelightName + "/TotalEstimates", totalLimelightEstimates);

        Logger.recordOutput("Limelight/" + limelightName + "/AcceptedPose", transformedPose);

        // Add measurement to QuestNav pose estimator if enabled
        if (QuestNavConstants.USE_LIMELIGHT_FOR_VISION_MEASUREMENTS) {
            estimatesAddedToQuest++;
            Logger.recordOutput("QuestNav/" + limelightName + "/LimelightEstimates", estimatesAddedToQuest);
            questNavSubsystem.addVisionMeasurement(transformedPose, estimate.timestampSeconds, LimelightConstants.calculateDynamicStdDevs(estimate), estimate);
        }
    }

    // MARK: Get Bot Pose
    public Pose2d getPose2d() {
        return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }

    // MARK: Logging
    public void logValues() {
    }
}
