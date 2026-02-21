package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Subsystem that integrates QuestNav pose frames into the drivetrain's odometry.
 *
 * <p>This subsystem polls the QuestNav device for unread pose frames, filters and transforms
 * each valid frame into the robot coordinate frame, publishes a copy to NetworkTables for
 * debugging, and forwards the measurement to the drivetrain using the configured
 * {@code QuestNavConstants.QUESTNAV_STD_DEVS} measurement noise.
 *
 * <p>Design notes:
 * <ul>
 *   <li>The Quest is treated as a vision/pose source; frames must be transformed by
 *       {@code QuestNavConstants.ROBOT_TO_QUEST.inverse()} to map the Quest's reported pose
 *       into the robot centre frame.</li>
 *   <li>This subsystem calls {@code questNav.commandPeriodic()} each scheduler tick to allow
 *       the QuestNav library to run its internal updates.</li>
 * </ul>
 */
public class QuestNavSubsystem extends SubsystemBase {
    /** Drivetrain used to query current state and to add vision measurements. */
    Drive drivetrain;

    /** Local QuestNav instance used to read pose frames. */
    QuestNav questNav;

    /** Pose estimator that fuses QuestNav, Limelight, and drivetrain odometry */
    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * Construct the QuestNavSubsystem.
     *
     * @param drivetrain the drivetrain subsystem that will consume vision measurements
     */
    public QuestNavSubsystem(
        Drive drivetrain
    ) {
        this.questNav = new QuestNav();
        this.drivetrain = drivetrain;

        // Initialize the pose estimator with drivetrain kinematics and initial pose
        this.poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.drivetrain.getKinematics(),
            drivetrain.getState().RawHeading,
            drivetrain.getState().ModulePositions,
            new Pose2d(),
            QuestNavConstants.ODOMETRY_STD_DEVS,
            QuestNavConstants.QUESTNAV_STD_DEVS
        );

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    Integer questEstimatesCounter = 0;

    /**
     * Allows the QuestNav library to progress its internal state and commands.
     * This MUST be called BEFORE periodic() to populate the frame buffer.
     * Called from Robot.robotPeriodic() before CommandScheduler runs.
     */
    public void questPeriodicCommand() {
        questNav.commandPeriodic();
    }

    /**
     * Periodic update (called roughly every 20ms). Updates the pose estimator with odometry
     * and reads unread pose frames from QuestNav.
     *
     * <p>For each unread {@link PoseFrame}:
     * <ol>
     *   <li>skip frames where {@code isTracking()} is false;</li>
     *   <li>convert the Quest-reported pose into robot-centred coordinates using
     *       {@code QuestNavConstants.ROBOT_TO_QUEST.inverse()};</li>
     *   <li>publish a copy to NetworkTables under the key "QuestNav Pose" for debugging/visualisation;</li>
     *   <li>add the measurement to both the drivetrain and the local pose estimator.</li>
     * </ol>
     */
    @Override
    public void periodic() {
        questPeriodicCommand();

        // Update pose estimator with latest odometry from drivetrain
        poseEstimator.update(
            drivetrain.getState().RawHeading,
            drivetrain.getState().ModulePositions
        );

        Logger.recordOutput("QuestNav/Latency", getLatency());
        Logger.recordOutput("QuestNav/Connected", questIsConnected());
        Logger.recordOutput("QuestNav/Battery", getBatteryPercentage());
        Logger.recordOutput("QuestNav/EstimatedPose", getEstimatedPose());

        // Gets most recent pose frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            // Checks to make sure the Quest was tracking the pose in the frame
            if (questFrame.isTracking()) {
                Pose3d questPose = questFrame.questPose3d();

                double timestamp = questFrame.dataTimestamp();

                // Transform questPose by Transform3d based on the location of the Quest mount
                Pose3d transformedPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

                // Log pose with AdvantageKit and put to NetworkTables
                Logger.recordOutput("QuestNav/Pose", transformedPose.toPose2d());

                // Add to both drivetrain and local pose estimator
                drivetrain.addVisionMeasurement(transformedPose.toPose2d(), timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
                addVisionMeasurement(transformedPose.toPose2d(), timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
                incrementPoseCounter();
            }
        }
    }

    /**
     * Adds a vision measurement to the local pose estimator.
     * This should be called by the LimelightSubsystem to add Limelight measurements.
     *
     * @param visionPose the pose measured by the vision system
     * @param timestamp the timestamp of the measurement in seconds
     * @param stdDevs the standard deviations for the measurement [x, y, theta]
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    /**
     * Gets the current estimated pose from the pose estimator.
     * This fuses QuestNav, Limelight, and drivetrain odometry.
     *
     * @return the current estimated pose
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the QuestNav pose and the local pose estimator.
     *
     * @param pose3d the new pose to set
     */
    public void setQuestPose(Pose3d pose3d) {
        questNav.setPose(pose3d.transformBy(QuestNavConstants.ROBOT_TO_QUEST));

        // Also reset the pose estimator
        poseEstimator.resetPosition(
            drivetrain.getState().RawHeading,
            drivetrain.getState().ModulePositions,
            pose3d.toPose2d()
        );
    }

    public boolean questIsConnected() {
        return questNav.isConnected();
    }

    public void incrementPoseCounter() {
        questEstimatesCounter++;
        Logger.recordOutput("QuestNav/EstimateCount", questEstimatesCounter);
    }

    public double getLatency() {
        return questNav.getLatency();
    }

    public int getBatteryPercentage() {
        return questNav.getBatteryPercent().orElse(0);
    }
}