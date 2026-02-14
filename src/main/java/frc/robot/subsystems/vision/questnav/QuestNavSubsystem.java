package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.Logger;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;

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
    CommandSwerveDrivetrain drivetrain;

    /** Local QuestNav instance used to read pose frames. */
    QuestNav questNav;

    /** Executor for timeout-protected QuestNav calls. */
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    /** Maximum time in milliseconds to wait for QuestNav data before skipping. */
    private static final long QUESTNAV_TIMEOUT_MS = 5;

    /** Counter for timeouts. */
    private final AtomicInteger timeoutCount = new AtomicInteger(0);

    /**
     * Construct the QuestNavSubsystem.
     *
     * @param drivetrain the drivetrain subsystem that will consume vision measurements
     */
    public QuestNavSubsystem(
        CommandSwerveDrivetrain drivetrain
    ) {
        this.questNav = new QuestNav();
        this.drivetrain = drivetrain;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    Pose2d mostRecentPose2d = new Pose2d();
    Field2d questField2d = new Field2d();

    /**
     * Allows the QuestNav library to progress its internal state and commands.
     * This MUST be called BEFORE periodic() to populate the frame buffer.
     * Called from Robot.robotPeriodic() before CommandScheduler runs.
     */
    public void questPeriodicCommand() {
        questNav.commandPeriodic();
    }

    /**
     * Periodic update (called roughly every 20ms). Reads unread pose frames from QuestNav and
     * forwards valid frames to the drivetrain's odometry.
     *
     * <p>For each unread {@link PoseFrame}:
     * <ol>
     *   <li>skip frames where {@code isTracking()} is false;</li>
     *   <li>convert the Quest-reported pose into robot-centred coordinates using
     *       {@code QuestNavConstants.ROBOT_TO_QUEST.inverse()};</li>
     *   <li>publish a copy to NetworkTables under the key "QuestNav Pose" for debugging/visualisation;</li>
     *   <li>call {@code drivetrain.addVisionMeasurement(...)} with the transformed pose, the frame's
     *       timestamp, and the configured measurement standard deviations.</li>
     * </ol>
     *
     * <p>This method uses a timeout to prevent blocking the robot loop if QuestNav is slow to respond.
     * If the timeout is exceeded, the measurement is skipped to maintain robot responsiveness.
     */
    @Override
    public void periodic() {
        Logger.recordOutput("QuestNav Latency", questNav.getLatency());
        Logger.recordOutput("QuestNav is Tracking", questNav.isTracking());
        Logger.recordOutput("QuestNav Battery", questNav.getBatteryPercent().orElse(0));
        Logger.recordOutput("QuestNav Unread Pose Frames", questNav.getFrameCount().orElse(0));

        NetworkTablesUtil.put("QuestNav is Connected", questNav.isConnected());

        // Gets most recent pose frames from the Quest with timeout protection
        PoseFrame[] questFrames = null;
        try {
            CompletableFuture<PoseFrame[]> future = CompletableFuture.supplyAsync(
                () -> questNav.getAllUnreadPoseFrames(),
                executor
            );

            questFrames = future.get(QUESTNAV_TIMEOUT_MS, TimeUnit.MILLISECONDS);
        } catch (TimeoutException e) {
            int count = timeoutCount.incrementAndGet();
            System.err.println("[QuestNav] WARNING: Timed out waiting for pose frames (timeout #" + count + "). Skipping this cycle to prevent robot lag.");
            Logger.recordOutput("QuestNav Timeout Count", count);
            return;
        } catch (InterruptedException | ExecutionException e) {
            System.err.println("[QuestNav] ERROR: Exception while getting pose frames: " + e.getMessage());
            return;
        }

        if (questFrames == null) {
            return;
        }

        for (PoseFrame questFrame : questFrames) {
            System.out.println("[QuestNav] Processing frame, tracking: " + questFrame.isTracking());
            // Checks to make sure the Quest was actually tracking the pose in the frame
            if (questFrame.isTracking()) {
                Pose3d questPose = questFrame.questPose3d();

                double timestamp = questFrame.dataTimestamp();

                // Transform questPose by Transform3d based on the location of the Quest mount
                Pose3d transformedPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

                NetworkTable Table = NetworkTablesUtil.getTable("VisionSystems");
                //Table.getEntry("QuestNavPoseTest1").setValue(transformedPose);
                mostRecentPose2d = transformedPose.toPose2d();

                NetworkTablesUtil.put("QuestNavPose", transformedPose.toPose2d());

                // questField2d.setRobotPose(transformedPose.toPose2d());
                // NetworkTablesUtil.put("QuestNavFieldPose", questField2d);


                drivetrain.addVisionMeasurement(transformedPose.toPose2d(), timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
                System.out.println("[QuestNav] Added vision measurement: " + transformedPose.toPose2d());
            }
        }
    }

    public void setQuestPose(Pose3d pose3d) {
        questNav.setPose(pose3d.transformBy(QuestNavConstants.ROBOT_TO_QUEST));
        Logger.recordOutput("QuestNav Pose Set", pose3d);
    }
}
