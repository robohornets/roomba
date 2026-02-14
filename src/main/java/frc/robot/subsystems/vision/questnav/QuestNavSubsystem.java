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
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

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

    /** Executor for async QuestNav calls. */
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    /** The currently running async task, or null if none. */
    private final AtomicReference<CompletableFuture<PoseFrame[]>> currentFuture = new AtomicReference<>(null);

    /** Counter for skipped cycles when QuestNav is busy. */
    private final AtomicInteger skipCount = new AtomicInteger(0);

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

        // Check if there's a previous call still running
        CompletableFuture<PoseFrame[]> future = currentFuture.get();

        if (future != null && !future.isDone()) {
            // Previous call still running - skip cycle
            int count = skipCount.incrementAndGet();
            Logger.recordOutput("QuestNav Skip Count", count);
            if (count % 50 == 0) {
                System.err.println("[QuestNav] WARNING: Previous frame fetch still running (skip #" + count + "). Skipping this cycle.");
            }
            return;
        }

        // Check if previous call completed with data
        PoseFrame[] questFrames = null;
        if (future != null && future.isDone()) {
            try {
                questFrames = future.getNow(null);
                currentFuture.set(null);
            } catch (CompletionException e) {
                System.err.println("[QuestNav] ERROR: Previous frame fetch failed: " + e.getMessage());
                currentFuture.set(null);
            }
        }

        // Process any frames we got from the previous call
        if (questFrames != null) {
            for (PoseFrame questFrame : questFrames) {
                System.out.println("[QuestNav] Processing frame, tracking: " + questFrame.isTracking());
                // Checks to make sure the Quest was tracking the pose in the frame
                if (questFrame.isTracking()) {
                    Pose3d questPose = questFrame.questPose3d();

                    double timestamp = questFrame.dataTimestamp();

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

        // Start a new async call
        CompletableFuture<PoseFrame[]> newFuture = CompletableFuture.supplyAsync(
            () -> questNav.getAllUnreadPoseFrames(),
            executor
        );
        currentFuture.set(newFuture);
    }

    public void setQuestPose(Pose3d pose3d) {
        questNav.setPose(pose3d.transformBy(QuestNavConstants.ROBOT_TO_QUEST));
        Logger.recordOutput("QuestNav Pose Set", pose3d);
    }
}
