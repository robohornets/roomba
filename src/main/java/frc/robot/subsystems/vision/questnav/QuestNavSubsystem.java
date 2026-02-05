package frc.robot.subsystems.vision.questnav;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    CommandSwerveDrivetrain drivetrain;

    /** Local QuestNav instance used to read pose frames. */
    QuestNav questNav = new QuestNav();

    /**
     * Construct the QuestNavSubsystem.
     *
     * @param drivetrain the drivetrain subsystem that will consume vision measurements
     */
    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
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
     */
    @Override
    public void periodic() {
        // Gets most recent pose frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            // Checks to make sure the Quest was actually tracking the pose in the frame
            if (questFrame.isTracking()) {
                Pose3d questPose = questFrame.questPose3d();

                double timestamp = questFrame.dataTimestamp();

                // Transform questPose by Transform3d based on the location of the Quest mount
                Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

                NetworkTablesUtil.put("QuestNav Pose", robotPose.toPose2d());

                drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
            }
        }
        // Allow QuestNav library to progress internal state/commands
        questNav.commandPeriodic();
    }
}
