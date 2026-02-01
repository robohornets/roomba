package frc.robot.subsystems.vision.questnav;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    
    QuestNav questNav = new QuestNav();

    // Runs every 20ms to update robot pose from Quest
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
        questNav.commandPeriodic();
    }
}
