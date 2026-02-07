// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Optional;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;
import com.btwrobotics.WhatTime.frc.DriverStation.MatchTimeManager;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorBulkActions;
import com.btwrobotics.WhatTime.frc.YearlyMethods.Rebuilt.RebuiltHubManager;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Commands;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;

    private PowerDistribution pdp = new PowerDistribution();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = false;

    public double matchTimeRemainingSeconds = 160.0;
    public double matchTimeElapsedSeconds = 0.0;

    // A bool representing if the hub is inactive first or second
    public Optional<Boolean> inactiveFirst;
    public Optional<Alliance> firstInactiveAlliance;

    // The current alliance for the robot
    public Optional<Alliance> currentAlliance;

    // Manages rumble for Xbox controller
    // Start high so it doesn't trigger randomly
    public double nextRumbleStartTime = 1000;
    public MotorBulkActions motorBulkActions = new MotorBulkActions();
    


    // MARK: Hub Manager
    public MatchTimeManager matchTimeManager = new MatchTimeManager();
    // public RebuiltHubManager rebuiltHubManager = new RebuiltHubManager(matchTimeManager);

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        currentAlliance = DriverStation.getAlliance();

        motorBulkActions.setNeutralModeBulk(Arrays.asList(
            robotContainer.shooterSubsystem.shooterPitchMotor,
            robotContainer.shooterSubsystem.shooterMotor,
            robotContainer.climberSubsystem.climberLeft,
            robotContainer.climberSubsystem.climberRight
        ), NeutralModeValue.Brake);

        NetworkTablesUtil.put("Current Alliance", currentAlliance);
    }

    @Override
    public void robotPeriodic() {
        pdp.clearStickyFaults();

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (kUseLimelight) {
            var driveState = robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }

        matchTimeRemainingSeconds = DriverStation.getMatchTime();
        matchTimeElapsedSeconds = 160 - matchTimeRemainingSeconds;

        if (matchTimeElapsedSeconds - nextRumbleStartTime >= 0 && matchTimeElapsedSeconds - nextRumbleStartTime <= 2) {
            robotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 1.0);
            robotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, 1.0);
            robotContainer.debugJoystick.setRumble(RumbleType.kBothRumble, 1.0);
        }
        else {
            robotContainer.driverJoystick.setRumble(RumbleType.kBothRumble, 0.0);
            robotContainer.operatorJoystick.setRumble(RumbleType.kBothRumble, 0.0);
            robotContainer.debugJoystick.setRumble(RumbleType.kBothRumble, 0.0);
        }

        updateNetworkTablesValues();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        // NetworkTablesUtil.put("Hub is Active", rebuiltHubManager.hubIsActive());
        // NetworkTablesUtil.put("First Inactive Hub", rebuiltHubManager.getInactiveFirstAlliance());
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}


    public void updateNetworkTablesValues() {
        // MARK: use limelight to calculate this
        double[] robotPose = NetworkTableInstance.getDefault().getTable("Pose").getEntry("robotPose").getDoubleArray(new double[]{0.0,0.0,0.0});

        NetworkTableInstance.getDefault().getTable("CustomDashboard").getEntry("Pose").setDoubleArray(robotPose);
        NetworkTablesUtil.put("Time Remaining", DriverStation.getMatchTime());
        NetworkTablesUtil.put("Shooter Pitch", robotContainer.shooterSubsystem.getShooterMotorPitchDeg());
    }
}
