// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.btwrobotics.WhatTime.frc.DriverStation.MatchTimeManager;
import com.btwrobotics.WhatTime.frc.MotorManagers.MotorBulkActions;
import com.btwrobotics.WhatTime.frc.YearlyMethods.Rebuilt.RebuiltHubManager;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AdvantageKit.AdvantageKitConstants;


public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;

    private PowerDistribution pdp = new PowerDistribution();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

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
    public RebuiltHubManager rebuiltHubManager = new RebuiltHubManager(matchTimeManager);

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        // Configure logging for AdvantageKit
        Logger.recordMetadata("ProjectName", "1209Roomba");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (AdvantageKitConstants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logging
        Logger.start();

        currentAlliance = DriverStation.getAlliance();

        motorBulkActions.setNeutralModeBulk(Arrays.asList(
            robotContainer.shooterSubsystem.shooterPitchMotor,
            robotContainer.intakeSubsystem.angleMotor
        ), NeutralModeValue.Brake);

        // Reset motor speeds to zero.
        robotContainer.intakeSubsystem.angleMotor.set(0.0);
        robotContainer.shooterSubsystem.shooterPitchMotor.set(0.0);


        Logger.recordOutput("FieldInfo/CurrentAlliance", currentAlliance.toString());
    }

    @Override
    public void robotPeriodic() {
        pdp.clearStickyFaults();

        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

        matchTimeRemainingSeconds = DriverStation.getMatchTime();
        matchTimeElapsedSeconds = 160 - matchTimeRemainingSeconds;

        if (matchTimeElapsedSeconds - nextRumbleStartTime >= 0 && matchTimeElapsedSeconds - nextRumbleStartTime <= 2) {
            robotContainer.driverJoystick.joystick.setRumble(RumbleType.kBothRumble, 1.0);
            robotContainer.operatorJoystick.joystick.setRumble(RumbleType.kBothRumble, 1.0);
            robotContainer.debugJoystick.joystick.setRumble(RumbleType.kBothRumble, 1.0);
        }
        else {
            robotContainer.driverJoystick.joystick.setRumble(RumbleType.kBothRumble, 0.0);
            robotContainer.operatorJoystick.joystick.setRumble(RumbleType.kBothRumble, 0.0);
            robotContainer.debugJoystick.joystick.setRumble(RumbleType.kBothRumble, 0.0);
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
        Logger.recordOutput("RebuiltHubManager/IsActive", rebuiltHubManager.hubIsActive());
        Logger.recordOutput("RebuiltHubManager/InactiveFirst", rebuiltHubManager.getInactiveFirstAlliance().toString());
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

    Field2d robotField2d = new Field2d();
    
    Field2d limelight4Field2d = new Field2d();
    Field2d limelight2Field2d = new Field2d();

    public void updateNetworkTablesValues() {
        // MARK: use limelight to calculate this
        // double[] robotPose = NetworkTableInstance.getDefault().getTable("Pose").getEntry("robotPose").getDoubleArray(new double[]{0.0,0.0,0.0});
        // NetworkTableInstance.getDefault().getTable("CustomDashboard").getEntry("Pose").setDoubleArray(robotPose);

        Logger.recordOutput("MatchInfo/TimeRemaining", DriverStation.getMatchTime());

        //robotField2d.setRobotPose(robotContainer.drivetrain.getState().Pose);
        // NetworkTablesUtil.put("Main Robot Pose", robotField2d);
    }
}
