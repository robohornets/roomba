// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.btwrobotics.WhatTime.frc.DriverStation.MatchTimeManager;
import com.btwrobotics.WhatTime.frc.YearlyMethods.Rebuilt.RebuiltHubManager;
import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // MARK: Hub Manager
    public MatchTimeManager matchTimeManager = new MatchTimeManager();
    public RebuiltHubManager rebuiltHubManager = new RebuiltHubManager(matchTimeManager);

    public Robot() {
        robotContainer = new RobotContainer();
    }

    // MARK: Robot Init
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

        DriverStation.silenceJoystickConnectionWarning(true);

        // Enable motors (WhatTime Motor.isEnabled defaults to false — drive/goTo do nothing until this is called)
        robotContainer.shooterSubsystem.shooterPitchMotor.toggleEnabled(true);
        robotContainer.shooterSubsystem.leftShooterMotor.toggleEnabled(true);
        robotContainer.shooterSubsystem.rightShooterMotor.toggleEnabled(true);
        // robotContainer.intakeSubsystem.angleMotor.toggleEnabled(true);
        robotContainer.intakeSubsystem.intakeWheelsMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.feederBedMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.feederFeederMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.shooterFeederMotor.toggleEnabled(true);

        // Apply full motor config here (after CAN bus is stable — constructor-time apply() silently fails)
        // StatusCodes are logged so you can verify success in AdvantageScope under MotorConfig/
        Logger.recordOutput("MotorConfig/ShooterPitch",    robotContainer.shooterSubsystem.shooterPitchMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/LeftShooter",     robotContainer.shooterSubsystem.leftShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/RightShooter",    robotContainer.shooterSubsystem.rightShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        // Logger.recordOutput("MotorConfig/IntakeAngle",     robotContainer.intakeSubsystem.angleMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/IntakeWheels",    robotContainer.intakeSubsystem.intakeWheelsMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/FeederBed",       robotContainer.feederSubsystem.feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/FeederFeeder",    robotContainer.feederSubsystem.feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/ShooterFeeder",   robotContainer.feederSubsystem.shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
    }

    // MARK: Robot Periodic
    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

        logDriveStationValues();

        // if (!currentAlliance.equals(DriverStation.getAlliance())) {
        //     currentAlliance = DriverStation.getAlliance();
        //     Logger.recordOutput("FieldInfo/CurrentAlliance", currentAlliance.toString());
        // }
    }

    // MARK: Disabled Init
    @Override
    public void disabledInit() {
    }

    // MARK: Disabled Periodic
    @Override
    public void disabledPeriodic() {
        pdp.clearStickyFaults();
    }

    // MARK: Disabled Exit
    @Override
    public void disabledExit() {
    }

    // MARK: Autonomous Init
    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    // MARK: Autonomous Periodic
    @Override
    public void autonomousPeriodic() {}

    // MARK: Autonomous Exit
    @Override
    public void autonomousExit() {
        // NamedCommands.getCommand("ShootStop");
    }

    // MARK: Teleop Init
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    // MARK: Teleop Periodic
    @Override
    public void teleopPeriodic() {
        // NetworkTablesUtil.put("Hub is Active", rebuiltHubManager.hubIsActive());
        // NetworkTablesUtil.put("First Inactive Hub", rebuiltHubManager.getInactiveFirstAlliance());
        Logger.recordOutput("RebuiltHubManager/IsActive", rebuiltHubManager.hubIsActive());
        Logger.recordOutput("RebuiltHubManager/InactiveFirst", rebuiltHubManager.getInactiveFirstAlliance().toString());
    }

    // MARK: Teleop Exit
    @Override
    public void teleopExit() {}

    // MARK: Test Init
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    // MARK: Test Periodic
    @Override
    public void testPeriodic() {}

    // MARK: Test Exit
    @Override
    public void testExit() {}

    // MARK: Simulation Periodic
    @Override
    public void simulationPeriodic() {}


    // MARK: Log DriverStation
    private void logDriveStationValues() {
        Logger.recordOutput("DriverStation/GameSpecificMessage", DriverStation.getGameSpecificMessage());
        Logger.recordOutput("DriverStation/MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("DriverStation/MatchType", DriverStation.getMatchType());
        Logger.recordOutput("DriverStation/MatchNumber", DriverStation.getMatchNumber());
        Logger.recordOutput("DriverStation/IsAutonomous", DriverStation.isAutonomous());
        Logger.recordOutput("DriverStation/AutonomousEnabled", DriverStation.isAutonomousEnabled());
        Logger.recordOutput("DriverStation/IsEnabled", DriverStation.isEnabled());
        Logger.recordOutput("DriverStation/IsEStopped", DriverStation.isEStopped());
    }
}
