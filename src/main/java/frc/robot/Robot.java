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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AdvantageKit.AdvantageKitConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Elastic;


 public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command lastSelectedAuto = null;

    private final RobotContainer robotContainer;

    private PowerDistribution powerDistributionHub = new PowerDistribution();

    CANBus canivore = new CANBus("Mechanisms");

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private int commandSchedulerLoops = 0;

    public Robot() {
        robotContainer = new RobotContainer();
    }

    // MARK: Robot Init
    @Override
    public void robotInit() {
        robotContainer.drivetrain.limelightSubsystem.setEnabled(false);

        // Configure logging for AdvantageKit
        Logger.recordMetadata("ProjectName", "1209Roomba");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (AdvantageKitConstants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                // Logger.addDataReceiver(new WPILOGWriter("/media/sda1/logs"));
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
        // robotContainer.shooterSubsystem.shooterPitchMotor.toggleEnabled(true);
        robotContainer.shooterSubsystem.leftShooterMotor.toggleEnabled(true);
        robotContainer.shooterSubsystem.rightShooterMotor.toggleEnabled(true);
        robotContainer.intakeSubsystem.intakeWheelsMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.feederBedMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.feederFeederMotor.toggleEnabled(true);
        robotContainer.feederSubsystem.shooterFeederMotor.toggleEnabled(true);

        // Apply and log motor configs to AdvantageKit
        // Logger.recordOutput("MotorConfig/ShooterPitch", robotContainer.shooterSubsystem.shooterPitchMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/LeftShooter", robotContainer.shooterSubsystem.leftShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/RightShooter", robotContainer.shooterSubsystem.rightShooterMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/IntakeWheelLeft", robotContainer.intakeSubsystem.intakeWheelLeft.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/IntakeWheelRight", robotContainer.intakeSubsystem.intakeWheelRight.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/FeederBed", robotContainer.feederSubsystem.feederBedMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/FeederFeeder",robotContainer.feederSubsystem.feederFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
        Logger.recordOutput("MotorConfig/ShooterFeeder", robotContainer.feederSubsystem.shooterFeederMotor.getMotor().getConfigurator().apply(RobotContainer.mechanismsMotorConfiguration).toString());
    }

    // MARK: Robot Periodic
    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

        if (infrequentPeriodic(2)) {
            robotContainer.statusChecks.runAllStatusChecks();

            logDriveStationValues();
            logPowerDistribution();
            logRobotController();
        }

        commandSchedulerLoops++;
    }

    // MARK: Disabled Init
    @Override
    public void disabledInit() {
        // Set Limelight to use external yaw to seed internal imu
        robotContainer.drivetrain.limelightSubsystem.setIMUMode(1);

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab("Disabled");
        }

        robotContainer.intakeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // MARK: Disabled Periodic
    @Override
    public void disabledPeriodic() {
        powerDistributionHub.clearStickyFaults();

        // Update starting pose in background
        if (DriverStation.isFMSAttached() || isSimulation()) {
            Command selectedAuto = robotContainer.getAutonomousCommand();
            if (selectedAuto != lastSelectedAuto) {
                lastSelectedAuto = selectedAuto;
                if (selectedAuto instanceof PathPlannerAuto auto) {
                    Pose2d startingPose = auto.getStartingPose();
                    if (startingPose != null) {
                        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
                            robotContainer.drivetrain.resetPose(FlippingUtil.flipFieldPose(startingPose));
                        }
                        else {
                            robotContainer.drivetrain.resetPose(startingPose);
                        }
                    }
                }
            }
        }
    }

    // MARK: Disabled Exit
    @Override
    public void disabledExit() {
        robotContainer.intakeSubsystem.angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // MARK: Autonomous Init
    @Override
    public void autonomousInit() {
        // Use external-only heading during auto so pose resets are reflected immediately
        robotContainer.drivetrain.limelightSubsystem.setIMUMode(0);

        CommandScheduler.getInstance().cancelAll();

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab("Autonomous");
        }
    }

    // MARK: Autonomous Periodic
    @Override
    public void autonomousPeriodic() {}

    // MARK: Autonomous Exit
    @Override
    public void autonomousExit() {
        NamedCommands.getCommand("ShootStop");
    }

    // MARK: Teleop Init
    @Override
    public void teleopInit() {
        // Set to use internal IMU as main and external as drift correction
        robotContainer.drivetrain.limelightSubsystem.setIMUMode(3);

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }

        if (DriverStation.isFMSAttached()) {
            Elastic.selectTab("Teleoperated");
        }
    }

    // MARK: Teleop Periodic
    @Override 
    public void teleopPeriodic() {}

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

    // MARK: InfrequentPeriodic
    private boolean infrequentPeriodic(int numCommandLoops) {
        return commandSchedulerLoops % numCommandLoops == 0;
    }


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
        Logger.recordOutput("DriverStation/IsFMSAttached", DriverStation.isFMSAttached());
        Logger.recordOutput("DriverStation/IsDSAttached", DriverStation.isDSAttached());
    }

    // MARK: Log PD
    private boolean pdRegistered = false;
    private void logPowerDistribution() {
        if (!pdRegistered) {
            SmartDashboard.putData("PowerDistribution", powerDistributionHub);
            pdRegistered = true;
        }
        Logger.recordOutput("PowerDistribution/AllChannelCurrents", powerDistributionHub.getAllCurrents());
        Logger.recordOutput("PowerDistribution/TotalCurrent", powerDistributionHub.getTotalCurrent());
        Logger.recordOutput("PowerDistribution/TotalEnergy", powerDistributionHub.getTotalEnergy());
        Logger.recordOutput("PowerDistribution/TotalPower", powerDistributionHub.getTotalPower());
        Logger.recordOutput("PowerDistribution/Voltage", powerDistributionHub.getVoltage());
        Logger.recordOutput("PowerDistribution/Temperature", powerDistributionHub.getTemperature());
    }

    // MARK: Log Robot Control
    private void logRobotController() {
        // Log RIO information
        Logger.recordOutput("RobotController/CPUTemp", RobotController.getCPUTemp());
        Logger.recordOutput("RobotController/CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
        Logger.recordOutput("RobotController/TXFullCount", RobotController.getCANStatus().txFullCount);
        Logger.recordOutput("RobotController/CommsDisableCount", RobotController.getCommsDisableCount());
        Logger.recordOutput("RobotController/RSLState", RobotController.getRSLState());
        Logger.recordOutput("RobotController/IsBrownedOut", RobotController.isBrownedOut());

        // Log CANivore information
        CANBus.CANBusStatus status = canivore.getStatus();
        Logger.recordOutput("CANivore/CANbusUtilization", status.BusUtilization);
        Logger.recordOutput("CANivore/TXFullCount", status.TxFullCount);
        Logger.recordOutput("CANivore/BussOffCount", status.BusOffCount);
    }
}
