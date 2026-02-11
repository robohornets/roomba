// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import com.btwrobotics.WhatTime.frc.DashboardManagers.NetworkTablesUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.namedcommands.RegisterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DebugJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.joysticks.OperatorJoystick;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.motor.MotorSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import frc.robot.subsystems.vision.limelight.LimelightSubsystem;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

public class RobotContainer {
    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // MARK: Drivetrain
    // Create the swerve drivetrain subsystem for the robot
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Field centric drive
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    // Robot centric drive
    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // MARK: Vision
    // Uses the Quest to periodically add vision measurements
    public QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain);

    // Read AprilTags from the Limelight periodically to add vision measurements
    LimelightSubsystem limelightSubsystem = new LimelightSubsystem(drivetrain, "limelight-four");
    LimelightSubsystem limelight2Subsystem = new LimelightSubsystem(drivetrain, "limelight-two");


    // MARK: Xbox Controllers
    public final DriverJoystick driverJoystick = new DriverJoystick(new CommandXboxController(0), drivetrain, questNavSubsystem);
    public final OperatorJoystick operatorJoystick = new OperatorJoystick(new CommandXboxController(1), drivetrain);
    public final DebugJoystick debugJoystick = new DebugJoystick(new CommandXboxController(2), drivetrain);

    
    // MARK: Subsystems
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final MotorSubsystem motorSubsystem = new MotorSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivetrain);
    
    
    // MARK: Register Commands
    public final RegisterCommands registerCommands = new RegisterCommands(intakeSubsystem, shooterSubsystem, climberSubsystem, motorSubsystem);
    

    // MARK: Tests
    public final Tests tests = new Tests(intakeSubsystem, shooterSubsystem, climberSubsystem, motorSubsystem);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        
        registerCommands.registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        NetworkTablesUtil.put("Auto Mode", autoChooser);

        // MARK: Run Tests
        /* Disable tests on actual code */
        tests.runTests();

        configureBindings();
        configureDefaults();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }


    private void configureDefaults() {
        
        
    }

    private void configureBindings() {
        driverJoystick.configureBindings();
        operatorJoystick.configureBindings();
        debugJoystick.configureBindings();

        // Positive X is forward, Positive Y is left according to WPILib
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverJoystick.joystick.povUp().onTrue(
            Commands.runOnce(
                () -> {
                    // NetworkTable Table = NetworkTablesUtil.getTable("limelight-two");
                    // NetworkTableEntry Entry = Table.getEntry("botpose");
                    // double[] pos = Entry.getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0});
                    // Translation2d translation2d = new Translation2d(pos[0], pos[1]);
                    
                    // Rotation2d rotation2d = new Rotation2d(drivetrain.getPigeon2().getYaw().getValueAsDouble());
                    // LimelightHelpers.getRobotPose_FieldSpace2D()
                    
                    drivetrain.resetPose(LimelightHelpers.getBotPose2d("limelight-four"));
                }
            )
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverJoystick.joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.joystick.getLeftY(), -driverJoystick.joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/strt and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.joystick.back().and(driverJoystick.joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.joystick.back().and(driverJoystick.joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.joystick.start().and(driverJoystick.joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.joystick.start().and(driverJoystick.joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
