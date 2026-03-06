// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.namedcommands.RegisterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.joysticks.DebugJoystick;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.joysticks.OperatorJoystick;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;


public class RobotContainer {
    // public static double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // public static double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // MARK: Drivetrain
    // Create the swerve drivetrain subsystem for the robot
    public final Drive drivetrain = new Drive(TunerConstants.createDrivetrain());


    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

    // MARK: Vision
    // Uses the Quest to periodically add vision measurements
    // public QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain);

    // LimelightSubsystem limelight2Subsystem = new LimelightSubsystem(drivetrain, "limelight-two");

    // MARK: Subsystems
    // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // public final MotorSubsystem motorSubsystem = new MotorSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivetrain);

    // MARK: Xbox Controllers
    public final DriverJoystick driverJoystick = new DriverJoystick(new CommandXboxController(0), drivetrain, shooterSubsystem, intakeSubsystem);
    public final OperatorJoystick operatorJoystick = new OperatorJoystick(new CommandXboxController(1), drivetrain, intakeSubsystem);
    public final DebugJoystick debugJoystick = new DebugJoystick(new CommandXboxController(2), drivetrain, shooterSubsystem, intakeSubsystem);
    
    // MARK: Register Commands
    public final RegisterCommands registerCommands = new RegisterCommands(intakeSubsystem, shooterSubsystem);
    

    // MARK: Tests
    // public final Tests tests = new Tests(intakeSubsystem, shooterSubsystem, climberSubsystem, motorSubsystem);

    // private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        
        registerCommands.registerCommands();

        // autoChooser = new LoggedDashboardChooser<>("Autonomous Mode", AutoBuilder.buildAutoChooser());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // MARK: Run Tests
        /* Disable tests on actual code */
        // tests.runTests();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    // MARK: Configure Bindings
    private void configureBindings() {
        driverJoystick.configureBindings();
        operatorJoystick.configureBindings();
        // debugJoystick.configureBindings();

        // Positive X is forward, Positive Y is left according to WPILib
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         Drive.drive
        //             .withVelocityX(-driverJoystick.joystick.getLeftY() * DriveConstants.MAX_SPEED) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverJoystick.joystick.getLeftX() * DriveConstants.MAX_SPEED) // Drive left with negative X (left)
        //             .withRotationalRate(-driverJoystick.joystick.getRightX() * DriveConstants.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
        //     )
        // );
        drivetrain.setDefaultCommand(drivetrain.joysticksDefaultCommand(driverJoystick.joystick));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverJoystick.joystick.b().whileTrue(drivetrain.applyRequest(() ->
            drivetrain.point.withModuleDirection(new Rotation2d(-driverJoystick.joystick.getLeftY(), -driverJoystick.joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/strt and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.joystick.back().and(driverJoystick.joystick.y()).whileTrue(drivetrain.drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.joystick.back().and(driverJoystick.joystick.x()).whileTrue(drivetrain.drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.joystick.start().and(driverJoystick.joystick.y()).whileTrue(drivetrain.drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.joystick.start().and(driverJoystick.joystick.x()).whileTrue(drivetrain.drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return autoChooser.get();
        return autoChooser.getSelected();
    }
}
