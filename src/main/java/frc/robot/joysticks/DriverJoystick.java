package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterConstants;
import frc.robot.subsystems.mechanisms.shooter.ShooterDataPoint;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    public DriverJoystick(
        CommandXboxController joystick, 
        Drive drivetrain, 
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        FeederSubsystem feederSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
        
        Logger.recordOutput("DriverJoystick/ShooterSpeed", 0.0);
    }

    public void configureBindings() {
        // MARK: Intake Toggle - A
        joystick.a().onTrue(
            NamedCommands.getCommand("IntakeToggle")
        );

        // MARK: Intake Agitate - B
        joystick.b().onTrue(
            NamedCommands.getCommand("IntakeAgitate")
        );

        // MARK: X - Lock to hub
        joystick.x().onTrue(
            Commands.runOnce(
                () -> {
                    drivetrain.toggleLockedToHub();
                    Logger.recordOutput("SwerveDrive/LockedToHub", drivetrain.isLockedToHub());
                }
            )
        );

        // MARK: Reset Shooter Angle - Y
        joystick.y().whileTrue(
            Commands.run(
                () -> {
                    shooterSubsystem.shooterPitchMotor.goTo(ShooterConstants.SHOOTER_MAX_ANGLE);
                }
            )
        );

        final ShooterDataPoint[] saveShooterDataPoint = {new ShooterDataPoint(0.0, 0.0, 0.0)};

        // MARK: RT - Shooter shoot
        joystick.rightTrigger()
            .whileTrue(
                Commands.parallel(
                    Commands.startRun(
                        () -> {
                            ShooterDataPoint shooterDataPoint = saveShooterDataPoint[0];
    
                            shooterDataPoint = shooterSubsystem.shooterCalculateTrajectory();
                            double rpm = shooterSubsystem.getRequiredRPM(shooterDataPoint);
                            double maxRPM = 1200; // MARK: Populate max rpm
    
                            shooterDataPoint.speed = rpm / maxRPM;
                            
                            // shooterDataPoint.angle = ( shooterDataPoint.angle - 65) / 180; // angle (0.5 = 180deg)
    
                            saveShooterDataPoint[0] = shooterDataPoint;
                        },
                        () -> {
                            ShooterDataPoint shooterDataPoint = saveShooterDataPoint[0];

                            Logger.recordOutput("DriverJoystick/ShooterSpeed", shooterDataPoint.speed);
                            Logger.recordOutput("DriverJoystick/ShooterPitch", shooterDataPoint.angle);
    
                            // maintain motor speed
                            shooterSubsystem.shooterMotors.drive(shooterDataPoint.speed);
                            shooterSubsystem.shooterPitchMotor.goTo(shooterDataPoint.angle);
    
                            saveShooterDataPoint[0] = shooterDataPoint;
                        },
                        shooterSubsystem, drivetrain
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        Commands.runOnce(
                            ()->{
                                feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                            }, feederSubsystem
                        )
                    )
                )
            )
            .onFalse(
                Commands.runOnce(
                    () -> {

                        shooterSubsystem.shooterMotors.drive(0.0);
                        shooterSubsystem.shooterPitchMotor.goTo(ShooterConstants.SHOOTER_MAX_ANGLE);

                        feederSubsystem.setFeederState(FeederState.OFF);

                    },
                    shooterSubsystem
                )
            );

        // MARK: LT - Intake
        joystick.leftTrigger()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                    },
                    () -> {
                        intakeSubsystem.setIntake(IntakeStates.OFF);
                    },
                    intakeSubsystem
                )
            );

        joystick.rightBumper().whileTrue(
            Commands.runEnd(
                () -> {
                    feederSubsystem.setFeederState(FeederState.ALL_FEEDER_OUT);
                },
                () -> {
                    feederSubsystem.setFeederState(FeederState.OFF);
                }, feederSubsystem
            )
        );
        
        // MARK: Intake Out - LB
        joystick.leftBumper().whileTrue(
            Commands.runEnd(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.INTAKE_OUT);
                },
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.OFF);
                }, intakeSubsystem
            )
        );

        joystick.povUp().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("SwerveDrive/SetSwervePoseLimelight", true);

                    drivetrain.resetPose(LimelightHelpers.getBotPose2d("limelight-four"));
                }
            )
        );

        // Reset Field Centric Heading
        joystick.povDown().onTrue(
            drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric)
        );

        joystick.povLeft().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("QuestNav/SetQuestPose", true);
                    // Reset QuestNav pose to Limelight position
                    drivetrain.questNavSubsystem.setQuestPose(
                        LimelightHelpers.getBotPose3d_wpiBlue("limelight-four")
                            .transformBy(
                                new Transform3d(LimelightConstants.LIMELIGHT_4_TRANSFORM_FROM_CENTRE).inverse()
                            )
                    );

                    Logger.recordOutput("QuestNav/SetQuestPose", false);
                }
            )
        );

        joystick.povRight();
    }
}