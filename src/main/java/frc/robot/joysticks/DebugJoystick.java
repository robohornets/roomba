package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    private double shooterSpeed;
    private double shooterPitch;

    public DebugJoystick(
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

        this.shooterSpeed = 0.0;
        this.shooterPitch = 0.0;

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {

                    double leftJoystickValue = Math.abs(joystick.getLeftY()) > 0.05 ? -joystick.getLeftY(): 0.0;
                    double rightJoystickValue = Math.abs(joystick.getRightY()) > 0.05 ? -joystick.getRightY(): 0.0;

                    double changeAmountPerTick = 0.0025;
                    double change = Math.signum(leftJoystickValue) * changeAmountPerTick;
                    shooterSpeed = MathUtil.clamp(shooterSpeed + change, -0.1, 1.0);

                    shooterPitch = MathUtil.clamp(shooterPitch + Math.signum(rightJoystickValue) * changeAmountPerTick, 5.0 ,65.0);

                    shooterSubsystem.shooterMotors.drive(shooterSpeed);

                    Logger.recordOutput("ShooterSubsystem/ShooterSpeed", shooterSpeed);
                    Logger.recordOutput("ShooterSubsystem/FeederSpeed", rightJoystickValue);
                }, shooterSubsystem
            )
        );

        // intakeSubsystem.setDefaultCommand(
        //     Commands.run(
        //         () -> {
        //             double triggerSpeed = joystick.getLeftTriggerAxis() > joystick.getRightTriggerAxis() ? -joystick.getLeftTriggerAxis(): joystick.getRightTriggerAxis();
        //             intakeSubsystem.intakeWheelsMotor.getMotor().set(triggerSpeed);

        //             Logger.recordOutput("IntakeSubsystem/WheelSpeed", triggerSpeed);
        //         }, intakeSubsystem
        //     )
        // );
    }

    public void configureBindings() {
        joystick.a().onTrue(
            NamedCommands.getCommand("IntakeDown")
        );

        joystick.b().onTrue(
            NamedCommands.getCommand("IntakeUp")
        );

        joystick.x().onTrue(
            Commands.runOnce(
                () -> {
                    feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                }
            )
        );

        joystick.y().onTrue(
            Commands.runOnce(
                () -> {
                    feederSubsystem.setFeederState(FeederState.OFF);
                }
            )
        );

        joystick.leftTrigger()
            .whileTrue(
                Commands.runEnd(
                    () -> intakeSubsystem.setIntake(IntakeStates.INTAKE_IN),
                    () -> intakeSubsystem.setIntake(IntakeStates.OFF),
                    intakeSubsystem
                )
            );

        joystick.leftBumper().onTrue(
            Commands.runOnce(
                () -> {
                    shooterSubsystem.shooterPitchMotor.set(50);
                }
            )
        );

        // Reset pose to limelight output
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
