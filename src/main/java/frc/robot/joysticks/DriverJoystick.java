package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public DriverJoystick(
        CommandXboxController joystick, 
        Drive drivetrain, 
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void configureBindings() {
        joystick.a();

        joystick.b();

        // MARK: X - Lock to hub
        joystick.x().onTrue(
            Commands.runOnce(
                () -> {
                    drivetrain.toggleLockedToHub();
                    Logger.recordOutput("SwerveDrive/LockedToHub", drivetrain.isLockedToHub());
                }
            )
        );

        joystick.y();

        joystick.rightTrigger().whileTrue(
            Commands.run(
                () -> {
                    shooterSubsystem.feedMotor.set(0.5);
                }
            )
        ).onFalse(
            Commands.run(
                () -> {
                    shooterSubsystem.feedMotor.set(0.0);
                }
            )
        );

        joystick.leftTrigger().whileTrue(
            Commands.run(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                }
            )
        ).onFalse(
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.OFF);
                }
            )
        );

        joystick.rightBumper().onTrue(
            Commands.runOnce(
                () -> {
                    shooterSubsystem.toggleShooterMotors();
                }
            )
        );

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}
