package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

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
                    shooterSubsystem.feedMotor.set(1.0);
                }
            )
        ).onFalse(
            Commands.runOnce(
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
            NamedCommands.getCommand("IntakeDown")
        );

        joystick.leftBumper().onTrue(
            NamedCommands.getCommand("IntakeUp")
        );

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}
