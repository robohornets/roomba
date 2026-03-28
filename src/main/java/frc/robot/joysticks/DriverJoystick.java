package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

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
    }

    public void configureBindings() {
        // MARK: Intake Down - A
        joystick.a();
        // MARK: Intake Agitate - B
        joystick.b().whileTrue(
            NamedCommands.getCommand("IntakeAgitate")
        );

        // MARK: X - Lock to hub
        joystick.x().onTrue(
            Commands.runOnce(
                () -> {
                    drivetrain.toggleLockedToHub();
                }
            )
        );

        // MARK: Reset Shooter Angle - Y
        joystick.y().whileTrue(
            Commands.runEnd(
                () -> {
                    drivetrain.setWiggleAgitation(true);
                },
                () -> {
                    drivetrain.setWiggleAgitation(false);
                }
            )
        );

        // MARK: RT - Shooter shoot
        joystick.rightTrigger().whileTrue(
            NamedCommands.getCommand("ShootWheel")
        ).onFalse(
            NamedCommands.getCommand("ShootStop")
        );

        // MARK: LT - Intake
        joystick.leftTrigger()
            .whileTrue(
                Commands.runEnd(
                    () -> intakeSubsystem.setIntake(IntakeStates.INTAKE_IN),
                    () -> intakeSubsystem.setIntake(IntakeStates.OFF),
                    intakeSubsystem
                )
            );

        joystick.rightBumper().whileTrue(
            Commands.runEnd(
                () -> feederSubsystem.setFeederState(FeederState.ALL_FEEDER_OUT),
                () -> feederSubsystem.setFeederState(FeederState.OFF),
                feederSubsystem
            )
        );
        
        // MARK: Intake Out - LB
        joystick.leftBumper().onTrue(
            NamedCommands.getCommand("IntakeDown")
        );


        joystick.povUp();

        // Reset Field Centric Heading
        joystick.povDown().onTrue(
            drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric)
        );

        joystick.povLeft();

        joystick.povRight();
    }
}