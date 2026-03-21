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
import frc.robot.subsystems.mechanisms.shooter.ShooterConstants;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class OperatorJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;
    
    public OperatorJoystick(
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
        joystick.a().onTrue(
            NamedCommands.getCommand("IntakeDown")
        );

        // MARK: Intake Up - B
        joystick.b().onTrue(
            NamedCommands.getCommand("IntakeUp")
        );

        // MARK: Lock to hub - X
        joystick.x().onTrue(
            Commands.runOnce(
                () -> {
                    drivetrain.toggleLockedToHub();
                    Logger.recordOutput("SwerveDrive/LockedToHub", drivetrain.isLockedToHub());
                }
            )
        );

        // MARK: Intake Agitate - Y
        joystick.y().whileTrue(
            NamedCommands.getCommand("IntakeAgitate")
        );

        // MARK: RT - Shooter shoot
        joystick.rightTrigger()
            .whileTrue(
               NamedCommands.getCommand("ShootStart")
            )
            .onFalse(
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
        joystick.leftBumper().whileTrue(
            Commands.runEnd(
                () -> intakeSubsystem.setIntake(IntakeStates.INTAKE_OUT),
                () -> intakeSubsystem.setIntake(IntakeStates.OFF),
                intakeSubsystem
            )
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