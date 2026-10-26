package frc.robot.joysticks;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class OperatorJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final IntakeSubsystem intakeSubsystem;
    public OperatorJoystick(
        CommandXboxController joystick, 
        Drive drivetrain, 
        ShooterSubsystem shooterSubsystem,
        IntakeSubsystem intakeSubsystem,
        FeederSubsystem feederSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.intakeSubsystem = intakeSubsystem;


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
            NamedCommands.getCommand("IntakeReset")
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
        joystick.rightBumper().whileTrue(
            NamedCommands.getCommand("FeederOut")
        ).onFalse(
            NamedCommands.getCommand("FeederOff")
        );
        
        joystick.rightTrigger().whileTrue(
            NamedCommands.getCommand("FeederIn")
        ).onFalse(
            NamedCommands.getCommand("FeederOff")
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