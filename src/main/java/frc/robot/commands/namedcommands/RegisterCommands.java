package frc.robot.commands.namedcommands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeConstants;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class RegisterCommands {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    FeederSubsystem feederSubsystem;

    public RegisterCommands(
        IntakeSubsystem intakeSubsystem, 
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
    }
    
    public void registerCommands(){
        // MARK: ShootFullSpeed
        NamedCommands.registerCommand("ShootFullSpeed",
            Commands.run(
                () -> {
                    shooterSubsystem.shooterMotors.drive();
                }
            ).withTimeout(5)
        );
        
        // MARK: IntakeDown
        NamedCommands.registerCommand("IntakeDown",
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE);
                }, intakeSubsystem
            )
        );

        // MARK: IntakeUp
        NamedCommands.registerCommand("IntakeUp",
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setPosition(IntakeConstants.INTAKE_MAX_VALUE);
                }, intakeSubsystem
            )
        );

        // MARK: IntakeToggle
        NamedCommands.registerCommand("IntakeToggle", 
            Commands.runOnce(
                () -> {
                    if (intakeSubsystem.anglePosition == IntakeConstants.INTAKE_MAX_VALUE) {
                        intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE);
                    } else {
                        intakeSubsystem.setPosition(IntakeConstants.INTAKE_MAX_VALUE);
                    }
                }, intakeSubsystem
            ));

        // MARK: IntakeAgitate
        NamedCommands.registerCommand("IntakeAgitate",
            Commands.sequence(
                Commands.runOnce(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MAX_VALUE / 2)),
                Commands.waitSeconds(0.75),
                Commands.runOnce(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE))
            )
        );

        // MARK: AgitateAutoFuel
        NamedCommands.registerCommand("AgitateAutoFuel",
            Commands.repeatingSequence(
                Commands.runOnce(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MAX_VALUE / 2)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE)),
                Commands.waitSeconds(2)
            )
        );

        // MARK: RunIntakeIn
        NamedCommands.registerCommand("IntakeIn",
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                }
            )
        );

        // MARK: 
        NamedCommands.registerCommand("RunAllFeederIn",
            Commands.runOnce(
                () -> {
                    feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                }
            )
        );
    }
}
