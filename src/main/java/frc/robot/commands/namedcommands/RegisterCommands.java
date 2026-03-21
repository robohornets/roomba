package frc.robot.commands.namedcommands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeConstants;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterConstants;
import frc.robot.subsystems.mechanisms.shooter.ShooterDataPoint;

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
        NamedCommands.registerCommand("ShootStart",
            Commands.parallel(
                Commands.run(
                    () -> {
                        ShooterDataPoint shooterDataPoint = shooterSubsystem.shooterCalculateTrajectory();

                        double rpm = shooterSubsystem.getRequiredRPM(shooterDataPoint);
                        double maxRPM = 1300; // MARK: Populate max rpm

                        shooterDataPoint.speed = rpm / maxRPM;
                        Logger.recordOutput("ShooterSubsystem/ShooterSpeed", shooterDataPoint.speed);

                    shooterSubsystem.shooterMotors.drive(shooterDataPoint.speed);
                },
                shooterSubsystem
            )
        );

        NamedCommands.registerCommand("ShootAllSystems",
            Commands.repeatingSequence(
                NamedCommands.getCommand("IntakeAgitate")
            ).beforeStarting(
                Commands.sequence(
                    Commands.waitSeconds(2),
                    Commands.runOnce(
                        ()->{
                            feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                        }, feederSubsystem
                    )
                )
            )
        );

        NamedCommands.registerCommand("ShootStop",
            Commands.run(
                () -> {
                    shooterSubsystem.shooterMotors.drive(0.0);
                    feederSubsystem.setFeederState(FeederState.OFF);
                    
                }, shooterSubsystem, feederSubsystem
            )
        );

        // MARK: IntakeDown
        NamedCommands.registerCommand("IntakeDown",
            Commands.runOnce(
                () -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE),
                intakeSubsystem
            )
        );

        // MARK: IntakeUp
        NamedCommands.registerCommand("IntakeUp",
            Commands.runOnce(
                () -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MAX_VALUE),
                intakeSubsystem
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
            Commands.repeatingSequence(
                Commands.runOnce(() -> intakeSubsystem.setPosition(0.35)),
                Commands.waitSeconds(0.75),
                Commands.runOnce(() -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE)),
                Commands.waitSeconds(0.75)
            )
        );

        // MARK: RunIntakeIn
        NamedCommands.registerCommand("IntakeIn",
            Commands.runOnce(
                () -> intakeSubsystem.setIntake(IntakeStates.INTAKE_IN),
                intakeSubsystem
            )
        );

        NamedCommands.registerCommand("FeederIn",
            Commands.run(
                () -> feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN),
                feederSubsystem
            )
        );

        NamedCommands.registerCommand("FeederOff",
            Commands.run(
                () -> feederSubsystem.setFeederState(FeederState.OFF),
                feederSubsystem
            )
        );

        NamedCommands.registerCommand("FeederOut",
            Commands.run(
                () -> feederSubsystem.setFeederState(FeederState.ALL_FEEDER_OUT),
                feederSubsystem
            )
        );
    }
}