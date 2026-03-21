package frc.robot.commands.namedcommands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
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
            NamedCommands.getCommand("ShootWheel")
            .alongWith(
                Commands.waitSeconds(2.0)
                .andThen(
                    NamedCommands.getCommand("FeederIn")
                    .alongWith(
                        NamedCommands.getCommand("IntakeAgitate")
                    )
                )
            )
        );

        NamedCommands.registerCommand("ShootWithFeeder",
            NamedCommands.getCommand("ShootWheel")
            .alongWith(
                Commands.waitSeconds(2.0)
                .andThen(
                    NamedCommands.getCommand("FeederIn")
                )
            )
        );

        NamedCommands.registerCommand("ShootWheel",
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

        NamedCommands.registerCommand("ShootStop",
            Commands.runOnce(
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
            ).finallyDo(
                () -> intakeSubsystem.setPosition(IntakeConstants.INTAKE_MIN_VALUE)
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
            Commands.runOnce(
                () -> feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN),
                feederSubsystem
            )
        );

        NamedCommands.registerCommand("FeederOff",
            Commands.runOnce(
                () -> feederSubsystem.setFeederState(FeederState.OFF),
                feederSubsystem
            )
        );

        NamedCommands.registerCommand("FeederOut",
            Commands.runOnce(
                () -> feederSubsystem.setFeederState(FeederState.ALL_FEEDER_OUT),
                feederSubsystem
            )
        );
    }
}
