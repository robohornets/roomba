package frc.robot.commands.namedcommands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeConstants;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterDataPoint;

public class RegisterCommands {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    FeederSubsystem feederSubsystem;
    Drive drivetrain;

    public RegisterCommands(
        IntakeSubsystem intakeSubsystem, 
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem,
        Drive drivetrain
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.drivetrain = drivetrain;
    }
    
    public void registerCommands(){

        NamedCommands.registerCommand("WiggleStart",
            Commands.runOnce(
                () ->  drivetrain.setWiggleAgitation(true),
                drivetrain
            )
        );
        NamedCommands.registerCommand("WiggleStop",
            Commands.runOnce(
                () ->  drivetrain.setWiggleAgitation(false),
                drivetrain
            )
        );

        NamedCommands.registerCommand("ShootWheel", 
            Commands.run(
                () -> {
                    if (shooterSubsystem.canShoot()) {
                        ShooterDataPoint shooterDataPoint = shooterSubsystem.shooterCalculateTrajectory();

                        double rpm = shooterSubsystem.getRequiredRPM(shooterDataPoint);
                        double maxRPM = 1300; // MARK: Populate max rpm

                        shooterDataPoint.speed = rpm / maxRPM;
                        // shooterDataPoint.speed = 0.30;
                        Logger.recordOutput("ShooterSubsystem/ShooterSpeed", shooterDataPoint.speed);

                        shooterSubsystem.setFlywheelSpeed(shooterDataPoint.speed);
                    }
                },
                shooterSubsystem
            )
        );

        NamedCommands.registerCommand("ShootStop",
            Commands.runOnce(
                () -> {
                    shooterSubsystem.setFlywheelSpeed(0.0);
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

        NamedCommands.registerCommand("IntakeOff",
            Commands.runOnce(
                () -> intakeSubsystem.setIntake(IntakeStates.OFF),
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
