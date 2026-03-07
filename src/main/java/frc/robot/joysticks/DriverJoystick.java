package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.FeederState;
import frc.robot.subsystems.mechanisms.shooter.ShooterDataPoint;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private final DoubleEntry shooterSpeedEntry;

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

        NetworkTable table = NetworkTableInstance.getDefault().getTable("DriverJoystick");
        
        shooterSpeedEntry = table.getDoubleTopic("ShooterSpeed").getEntry(0.0);
        shooterSpeedEntry.set(0.0);
        
        Logger.recordOutput("DriverJoystick/ShooterSpeed", 0.0);
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

        final double[] shooterSpeed = {0.0};

        // MARK: RT - Shooter shoot
        joystick.rightTrigger()
            .whileTrue(
                Commands.startRun(
                    () -> {
                        // calculate required shooter speed and angle
                        ShooterDataPoint shooterDataPoint = shooterSubsystem.calculateShooterValues(shooterSubsystem.shooterUpperLower(), drivetrain.getDistanceToHub());
                        
                        // calculate required speed (0-1)

                        shooterSpeed[0] = 0.5;
                    },
                    () -> {
                        Logger.recordOutput("DriverJoystick/ShooterSpeed", shooterSpeed[0]);
                        // maintain motor speed
                        shooterSubsystem.shooterMotors.drive(shooterSpeed[0]);
                    },
                    shooterSubsystem, drivetrain
                )
            )
            .onFalse(
                Commands.run(
                    () -> {
                        shooterSpeed[0] = Math.max(0.0, shooterSpeed[0] - 0.05);
                        shooterSubsystem.shooterMotors.drive(shooterSpeed[0]);
                        Logger.recordOutput("DriverJoystick/ShooterSpeed", shooterSpeed[0]);
                    },
                    shooterSubsystem
                )
                .until(() -> shooterSpeed[0] <= 0.0)
            );

        // MARK: LT - Intake
        joystick.leftTrigger()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                    },
                    () -> {
                        intakeSubsystem.setIntake(IntakeStates.OFF);
                    },
                    intakeSubsystem
                )
            );

        // MARK: RB - Shooter feeder
        joystick.rightBumper()
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        shooterSubsystem.setFeeder(FeederState.FEEDER_IN);
                    }, 
                    () -> {
                        shooterSubsystem.setFeeder(FeederState.OFF);
                    },
                    shooterSubsystem
                )
            );

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}
