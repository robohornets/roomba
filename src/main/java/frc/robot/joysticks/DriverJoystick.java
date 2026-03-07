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

        final double[] shooter = {0.0, 0.25};

        // MARK: RT - Shooter shoot
        joystick.rightTrigger()
            .whileTrue(
                Commands.startRun(
                    () -> {
                        // calculate required shooter speed and angle
                        ShooterDataPoint shooterDataPoint = shooterSubsystem.calculateShooterValues(shooterSubsystem.shooterUpperLower(), drivetrain.getDistanceToHub());
                        
                        // calculate required speed and angle

                        double rpm = shooterSubsystem.getRequiredRPM(shooterDataPoint);

                        shooter[0] = 0.5; // speed
                        shooter[1] = 0.25; // angle (0.5 = 180deg)
                    },
                    () -> {
                        Logger.recordOutput("DriverJoystick/ShooterSpeed", shooter[0]);
                        Logger.recordOutput("DriverJoystick/ShooterPitch", shooter[1]);

                        // maintain motor speed
                        shooterSubsystem.shooterMotors.drive(shooter[0]);
                        shooterSubsystem.shooterFeedMotor.drive(shooter[0]);
                        shooterSubsystem.shooterPitchMotor.goTo(shooter[1]);
                    },
                    shooterSubsystem, drivetrain
                )
            )
            .onFalse(
                Commands.run(
                    () -> {

                        shooter[0] = Math.max(0.0, shooter[0] - 0.05);
                        shooterSubsystem.shooterMotors.drive(shooter[0]);
                        shooterSubsystem.shooterFeedMotor.drive(shooter[0]);

                        shooterSubsystem.shooterPitchMotor.goTo(0.0);

                        Logger.recordOutput("DriverJoystick/ShooterSpeed", shooter[0]);
                        Logger.recordOutput("DriverJoystick/ShooterPitch", shooter[1]);
                    },
                    shooterSubsystem
                )
                .until(() -> shooter[0] <= 0.0)
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

        joystick.rightBumper();

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}
