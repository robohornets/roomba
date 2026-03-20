package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.feeder.FeederState;
import frc.robot.subsystems.mechanisms.feeder.FeederSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterConstants;
import frc.robot.subsystems.mechanisms.shooter.ShooterDataPoint;
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

        final ShooterDataPoint[] saveShooterDataPoint = {new ShooterDataPoint(0.0, 0.0, 0.0)};

        // MARK: RT - Shooter shoot
        joystick.rightTrigger()
            .whileTrue(
                Commands.sequence(
                    Commands.startRun(
                        () -> {
                            ShooterDataPoint shooterDataPoint = saveShooterDataPoint[0];

                            shooterDataPoint = shooterSubsystem.shooterCalculateTrajectory();
                            double rpm = shooterSubsystem.getRequiredRPM(shooterDataPoint);

                            double maxRPM = 1200; // MARK: Populate max rpm
                            shooterDataPoint.speed = rpm / maxRPM;
                            // shooterDataPoint.angle = ( shooterDataPoint.angle - 65) / 180; // angle (0.5 = 180deg)

                            saveShooterDataPoint[0] = shooterDataPoint;
                        },
                        () -> {
                            ShooterDataPoint shooterDataPoint = saveShooterDataPoint[0];
                            Logger.recordOutput("DriverJoystick/ShooterSpeed", shooterDataPoint.speed);
                            Logger.recordOutput("DriverJoystick/ShooterTargetAngle", shooterDataPoint.angle);

                            // maintain motor speed
                            shooterSubsystem.shooterMotors.drive(shooterDataPoint.speed);
                            // shooterSubsystem.shooterPitchMotor.goTo(shooterDataPoint.angle);

                            saveShooterDataPoint[0] = shooterDataPoint;
                        },
                        shooterSubsystem, drivetrain
                    ),
                    Commands.runOnce(() -> {
                        feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                    }, feederSubsystem).withTimeout(2.0)
                )
            )
            .onFalse(
                Commands.runOnce(
                    () -> {
                        // ShooterDataPoint shooterDataPoint = saveShooterDataPoint[0];

                        // shooterDataPoint.speed = Math.max(0.0, shooterDataPoint.speed - 0.05);
                        shooterSubsystem.shooterMotors.drive(0.0);

                        feederSubsystem.setFeederState(FeederState.OFF);

                        shooterSubsystem.shooterPitchMotor.goTo(ShooterConstants.SHOOTER_MAX_ANGLE);

                        // Logger.recordOutput("DriverJoystick/ShooterSpeed", shooterDataPoint.speed);
                        // Logger.recordOutput("DriverJoystick/ShooterPitch", shooterDataPoint.angle);

                        // saveShooterDataPoint[0] = shooterDataPoint;
                    },
                    shooterSubsystem
                )
                // .until(() -> saveShooterDataPoint[0].speed <= 0.0)
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
