package frc.robot.joysticks;

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
        // MARK: Nothing - A
        joystick.a();

        // MARK: Nothing - B
        joystick.b();

        // MARK: Jostle Fuel - X
        joystick.x().onTrue(
            NamedCommands.getCommand("IntakeAgitate")
        );

        // MARK: Reset shooter hood - Y
        joystick.y().whileTrue(
            Commands.run(
                () -> {
                    shooterSubsystem.shooterPitchMotor.goTo(ShooterConstants.SHOOTER_MAX_ANGLE);
                }
            )
        );

        // MARK: nothing - LT
        joystick.leftTrigger();


        // MARK: nothing - LB
        joystick.leftBumper();

        // MARK: Shoot Feed In - RT
        joystick.rightTrigger().whileTrue(
            Commands.runEnd(
                () -> {
                    feederSubsystem.setFeederState(FeederState.ALL_FEEDER_IN);
                },
                () -> {
                    feederSubsystem.setFeederState(FeederState.OFF);
                }
            )
        );

        // MARK: Shoot Feed Out - RB
        joystick.rightBumper().whileTrue(
            Commands.runEnd(
                () -> {
                    feederSubsystem.setFeederState(FeederState.ALL_FEEDER_OUT);
                },
                () -> {
                    feederSubsystem.setFeederState(FeederState.OFF);
                }
            )
        );
    }
}