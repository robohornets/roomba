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
        // MARK: Intake Down
        joystick.a().onTrue(
            NamedCommands.getCommand("IntakeDown")
        );

        joystick.b();

        joystick.x();

        joystick.y();

        // MARK: Intake In - LT
        joystick.leftTrigger().onTrue(
                // Toggle state of the intake
                Commands.runOnce(
                    () -> {
                        if (intakeSubsystem.getIntakeState().equals(IntakeStates.INTAKE_IN)) {
                            intakeSubsystem.setIntake(IntakeStates.OFF);
                        }
                        else {
                            intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                        }
                    }
                )
            );

        // MARK: Intake Out - LB
        joystick.leftBumper().whileTrue(
            Commands.runEnd(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.OFF);
                },
                () -> {
                    // Reset intake to last state.
                    intakeSubsystem.setIntake(intakeSubsystem.lastIntakeState);
                }
            )
        );

        joystick.rightTrigger();

        joystick.rightBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}