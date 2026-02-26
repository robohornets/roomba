package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.intake.IntakeStates;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;

public class OperatorJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final IntakeSubsystem intakeSubsystem;

    public OperatorJoystick(
        CommandXboxController joystick, 
        Drive drivetrain,
        IntakeSubsystem intakeSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void configureBindings() {
        joystick.a();

        joystick.b();

        joystick.x();

        joystick.y();

        // MARK: Intake out
        joystick.rightTrigger().onTrue(
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.INTAKE_OUT);
                }
            )
        );

        // MARK: Intake in
        joystick.leftTrigger().onTrue(
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setIntake(IntakeStates.INTAKE_IN);
                }
            )
        );

        joystick.rightBumper().onTrue(
            Commands.runOnce(
                () -> {
                    
                }
            )
        );

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}