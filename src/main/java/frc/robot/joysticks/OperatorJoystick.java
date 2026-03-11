package frc.robot.joysticks;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
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
        // MARK: Intake Down
        joystick.a().onTrue(
            NamedCommands.getCommand("IntakeDown")
        );

        // MARK: Intake Up
        joystick.b().onTrue(
            NamedCommands.getCommand("IntakeUp")
        );


        joystick.x();

        joystick.y();

        // MARK: Shooter accelerate
        joystick.rightTrigger();

        // MARK: Intake in
        joystick.leftTrigger();

        // MARK: Shooter feeder
        joystick.rightBumper();

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}