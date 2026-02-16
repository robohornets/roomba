package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;

public class OperatorJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;

    public OperatorJoystick(CommandXboxController joystick, Drive drivetrain) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
    }

    public void configureBindings() {
        joystick.a();

        joystick.b();

        joystick.x();

        joystick.y();

        joystick.rightTrigger();

        joystick.leftTrigger();

        joystick.rightBumper();

        joystick.leftBumper();

        joystick.povUp();

        joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}