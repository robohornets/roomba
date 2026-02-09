package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;

    public DebugJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain) {
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

        // Reset Field Centric Heading
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.povLeft();

        joystick.povRight();
    }
}
