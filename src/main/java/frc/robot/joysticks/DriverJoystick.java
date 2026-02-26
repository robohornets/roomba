package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;

    public DriverJoystick(
        CommandXboxController joystick, 
        Drive drivetrain
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
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
