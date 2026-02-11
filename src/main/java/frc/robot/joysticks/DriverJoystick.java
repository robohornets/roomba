package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final CommandSwerveDrivetrain drivetrain;
    private final QuestNavSubsystem questNavSubsystem;

    public DriverJoystick(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain, QuestNavSubsystem questNavSubsystem) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.questNavSubsystem = questNavSubsystem;
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

        // Reset Field Centric Heading
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //joystick.povDown();

        joystick.povLeft();

        joystick.povRight();
    }
}
