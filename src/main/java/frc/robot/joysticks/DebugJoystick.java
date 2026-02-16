package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;
import gg.questnav.questnav.QuestNav;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final QuestNavSubsystem questNavSubsystem;

    public DebugJoystick(
        CommandXboxController joystick, 
        Drive drivetrain,
        QuestNavSubsystem questNavSubsystem
    ) {
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

        joystick.povUp();

        // Reset Field Centric Heading
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric));

        joystick.povLeft().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("QuestNav/SetQuestPose", true);
                    // Reset QuestNav pose to Limelight position
                    questNavSubsystem.setQuestPose(LimelightHelpers.getBotPose3d("limelight-four"));
                }
            )
        );

        joystick.povRight();
    }
}
