package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;

    public DebugJoystick(
        CommandXboxController joystick, 
        Drive drivetrain
    ) {
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
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric));

        joystick.povLeft().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("QuestNav/SetQuestPose", true);
                    // Reset QuestNav pose to Limelight position
                    drivetrain.questNavSubsystem.setQuestPose(LimelightHelpers.getBotPose3d_wpiBlue("limelight-four"));
                    
                    Logger.recordOutput("QuestNav/SetQuestPose", false);
                }
            )
        );

        joystick.povRight();
    }
}
