package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

public class DriverJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final QuestNavSubsystem questNavSubsystem;

    public DriverJoystick(CommandXboxController joystick, Drive drivetrain, QuestNavSubsystem questNavSubsystem) {
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

        // Reset pose to limelight output
        joystick.povUp().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("SwerveDrive/SetSwervePoseLimelight", true);

                    drivetrain.resetPose(LimelightHelpers.getBotPose2d("limelight-four"));
                }
            )
        );

        // Reset Field Centric Heading
        joystick.povDown().onTrue(drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric));

        //joystick.povDown();

        joystick.povLeft().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("QuestNav/SetQuestPose", true);
                    // Reset QuestNav pose to Limelight position
                    questNavSubsystem.setQuestPose(
                        LimelightHelpers.getBotPose3d_wpiBlue("limelight-four")
                            .transformBy(
                                new Transform3d(LimelightConstants.LIMELIGHT_4_TRANSFORM_FROM_CENTRE).inverse()
                            )
                    );

                    Logger.recordOutput("QuestNav/SetQuestPose", false);
                }
            )
        );

        joystick.povRight();
    }
}
