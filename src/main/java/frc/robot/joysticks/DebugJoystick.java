package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleEntry shooterTargetTest;

    public DebugJoystick(
        CommandXboxController joystick, 
        Drive drivetrain,
        ShooterSubsystem shooterSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.shooterSubsystem = shooterSubsystem;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");

        shooterTargetTest = table.getDoubleTopic("ShooterAngleEntry").getEntry(1.0);

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {
                    shooterSubsystem.leftShooterMotor.set(joystick.getLeftY());
                    shooterSubsystem.rightShooterMotor.set(joystick.getLeftY());
                }, shooterSubsystem
            )
        );
    }

    public void configureBindings() {
        joystick.a();

        joystick.b();

        joystick.x();

        joystick.y();

        joystick.rightTrigger().whileTrue(
            Commands.run(
                () -> {
                    shooterSubsystem.feedMotor.set(Math.min(joystick.getRightTriggerAxis(), 0.5));
                }
            )
        );

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
        joystick.povDown().onTrue(
            drivetrain.runOnce(drivetrain.drivetrain::seedFieldCentric)
        );

        joystick.povLeft().onTrue(
            Commands.runOnce(
                () -> {
                    Logger.recordOutput("QuestNav/SetQuestPose", true);
                    // Reset QuestNav pose to Limelight position
                    drivetrain.questNavSubsystem.setQuestPose(
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
