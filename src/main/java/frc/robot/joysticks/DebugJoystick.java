package frc.robot.joysticks;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.limelight.LimelightConstants;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;

public class DebugJoystick {
    public final CommandXboxController joystick;
    private final Drive drivetrain;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleEntry shooterSpeedEntry;
    private double shooterSpeed;

    public DebugJoystick(
        CommandXboxController joystick, 
        Drive drivetrain,
        ShooterSubsystem shooterSubsystem, 
        IntakeSubsystem intakeSubsystem
    ) {
        this.joystick = joystick;
        this.drivetrain = drivetrain;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSpeed = 0.0;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");

        shooterSpeedEntry = table.getDoubleTopic("ShooterSpeed").getEntry(0.0);
        shooterSpeed = shooterSpeedEntry.get();
        shooterSpeedEntry.set(0.0);

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {

                    double leftJoystickValue = Math.abs(joystick.getLeftY()) > 0.05 ? -joystick.getLeftY(): 0.0;
                    double rightJoystickValue = Math.abs(joystick.getRightY()) > 0.05 ? -joystick.getRightY(): 0.0;

                    double speedChangeAmountPerTick = 0.005;
                    double change = Math.signum(leftJoystickValue) * speedChangeAmountPerTick;
                    shooterSpeed = MathUtil.clamp(shooterSpeed + change, -0.1, 1.0);
                    shooterSpeedEntry.set(shooterSpeed);


                    shooterSubsystem.leftShooterMotor.set(shooterSpeed);
                    shooterSubsystem.rightShooterMotor.set(shooterSpeed);
                    shooterSubsystem.feedMotor.set(rightJoystickValue);
                }, shooterSubsystem
            )
        );

        intakeSubsystem.setDefaultCommand(
            Commands.run(
                () -> {
                    intakeSubsystem.intakeWheelsMotor.set(joystick.getRightTriggerAxis());
                }, intakeSubsystem
            )
        );
    }

    public void configureBindings() {
        joystick.a();

        joystick.b();

        joystick.x();

        joystick.y().onTrue(
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setPosition(0.0);
                }
            )
        );

        // joystick.rightTrigger();

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
