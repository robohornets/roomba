package frc.robot.commands.namedcommands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class RegisterCommands {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public RegisterCommands(
        IntakeSubsystem intakeSubsystem, 
        ShooterSubsystem shooterSubsystem
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }
    
    public void registerCommands(){
        // MARK: ShootBallsClose
        NamedCommands.registerCommand("ShootFullSpeed",
            Commands.run(
                () -> {
                    shooterSubsystem.shooterMotors.runForward();
                }
            ).withTimeout(5)
        );
        
        // MARK: IntakeDown
        NamedCommands.registerCommand("IntakeDown",
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setPosition(0.0);
                }
            )
        );

        // MARK: IntakeUp
        NamedCommands.registerCommand("IntakeUp",
            Commands.runOnce(
                () -> {
                    intakeSubsystem.setPosition(-6.0);
                }
            )
        );
    }
}
