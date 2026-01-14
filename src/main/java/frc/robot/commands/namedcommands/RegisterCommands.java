package frc.robot.commands.namedcommands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;;

public class RegisterCommands {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    ClimberSubsystem climberSubsystem;

    public RegisterCommands(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
    }

    public void registerCommands(){
        NamedCommands.registerCommand("shoot",
            shooterSubsystem.alignShooterYawToHub()
                
        );
    }
}
