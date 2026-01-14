package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;


public class Tests {
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


    // Run tests
    public void runTests(){
        
        System.out.println(shooterSubsystem.getShooterYaw());
                
    }
}
