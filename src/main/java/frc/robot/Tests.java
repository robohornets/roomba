package frc.robot;

import frc.robot.subsystems.math.MathSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;


public class Tests {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    ClimberSubsystem climberSubsystem;
    MathSubsystem motorSubsystem;

    public Tests(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem, MathSubsystem motorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.motorSubsystem = motorSubsystem;
    }
    // Run tests
    public void runTests(){
        
    }
}
