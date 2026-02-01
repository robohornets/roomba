package frc.robot;

import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.motor.MotorSubsystem;


public class Tests {
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    ClimberSubsystem climberSubsystem;
    MotorSubsystem motorSubsystem;

    public Tests(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem, MotorSubsystem motorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.motorSubsystem = motorSubsystem;
    }
    // Run tests
    public void runTests(){
        
    }
}
