// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberFlipCommand extends Command {

  private DrivetrainSubsystem driveSubsystem;
  private ClimberSubsystem climberSubsystem;

  private double currentRollAngle;
  private double targetRollAngle = 180;
  private double allowedError = 2;

  private ProfiledPIDController spinController;

  /** Creates a new ClimberFlipCommand. */
  public ClimberFlipCommand(DrivetrainSubsystem drive, ClimberSubsystem climber) {
    driveSubsystem = drive;
    climberSubsystem = climber;

    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("max vel", 0);
    SmartDashboard.putNumber("max accel", 0);

    // spinController = new ProfiledPIDController(Constants.ClimberConstants.kClimberSpinP,
    //   Constants.ClimberConstants.kClimberSpinI,
    //   Constants.ClimberConstants.kClimberSpinD,
    //   new Constraints(Constants.ClimberConstants.kClimberSpinMaxVelocity, Constants.ClimberConstants.kClimberSpinMaxAcceleration));

    // spinController.setGoal(targetRollAngle);
    // spinController.setTolerance(allowedError);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinController = new ProfiledPIDController(SmartDashboard.getNumber("p", 0),
      Constants.ClimberConstants.kClimberSpinI,
      Constants.ClimberConstants.kClimberSpinD,
      new Constraints(SmartDashboard.getNumber("max vel", 0), SmartDashboard.getNumber("max accel", 0)));

    spinController.setGoal(targetRollAngle);
    spinController.setTolerance(allowedError);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRollAngle = driveSubsystem.getRollAngle();
    
    climberSubsystem.setSpinMotorOutput(spinController.calculate(currentRollAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spinController.atGoal();
  }
}
