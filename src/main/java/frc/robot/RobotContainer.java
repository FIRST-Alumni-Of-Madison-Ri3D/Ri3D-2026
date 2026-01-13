// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TimedShooterCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drivetrainSubsystem.setDefaultCommand(new RunCommand( () -> drivetrainSubsystem.arcadeDrive(-m_driverController.getLeftY() * 0.8, -m_driverController.getLeftX() * 0.8), drivetrainSubsystem));

    // SmartDashboard.putNumber("Shooter RPM", 0);
    // shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setMotorRPM(SmartDashboard.getNumber("Shooter RPM", 0)), shooterSubsystem));

    // SmartDashboard.putNumber("Intake RPM", 0);
    // intakeSubsystem.setDefaultCommand(intakeSubsystem.teleop(m_driverController::getRightTriggerAxis, ()->-SmartDashboard.getNumber("Intake RPM", 0)));

    SmartDashboard.putNumber("Intake Angle", 0);
    // intakeArmSubsystem.setDefaultCommand(new RunCommand(() -> intakeArmSubsystem.setPositionSetpoint(SmartDashboard.getNumber("Intake Angle", 0)), intakeArmSubsystem));

    //SmartDashboard.putNumber("Shooter Unjam", -1000);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.y().whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kReverse));
    // m_driverController.a().whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_driverController.b().whileTrue(shooterSubsystem.sysIdDynamic(Direction.kForward));
    // m_driverController.x().whileTrue(shooterSubsystem.sysIdDynamic(Direction.kReverse));

    m_driverController.leftBumper().whileTrue(intakeSubsystem.setIntakeVoltage(() -> Constants.IntakeConstants.kIntakeInVoltage)
      .alongWith(intakeArmSubsystem.setPositionCommand(SmartDashboard.getNumber("Intake Angle", 0))));

    m_driverController.leftBumper().whileFalse(intakeSubsystem.stopIntake()
      .alongWith(intakeArmSubsystem.setPositionCommand(Constants.IntakeArmConstants.kInPosition)));

    // m_driverController.rightBumper().onTrue(new TimedShooterCommand(shooterSubsystem, Constants.ShooterConstants.kShooterSlowInRPM, 0.8)
    //   .andThen(new TimedShooterCommand(shooterSubsystem, Constants.ShooterConstants.kClearShooterRPM, 0.5))
    //   .andThen(new RunCommand(() -> shooterSubsystem.setMotorRPM(ShooterConstants.kShooterRPM), shooterSubsystem)));

    m_driverController.rightBumper().whileTrue(new RunCommand(() -> shooterSubsystem.setMotorRPM(ShooterConstants.kShooterRPM), shooterSubsystem));

    m_driverController.rightBumper().onFalse(shooterSubsystem.stopMotor());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
