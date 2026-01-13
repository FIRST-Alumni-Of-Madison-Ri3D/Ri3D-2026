// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberArmMotor;
  private final SparkClosedLoopController climberArmController;
  private final RelativeEncoder climberArmEncoder;

  private final SparkMax climberSpinMotor;
  //private final SparkClosedLoopController climberSpinController;
  private final RelativeEncoder climberSpinEncoder;

  /** Creates a new ClimberArmSubsystem. */
  public ClimberSubsystem() {
    climberArmMotor = new SparkMax(Constants.ClimberConstants.kClimberArmId, MotorType.kBrushless);
    climberArmController = climberArmMotor.getClosedLoopController();
    climberArmEncoder = climberArmMotor.getEncoder();

    climberSpinMotor = new SparkMax(Constants.ClimberConstants.kClimberSpinId, MotorType.kBrushless);
    //climberSpinController = climberSpinMotor.getClosedLoopController();
    climberSpinEncoder = climberSpinMotor.getEncoder();

    // Climber arm setup
    SparkMaxConfig climberArmConfig = new SparkMaxConfig();

    climberArmConfig.smartCurrentLimit(80);

    climberArmConfig.inverted(Constants.ClimberConstants.kArmInvert);

    climberArmConfig.idleMode(IdleMode.kBrake);

    climberArmConfig.closedLoop
      .p(Constants.ClimberConstants.kClimberArmP)
      .i(Constants.ClimberConstants.kClimberArmI)
      .d(Constants.ClimberConstants.kClimberArmD)
      .allowedClosedLoopError(Constants.ClimberConstants.kClimberArmAllowableError, ClosedLoopSlot.kSlot0);

    // climberArmConfig.closedLoop.feedForward
    //   .kS(Constants.ClimberConstants.kClimberArmS)
    //   .kV(Constants.ClimberConstants.kClimberArmV)
    //   .kA(Constants.ClimberConstants.kClimberArmA)
    //   .kCos(Constants.ClimberConstants.kClimberArmG)
    //   .kCosRatio(Constants.ClimberConstants.kClimberArmRatio);

    // climberArmConfig.closedLoop.maxMotion
    //   .cruiseVelocity(Constants.ClimberConstants.kClimberArmCruiseVelocity)
    //   .maxAcceleration(Constants.ClimberConstants.kClimberArmMaxAcceleration)
    //   .allowedProfileError(Constants.ClimberConstants.kClimberArmAllowedProfileError);

    climberArmMotor.configure(climberArmConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setArmEncoderPosition(ClimberConstants.kClimberArmStartPosition);

    // Climber spin setup
    SparkMaxConfig climberSpinConfig = new SparkMaxConfig();

    climberSpinConfig.smartCurrentLimit(80);

    climberSpinConfig.inverted(Constants.ClimberConstants.kSpinInvert);

    climberSpinConfig.idleMode(IdleMode.kBrake);

    // climberSpinConfig.closedLoop
    //   .p(Constants.ClimberConstants.kClimberSpinP)
    //   .i(Constants.ClimberConstants.kClimberSpinI)
    //   .d(Constants.ClimberConstants.kClimberSpinD)
    //   .allowedClosedLoopError(Constants.ClimberConstants.kClimberSpinAllowableError, ClosedLoopSlot.kSlot0);

    // climberSpinConfig.closedLoop.feedForward
    //   .kS(Constants.ClimberConstants.kClimberSpinS)
    //   .kV(Constants.ClimberConstants.kClimberSpinV)
    //   .kA(Constants.ClimberConstants.kClimberSpinA)
    //   .kCos(Constants.ClimberConstants.kClimberSpinG)
    //   .kCosRatio(Constants.ClimberConstants.kClimberSpinRatio);

    // climberSpinConfig.closedLoop.maxMotion
    //   .cruiseVelocity(Constants.ClimberConstants.kClimberSpinCruiseVelocity)
    //   .maxAcceleration(Constants.ClimberConstants.kClimberSpinMaxAcceleration)
    //   .allowedProfileError(Constants.ClimberConstants.kClimberSpinAllowedProfileError);

    climberSpinMotor.configure(climberSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    setSpinEncoderPosition(ClimberConstants.kClimberSpinStartPosition);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber arm position", getArmEncoderPosition());
    //SmartDashboard.putNumber("Climber spin position", getSpinEncoderPosition());
  }

  public Command setArmPositionCommand(double setpoint) {
    return runOnce(() -> climberArmController.setSetpoint(setpoint, ControlType.kPosition));
  }

  public double getArmEncoderPosition() {
    return climberArmEncoder.getPosition();
  }

  public void setArmEncoderPosition(double position) {
    climberArmEncoder.setPosition(position);
  }



  // public Command setSpinPositionCommand(double setpoint) {
  //   return runOnce(() -> climberSpinController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl));
  // }

  public double getSpinEncoderPosition() {
    return climberSpinEncoder.getPosition();
  }

  public void setSpinEncoderPosition(double position) {
    climberSpinEncoder.setPosition(position);
  }

  public void setSpinMotorOutput(double percentOutput) {
    climberSpinMotor.set(percentOutput);
  }

  public void setArmMotorOutput(double percentOutput) {
    climberArmMotor.set(percentOutput);
  }

  public Command stopAllMotors() {
    return new ParallelCommandGroup(new InstantCommand(() -> climberArmMotor.stopMotor()), new InstantCommand(()-> climberSpinMotor.stopMotor()));
  }
    
}
