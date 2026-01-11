// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberArmSubsystem extends SubsystemBase {
  private final SparkMax climberArmMotor;
  private final SparkClosedLoopController controller;
  

  /** Creates a new ClimberArmSubsystem. */
  public ClimberArmSubsystem() {
    climberArmMotor = new SparkMax(Constants.ClimberConstants.kClimberArmId, MotorType.kBrushless);
    controller = climberArmMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(80);

    config.inverted(Constants.ClimberConstants.kInvert);

    config.closedLoop
      .p(Constants.ClimberConstants.kP)
      .i(Constants.ClimberConstants.kI)
      .d(Constants.ClimberConstants.kD)
      .outputRange(-0.3, 0.3)
      .allowedClosedLoopError(1.0, ClosedLoopSlot.kSlot0);

    config.closedLoop.feedForward
      .kS(Constants.ClimberConstants.kS)
      .kV(Constants.ClimberConstants.kV)
      .kA(Constants.ClimberConstants.kA)
      .kCos(Constants.ClimberConstants.kG)
      .kCosRatio(Constants.ClimberConstants.encoderToPositionRatio);

    climberArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPositionSetpoint(double setpoint) {
    controller.setReference(setpoint, ControlType.kPosition);
  }

  
}
