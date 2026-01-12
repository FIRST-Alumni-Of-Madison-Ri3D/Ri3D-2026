// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkMax intakeArmMotor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    intakeArmMotor = new SparkMax(Constants.IntakeArmConstants.kIntakeArmId, MotorType.kBrushless);
    controller = intakeArmMotor.getClosedLoopController();
    encoder = intakeArmMotor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(80);

    config.inverted(Constants.IntakeArmConstants.kInvert);

    config.encoder.positionConversionFactor(Constants.IntakeArmConstants.kEncoderToPositionRatio);

    config.closedLoop
      .p(Constants.IntakeArmConstants.kP)
      .i(Constants.IntakeArmConstants.kI)
      .d(Constants.IntakeArmConstants.kD);

    config.closedLoop.feedForward
      .kS(Constants.IntakeArmConstants.kS)
      .kV(Constants.IntakeArmConstants.kV)
      .kA(Constants.IntakeArmConstants.kA)
      .kCos(Constants.IntakeArmConstants.kG);

    config.softLimit.forwardSoftLimit(0.24);
    config.softLimit.reverseSoftLimit(-0.05);

    intakeArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    controller.setSetpoint(Constants.IntakeArmConstants.kHoldPosition, ControlType.kPosition);

    encoder.setPosition(0.25);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Angle (actual)", encoder.getPosition());
  }

  public void setPositionSetpoint(double setpoint) {
    System.out.println(setpoint);
    intakeArmMotor.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
  }

  public void setEncoderPosition(double setpoint) {
    encoder.setPosition(setpoint);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public Command setPositionCommand(double setpoint) {
    return runOnce(() -> setPositionSetpoint(setpoint));
  }
}
