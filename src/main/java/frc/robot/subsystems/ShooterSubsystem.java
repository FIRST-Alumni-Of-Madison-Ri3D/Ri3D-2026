// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax flywheel;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    flywheel = new SparkMax(0, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();

    conf.smartCurrentLimit(80, 40);
    conf.inverted(Constants.DrivetrainConstants.kInvertLeft);
    conf.disableFollowerMode();
    flywheel.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setMotorSpeed(double speed) {
    if (speed > 1.0) {
      speed = 1.0;
    }
    if (speed < -1.0) {
      speed = -1.0;
    }
    flywheel.set(speed);
  }
}
