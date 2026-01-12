// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public static final MutVoltage appliedVoltage = Volts.mutable(0);
  public static final MutAngle currentRotations = Rotations.mutable(0);
  public static final MutAngularVelocity currentVelocity = RPM.mutable(0);
  private final SparkMax flywheel;
  private final RelativeEncoder enc;
  private final SysIdRoutine routine;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    flywheel = new SparkMax(Constants.ShooterConstants.kMotorId, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();

    conf.smartCurrentLimit(80, 40);
    conf.idleMode(IdleMode.kCoast);
    conf.inverted(Constants.ShooterConstants.kInvertMotor);
    conf.disableFollowerMode();

    conf.closedLoop
      .p(Constants.ShooterConstants.kP)
      .feedForward
      .kV(Constants.ShooterConstants.kV)
      .kA(Constants.ShooterConstants.kA);
    
    flywheel.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.2), Volts.of(7), Seconds.of(60)),
      new SysIdRoutine.Mechanism(this::setMotorVoltage, this::logSysID, this)
    );

    enc = flywheel.getEncoder();
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM (actual)", flywheel.getEncoder().getVelocity());
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

  public void setMotorVoltage(Voltage voltage) {
    flywheel.setVoltage(voltage.magnitude());
  }

  public void setMotorRPM(double rpm) {
    flywheel.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command stopMotor() {
    return runOnce(() -> flywheel.stopMotor());
  }

  public void logSysID(SysIdRoutineLog log) {
    log.motor("shooter")
      .voltage(appliedVoltage.mut_replace(flywheel.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
      .angularPosition(currentRotations.mut_replace(enc.getPosition(), Rotations))
      .angularVelocity(currentVelocity.mut_replace(enc.getVelocity(), RPM));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
