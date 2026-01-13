// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int kFrontLeftSparkId = 1;
    public static final int kBackLeftSparkId = 2;
    public static final int kFrontRightSparkId = 3;
    public static final int kBackRightSparkId = 4;

    public static final int kPigeonId = 5;

    public static final boolean kInvertLeft = true;
    public static final boolean kInvertRight = false;
  }

  public static class ShooterConstants {
    public static final int kMotorId = 20;
    public static final boolean kInvertMotor = false;

    public static final double kP = 7.0131E-05;
    public static final double kV = 0.1391 / 60;
    public static final double kA = 0.049104 / 60;

    public static final double kShooterRPM = 2800;
    public static final double kShooterSlowInRPM = 2000;
    public static final double kClearShooterRPM = -2000;
  }
  
  public static class IntakeConstants {
    public static final int kIntakeId = 11;
    public static final boolean kInvertIntake = false;

    public static final double kIntakeInVoltage = 10.5;
    public static final double kIntakeOutVoltage = -6;
  }

  public static class IntakeArmConstants {
    public static final int kIntakeArmId = 12;

    public static final boolean kInvert = false;

    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0.01;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0.5;

    /**
     * Conversion factor from motor encoder to angle of arm, with straight toward front of robot being 0 degrees
     */
    public static final double kEncoderToPositionRatio = 0.05;

    /**
     * Experimentally determined value to hold intake at.
     */
    public static final double  kHoldPosition = 0.015;
    public static final double kInPosition = 0.24;
  }
}
