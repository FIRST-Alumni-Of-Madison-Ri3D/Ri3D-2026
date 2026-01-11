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

  public static class ClimberConstants {
    public static final int kClimberArmId = 10;

    public static final boolean kInvert = false;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    /** In absolute arm revolutions? */
    public static final double kAllowableError = 0.05;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    /**
     * Conversion factor from motor encoder to absolute rotation of arm, with straight toward front of robot being 0
     */
    public static final double kEncoderToPositionRatio = 1.0;

    public static final double kClimberStartPosition = 0;
    public static final double kClimberOutPosition = 180;
    public static final double kClimberClimbPosition = 90;

    /** In RPM */
    public static final double kClimberCruiseVelocity = 100;

    /** In RPM per second */
    public static final double kClimberMaxAcceleration = 50;

    /** In absolute arm rotations */
    public static final double kClimberAllowedProfileError = 0.1;


  }
}
