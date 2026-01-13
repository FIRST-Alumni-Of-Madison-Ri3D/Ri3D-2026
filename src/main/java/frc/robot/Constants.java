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
    public static final int kClimberArmId = 31;
    public static final int kClimberSpinId = 32;

    public static final boolean kArmInvert = false;
    public static final boolean kSpinInvert = false;

    public static final double kClimberArmP = 0;
    public static final double kClimberArmI = 0;
    public static final double kClimberArmD = 0;
    /** In absolute arm revolutions? */
    public static final double kClimberArmAllowableError = 0.05;

    public static final double kClimberArmS = 0;
    public static final double kClimberArmV = 0;
    public static final double kClimberArmA = 0;
    public static final double kClimberArmG = 0;

    /**
     * Conversion factor from motor encoder to absolute rotation of arm, with straight toward front of robot being 0
     */
    public static final double kClimberArmRatio = 1.0;

    public static final double kClimberArmStartPosition = 0;
    public static final double kClimberArmOutPosition = 90;
    public static final double kClimberArmClimbPosition = 0;

    /** In RPM */
    public static final double kClimberArmCruiseVelocity = 100;

    /** In RPM per second */
    public static final double kClimberArmMaxAcceleration = 50;

    /** In absolute arm rotations */
    public static final double kClimberArmAllowedProfileError = 0.1;


    
    public static final double kClimberSpinP = 0;
    public static final double kClimberSpinI = 0;
    public static final double kClimberSpinD = 0;
    /** In absolute arm revolutions? */
    public static final double kClimberSpinAllowableError = 0.05;

    public static final double kClimberSpinS = 0;
    public static final double kClimberSpinV = 0;
    public static final double kClimberSpinA = 0;
    public static final double kClimberSpinG = 0;

    /**
     * Conversion factor from motor encoder to absolute rotation of arm, with straight toward front of robot being 0
     */
    public static final double kClimberSpinRatio = 1.0;

    public static final double kClimberSpinStartPosition = 0;
    public static final double kClimberSpinClimbPosition = 180;

    /** In degrees per second */
    public static final double kClimberSpinMaxVelocity = 100;

    /** In degrees per second per second */
    public static final double kClimberSpinMaxAcceleration = 50;

    /** In degrees */
    public static final double kClimberSpinAllowedProfileError = 0.1;


  }
}
