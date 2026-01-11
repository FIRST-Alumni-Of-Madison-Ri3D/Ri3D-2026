package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SparkMax fl;
    private final SparkMax fr;
    //private final SparkMax bl;
    //private final SparkMax br;

    private final DifferentialDrive drive;

    private final Pigeon2 pigeon;

    public DrivetrainSubsystem() {
        fl = new SparkMax(Constants.DrivetrainConstants.kFrontLeftSparkId, MotorType.kBrushed);
        fr = new SparkMax(Constants.DrivetrainConstants.kFrontRightSparkId, MotorType.kBrushed);
        //bl = new SparkMax(Constants.DrivetrainConstants.kBackLeftSparkId, MotorType.kBrushed);
        //br = new SparkMax(Constants.DrivetrainConstants.kBackRightSparkId, MotorType.kBrushed);
        
        SparkMaxConfig conf = new SparkMaxConfig();

        conf.smartCurrentLimit(80, 40);

        conf.inverted(Constants.DrivetrainConstants.kInvertLeft);
        fl.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //conf.follow(fl, false);
        //bl.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //conf.disableFollowerMode();

        conf.inverted(Constants.DrivetrainConstants.kInvertRight);
        fr.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //conf.follow(fr, false);
        //br.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        drive = new DifferentialDrive(fl, fr);

        pigeon = new Pigeon2(Constants.DrivetrainConstants.kPigeonId);
    }

    public void arcadeDrive(double x, double z) {
        drive.arcadeDrive(x, z, true);
    }

    public double getYawAngle() {
        return pigeon.getYaw().getValueAsDouble();
    }

    public double getPitchAngle() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getRollAngle() {
        return pigeon.getRoll().getValueAsDouble();
    }
}