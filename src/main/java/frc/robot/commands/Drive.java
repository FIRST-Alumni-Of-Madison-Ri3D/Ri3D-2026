package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Drive extends Command {
    private final DrivetrainSubsystem drive;
    private final DoubleSupplier x;
    private final DoubleSupplier z;

    public Drive(DrivetrainSubsystem drive, DoubleSupplier x, DoubleSupplier z) {
        this.drive = drive;
        this.x = x;
        this.z = z;
    }

    public void execute() {
        drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
    }

    public void end(boolean interrupted) {
        drive.arcadeDrive(0.0, 0.0); //Stop
    }

    public boolean isFinished(boolean interrupted) {
        return false;
    }
}
