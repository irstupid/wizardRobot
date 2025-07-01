package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
    SwerveDrive drive;
    double maxSpeed = 1;

    public Drive() {
        try {
            drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(maxSpeed);
        } catch (IOException e) {
            System.err.println("something bad happened " + e.getMessage());
        }
    }

    public Command printFrontLeft() {
        return runOnce(() -> {
            System.out.println(drive.getModules()[0].getAbsolutePosition());
        });
    }

    public Command printFrontRight() {
        return runOnce(() -> {
            System.out.println(drive.getModules()[1].getAbsolutePosition());
        });
    }

    public Command printBackLeft() {
        return runOnce(() -> {
            System.out.println(drive.getModules()[2].getAbsolutePosition());
        });
    }

    public Command printBackRight() {
        return runOnce(() -> {
            System.out.println(drive.getModules()[3].getAbsolutePosition());
        });
    }

    public Command test() {
        return run(() -> {
            drive.drive(new Translation2d(1, 0), 0, true, false);
        });
    }

    public Command simpleDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        return run(() -> {
            double sX = x.getAsDouble() * drive.getMaximumChassisVelocity();
            double sY = y.getAsDouble() * drive.getMaximumChassisVelocity();
            double sR = r.getAsDouble() * drive.getMaximumChassisAngularVelocity();
            drive.drive(new Translation2d(sX, sY), sR, false, false);
        });
    }
}
