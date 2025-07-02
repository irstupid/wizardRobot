package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
            System.err.println("something bad happened with yagsl initilization" + e.getMessage());
        }
        pathPlan();
    }

    private void pathPlan() {
        try {
            AutoBuilder.configure(
                    drive::getPose,
                    drive::resetOdometry,
                    drive::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        drive.drive(speedsRobotRelative, drive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                moduleFeedForwards.linearForces());
                    },
                    new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                    RobotConfig.fromGUISettings(), () -> {
                        return false;
                    },
                    this);
        } catch (Exception e) {
            System.err.println("something bad happened with pathPlanner initilization" + e.getMessage());
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
