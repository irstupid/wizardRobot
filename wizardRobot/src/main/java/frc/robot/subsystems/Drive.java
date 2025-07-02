package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
    SwerveDrive drive;
    double maxSpeed = 0.5;
    PhotonCamera camera = new PhotonCamera("LeftApriltag");

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

    public Command simpleDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        return run(() -> {
            double sX = x.getAsDouble() * drive.getMaximumChassisVelocity();
            double sY = y.getAsDouble() * drive.getMaximumChassisVelocity();
            double sR = r.getAsDouble() * drive.getMaximumChassisAngularVelocity();
            drive.drive(new Translation2d(sX, sY), sR, false, false);
        });
    }

    public Command visionPoint() {
        return run(() -> {
            PhotonPipelineResult result = camera.getLatestResult();
            double x = 0;
            double y = 0;
            double r = 0;
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                System.out.println(target.yaw);
                r = target.yaw / 6 * -maxSpeed;
            }
            drive.drive(new Translation2d(x, y), r, false, false);
        });
    }

    public Command visionDistance() {
        return run(() -> {
            PhotonPipelineResult result = camera.getLatestResult();
            double x = 0;
            double y = 0;
            double r = 0;
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                double distance = PhotonUtils.calculateDistanceToTargetMeters(0.381, 0.635, 0,
                        Units.degreesToRadians(target.getPitch()));
                System.out.println(distance);
                // r = target.yaw / 6 * -maxSpeed;
            }
            drive.drive(new Translation2d(x, y), r, false, false);
        });
    }
}
