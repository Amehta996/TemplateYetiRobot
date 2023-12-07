package frc.robot.subsystems;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;

public class VisionSubsystem extends Limelight {
    private NetworkTableInstance nti = null;

    TemplateDrivetrainSubsystem drivetrainSubsystem;
    public double xFinal, yFinal;

    Transform3d cameraToTarget;
    Pose3d object;

    public VisionSubsystem() {
        nti = NetworkTableInstance.getDefault();
        cameraToTarget = new Transform3d(new Translation3d(drivetrainSubsystem.getOdometry().getPoseMeters().getX(), drivetrainSubsystem.getOdometry().getPoseMeters().getY(), drivetrainSubsystem.getOdometry().getPoseMeters().getRotation().getDegrees()), new Rotation3d(drivetrainSubsystem.gyro.getRoll(), drivetrainSubsystem.gyro.getPitch(), drivetrainSubsystem.gyro.getYaw()));

        object = new Pose3d();
    }


    public boolean hasTargets() {
        return hasTarget();
    }

    public double getX() {
        return getTx();
    }

    public double getY() {
        return getTy();
    }

    public Pose3d goalToPose() {
        return ComputerVisionUtil.objectToRobotPose(object, cameraToTarget, new Transform3d());
    }
}
