package frc.robot.subsystems.vision;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String[] cameraIds =
    new String[] {
        "Barbary Fig",
        "Saguaro",
        "Golden Barrel"
      };

    public static final Transform3d[] cameraPoses =
    new Transform3d[] {
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.5),
                Units.inchesToMeters(5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Math.PI)),
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-10), 
                Units.inchesToMeters(-6.5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Units.degreesToRadians(-110))),
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.5),
                Units.inchesToMeters(5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Math.PI))
    };

    public static AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}