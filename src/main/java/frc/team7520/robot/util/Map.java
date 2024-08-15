/*
 * Robin Yan
 * 08/10/2024
 * A class designed to be a container of custom position/robot destinations.
 */
package frc.team7520.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

public class Map {

    /* For details check out https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf 
     * The red alliance's source, located on the blue alliance's side of the field, is (0,0)
     * 
     * Here is the official statement:
     * 
     * The XYZ Origin is established in the bottom left corner of
     * the field (as viewed in the image above). An x
     * coordinate of 0 is aligned with the Blue Alliance Station
     * diamond plate. A y coordinate of 0 is aligned with the
     * side border polycarbonate on the Scoring Table side of
     * the field. A z coordinate of 0 is on the carpet.
     * +Z is up into the air from the carpet, +X is horizontal to the
     * right (in this image above) toward the opposing alliance
     * stations, and +Y runs from the Field Border towards the SPEAKERS.
     * The face-pose of the tags is denoted with 1 degree
     * representation, the Z-rotation. 
     * 
     * 0° faces the red alliance
     * station, 90° faces the non- scoring table side, and 180°
     * faces the blue alliance station. Distances are measured to the
     * center of the tag.
     * 
     * ** Keep in mind that intake is the head! **
     */
    private Pose2d redSpeakerCenter = new Pose2d(15.2, 5.5, new Rotation2d(Math.toRadians(180)));
    private Pose2d redSpeakerSourceSide = new Pose2d(15.8, 4.3, new Rotation2d(Math.toRadians(-120)));
    private Pose2d redSpeakerAmpSide = new Pose2d(15.8, 6.6, new Rotation2d(Math.toRadians(120)));
    private Pose2d redAmp = new Pose2d();

    private Pose2d blueSpeakerCenter = new Pose2d(1.3, 5.5, new Rotation2d(0));
    private Pose2d blueSpeakerSourceSide = new Pose2d(0.7, 4.3, new Rotation2d(Math.toRadians(-60)));
    private Pose2d blueSpeakerAmpSide = new Pose2d(0.7, 6.6, new Rotation2d(Math.toRadians(60)));
    private Pose2d blueAmp = new Pose2d();
    
    /* 
    final private Pose2d AT1 = new Pose2d(new Translation2d(15.079472, 0.245872), new Rotation2d(Math.toRadians(120))); // B SOURCE 
    final private Pose2d AT2 = new Pose2d(new Translation2d(16.185134, 0.883666), new Rotation2d(Math.toRadians(120))); // B SOURCE
    final private Pose2d AT3 = new Pose2d(new Translation2d(16.579342, 4.982718), new Rotation2d(Math.toRadians(180))); // R SPEAKER 
    final private Pose2d AT4 = new Pose2d(new Translation2d(16.579342, 5.547868), new Rotation2d(Math.toRadians(180))); // R SPEAKER (0",104.5")
    final private Pose2d AT5 = new Pose2d(new Translation2d(14.700758, 8.2042), new Rotation2d(Math.toRadians(270))); // R AMP (76.1", 0")
    final private Pose2d AT6 = new Pose2d(new Translation2d(1.8415, 8.2042), new Rotation2d(Math.toRadians(270))); // B AMP (574.9", 0")
    final private Pose2d AT7 = new Pose2d(new Translation2d(-0.0381, 5.547868), new Rotation2d(Math.toRadians(0))); // B SPEAKER (651", 104.5")
    final private Pose2d AT8 = new Pose2d(new Translation2d(-0.0381, 4.982718), new Rotation2d(Math.toRadians(0))); // B SPEAKER
    final private Pose2d AT9 = new Pose2d(new Translation2d(0.356108, 0.883666), new Rotation2d(Math.toRadians(60))); // R SOURCE
    final private Pose2d AT10 = new Pose2d(new Translation2d(1.461516, 0.245872), new Rotation2d(Math.toRadians(60))); // R SOURCE
    final private Pose2d AT11 = new Pose2d(new Translation2d(11.904726, 3.713226), new Rotation2d(Math.toRadians(300))); // R STAGE
    final private Pose2d AT12 = new Pose2d(new Translation2d(11.904726, 4.49834), new Rotation2d(Math.toRadians(60))); // R STAGE
    final private Pose2d AT13 = new Pose2d(new Translation2d(11.220196, 4.105148), new Rotation2d(Math.toRadians(180))); // R STAGE
    final private Pose2d AT14 = new Pose2d(new Translation2d(5.320792, 4.105148), new Rotation2d(Math.toRadians(0))); // B STAGE
    final private Pose2d AT15 = new Pose2d(new Translation2d(4.641342, 4.49834), new Rotation2d(Math.toRadians(120))); // B STAGE
    final private Pose2d AT16 = new Pose2d(new Translation2d(4.641342, 3.713226), new Rotation2d(Math.toRadians(240))); // B STAGE
    private Pose2d closestAT;
    final private Pose2d[][] ORGANIZED_AT = {{AT1, AT2, AT3, AT4, AT5, AT11, AT12, AT13}, // RED ALLIANCE WING
                                        {AT6, AT7, AT8, AT9, AT10, AT14, AT15, AT16}}; // BLUE ALLIANCE WING
    final private Pose2d[] APRILTAGS = {AT1, AT2, AT3, AT4, AT5, AT6, AT7, AT8, AT9, AT10, AT11, AT12, AT13, AT14, AT15, AT16};
    final private double FIELD_LENGTH = 16.54; // In meters
    final private double FIELD_WIDTH = 8.21; 
    final private double FIELD_CENTER = FIELD_LENGTH/2;
    private int allianceSide = 0;
    */
    //public double xdistanceTag, ydistanceTag, zdistanceTag = 0;

    public Map() {
    }


    /**
     * Gets the central shooting Pose2d at the speaker. Location depends the alliance set.
     * @return a Pose2d
     */
    public Pose2d getSpeakerCenter() {
        if (SwerveSubsystem.isBlueAlliance) {
            return blueSpeakerCenter;
        } else {
            return redSpeakerCenter;
        }
    }

    /**
     * Gets the source side shooting Pose2d at the speaker. Location depends the alliance set.
     * @return a Pose2d
     */
    public Pose2d getSpeakerSourceSide() {
        if (SwerveSubsystem.isBlueAlliance) {
            return blueSpeakerSourceSide;
        } else {
            return redSpeakerSourceSide;
        }
    }

    /**
     * Gets the amp side shooting Pose2d at the speaker. Location depends the alliance set.
     * @return a Pose2d
     */
    public Pose2d getSpeakerAmpSide() {
        if (SwerveSubsystem.isBlueAlliance) {
            return blueSpeakerAmpSide;
        } else {
            return redSpeakerAmpSide;
        }
    }

    /**
     * Gets the amp Pose2d. Location depends the alliance set.
     * @return a Pose2d.
     */
    public Pose2d getAmp() {
        if (SwerveSubsystem.isBlueAlliance) {
            return blueAmp;
        } else {
            return redAmp;
        }
    }
}
