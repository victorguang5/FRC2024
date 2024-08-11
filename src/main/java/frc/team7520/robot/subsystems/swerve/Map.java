package frc.team7520.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Map {

    /* For details check out https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf */
    private Pose2d robotPose;
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

    
    //public double xdistanceTag, ydistanceTag, zdistanceTag = 0;

    public Map(Pose2d robotPose) {
        periodic(robotPose);
    }

    public void periodic(Pose2d robotPose) {
        this.robotPose = robotPose;
        //closestAT();

    }

    public Pose2d updateRobotPose(int id, Transform3d transformation) {
        Pose2d currentAT = APRILTAGS[id-1];
        //make use of robotPose and tag pose
    }

    public Rotation2d aprilTagDirection() {
        
    }

    /**
     * Checks which aprilTag is the closest to the robot. Loop depends on which side of the field you are on.
     */
    private void closestAT() {
        closestAT = ORGANIZED_AT[allianceSide][0];
        for (int i = 0; i < 8; i++) {
            closestAT = compareAT(closestAT, ORGANIZED_AT[allianceSide][i]);
        } 
        
    }

    /**
     * Checks which side of the field you are on. Red Alliance's Source is (0,0).
     */
    private void checkSide() {
        if (robotPose.getX() < FIELD_CENTER) {
            allianceSide = 1; // blue
        } else {
            allianceSide = 0; // red
        }
    }

    /**
     * Calculates the distance between the given aprilTag and robot pose
     * @param aprilTag
     * @return
     */
    private double checkDistance(Pose2d aprilTag) {
        return aprilTag.getTranslation().getDistance(robotPose.getTranslation());
    }

    /**
     * Compares which of the two given aprilTags have a smaller distance the robot
     * @param aprilTag1
     * @param aprilTag2
     * @return a Pose2d representing an AprilTag
     */
    private Pose2d compareAT(Pose2d aprilTag1, Pose2d aprilTag2) {
        if (checkDistance(aprilTag1) <= checkDistance(aprilTag2)) {
            return aprilTag1;
        }
        return aprilTag2;
    }

    /**
     * Calculates the abs x and y distance from the current position to a target position. This method makes use of literal 2d vectors through
     * the Translation2d library. In theory, find a position vector relative to the robot, find it's angle to the forward facing vector, and 
     * adding that to the heading find the abs vector with its direction, and therefore the abs x and y components. Very easy!
     * 
     * <p> By Robin
     * @param measurement the target percieved in 3 dimensions
     */
    public void vectorCalculatedDistanceTag(Transform3d measurement) {
        // What if we analyzed component paths as 2d vectors using translation2d?
        Translation2d relativeVector = new Translation2d(measurement.getX(), measurement.getY());
        Translation2d absoluteVector = new Translation2d(relativeVector.getNorm(), new Rotation2d(getHeading().getRadians() + relativeVector.getAngle().getRadians()));
        //SmartDashboard.putNumber("Distance To Target", relativeVector.getNorm());
        xdistanceTag = absoluteVector.getX();
        ydistanceTag = absoluteVector.getY();
    }

}
