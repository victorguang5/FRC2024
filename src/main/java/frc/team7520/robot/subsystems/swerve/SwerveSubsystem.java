// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import static frc.team7520.robot.Constants.Telemetry.SWERVE_VERBOSITY;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;
    /**
     * Maximum speed of the robot in meters per second, used to limit acceleration.
     */
    public double maximumSpeed = Units.feetToMeters(14.5);
    

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(Constants.Swerve.ANGLE_GEAR_RATIO, 1);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), Constants.Swerve.DRIVE_GEAR_RATIO, 1);
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");
        double MillisecondsToWait = 1000;
        double VelocityX = 1;
        double VelocityY = 0;
        double rotation = 0;
        SmartDashboard.putNumber("MillisecondsToWait", MillisecondsToWait);
        SmartDashboard.putNumber("VelocityX", VelocityX);
        SmartDashboard.putNumber("VelocityY", VelocityY);
        SmartDashboard.putNumber("rotation", rotation);
        System.out.println("ran here");

        
    
      

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = SWERVE_VERBOSITY;
        try {
//            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.

        swerveDrive.setMotorIdleMode(true);

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                     swerveDrive.swerveController.config.headingPIDF.i,
                                     swerveDrive.swerveController.config.headingPIDF.d), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
                );

    }

        

    public Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    
        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                     swerveDrive.swerveController.config.headingPIDF.i,
                                     swerveDrive.swerveController.config.headingPIDF.d), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    /**
     * Get the autonomous command for the robot.
     * @param autoName       Name of the auto file.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link PathPlannerAuto} command.
     */
    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        double x = translation.getX() / 4;
        double y = translation.getY() / 4;  
        Translation2d translate = new Translation2d(x, y);
        swerveDrive.drive(translate,
                rotation / 4,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d initialHolonomicPose) {
        swerveDrive.setGyro(new Rotation3d(0, 0, initialHolonomicPose.getRotation().getRadians()));

        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public void drivemeters() {
        double MillisecondsToWait = SmartDashboard.getNumber("MillisecondsToWait", 1000);
        double velocityX = SmartDashboard.getNumber("VelocityX", 1);
        double velocityY = SmartDashboard.getNumber("VelocityY", 0);
        double rotation = SmartDashboard.getNumber("rotation", 0);
        Translation2d translation = new Translation2d(velocityX, velocityY);
        boolean fieldRelative = false;
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
        System.out.println(MillisecondsToWait);
        WaitTime(MillisecondsToWait);
        translation = new Translation2d(0, 0);
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }
    
    private void WaitTime(double milliseconds) {
        try {
            Thread.sleep((int)milliseconds);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    public Command OnFly () {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(0)),
            // new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
            // new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
            // new Pose2d(2, -1, Rotation2d.fromDegrees(0)),
            // new Pose2d(0, 0, Rotation2d.fromDegrees(0))
            new Pose2d(3, 0, Rotation2d.fromDegrees(0))
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(45)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        RotationTarget rt = new RotationTarget(0.25, Rotation2d.fromDegrees(90), true);
        RotationTarget rt1 = new RotationTarget(0.5, Rotation2d.fromDegrees(180), true);
        RotationTarget rt2 = new RotationTarget(0.75, Rotation2d.fromDegrees(270), true);
        RotationTarget rt3 = new RotationTarget(1, Rotation2d.fromDegrees(0), true);
        List<RotationTarget> lst_rt = Arrays.asList(rt, rt1, rt2, rt3);
        ConstraintsZone cz = new ConstraintsZone(0.3, 0.6, new PathConstraints(0.05, 0.5, 0.5 * Math.PI, 0.5 * Math.PI));
        List<ConstraintsZone> lst_cz = Arrays.asList();
        EventMarker em = new EventMarker(0.7, null);
        List<EventMarker> lst_em = Arrays.asList();
    
        PathPlannerPath p2 = new PathPlannerPath(
            bezierPoints,
            lst_rt,
            lst_cz,
            lst_em,
            new PathConstraints(0.2, 0.5, 0.5 * Math.PI, 0.5 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)),
            false
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        return new FollowPathHolonomic(
                p2,
                this::getPose, // Robot pose supplier
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                     swerveDrive.swerveController.config.headingPIDF.i,
                                     swerveDrive.swerveController.config.headingPIDF.d), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }


    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("X Position", swerveDrive.getPose().getX());
        SmartDashboard.putNumber("Y Position", swerveDrive.getPose().getY());
        SmartDashboard.putNumber("Rotation", swerveDrive.getPose().getRotation().getDegrees());
        //SmartDashboard.putNumber("Raw Gyro", swerveDrive.);
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working. Pose set is 0.
     */
    public void resetOdometry() {
//        SmartDashboard.putNumber("ResetHeading", initialHolonomicPose.getRotation().getDegrees());

        swerveDrive.setGyro(new Rotation3d(0, 0, 0));

        swerveDrive.resetOdometry(new Pose2d());
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */


    

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     *  Set the target heading of the robot.
     *
     * @param angle The angle in degrees.
     */

    public void setTargetHeading(Rotation2d angle){

        swerveDrive.swerveController.lastAngleScalar = angle.getRadians();

    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);

        swerveDrive.swerveController.lastAngleScalar = angle.getRadians();
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    public void setGyro(Rotation2d yaw){
        swerveDrive.setGyro(new Rotation3d(0, 0, yaw.getRadians()));
    }

}
