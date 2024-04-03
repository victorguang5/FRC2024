package frc.team7520.robot.util;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.gamepiece.GamePieceSubsystem;
//import frc.robot.constants.AutoConstants;
//import frc.robot.subsystems.swerve.SwerveBase;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class PathPlannerHelper {
    static Pose2d lastPhotonPose2d;
    static Pose2d lastEndPose2d;
    static Pose2d lastMidPose2d;

    public static BooleanSupplier getAllianceColorBooleanSupplier(){
        return () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }

    public static void initializeAutoBuilder(SwerveSubsystem s_Swerve){
        s_Swerve.setupPathPlanner();
    }


    public static Command GoToPoseCommand_Preplanned(SwerveSubsystem s_Swerve, String pathName)
    {
        lastPhotonPose2d = null;
        lastEndPose2d = null;
        lastMidPose2d = null;
        return new SequentialCommandGroup(
            goToPoseCommand_preplanned(s_Swerve, pathName),
            goToPose(s_Swerve, lastEndPose2d),
            goToPose(s_Swerve, lastEndPose2d),
            goToPose(s_Swerve, lastEndPose2d)
        );
    } 
    


    private static Pose2d ConvertToAbsolutePose(SwerveSubsystem s_Swerve, Pose2d srcPose)
    {
        if (srcPose == null) return null;

        var deltaPose = srcPose;
        Pose2d startPose = s_Swerve.getPose();

        Translation2d startTranslation2d = startPose.getTranslation();
        Translation2d deltaTranslation2d = deltaPose.getTranslation();
        Translation2d endTranslation2d = startTranslation2d.plus(deltaTranslation2d);
        Rotation2d starRotation2d = startPose.getRotation();
        Rotation2d deltaRotation2d = deltaPose.getRotation();
        Rotation2d endRotation2d = starRotation2d.plus(deltaRotation2d);

        Pose2d endPose = new Pose2d(endTranslation2d,endRotation2d);

        return endPose;
    }



    private static Command goToPose_photon(SwerveSubsystem s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return null;

        // go to the endPose directly, finish cart heading rotation in the middle
        Pose2d startPose = s_Swerve.getPose();
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );
        
        RotationTarget rt = new RotationTarget(
            0.5, 
            endPose.getRotation(), false);
        List<RotationTarget> rts = Arrays.asList(rt);
        List<ConstraintsZone> czs = Arrays.asList();
        List<EventMarker> ems = Arrays.asList();
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                rts,
                czs, // list ConstraintsZone
                ems, // list event marker
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, endPose.getRotation()), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                false
        );
        
        path.preventFlipping =true;
        
        return AutoBuilder.followPath(path);
    }

    private static Command goToPose_photon_midPose(SwerveSubsystem s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return null;

        try
        {
            // get the endPose from photonvision
            // if it very close, go there directly
            // otherwise, go to the middle pose first, check photovision again, 
            // get the new endPose, then go to the final endPose
            Pose2d startPose = s_Swerve.getPose();
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose,
                endPose
            );
            double lengthToCompare = 1;
            int size = bezierPoints.size();
            var totalDistance = getTotalDistance(bezierPoints);
            if (totalDistance < lengthToCompare || size < 3)
            {
                return new SequentialCommandGroup(
                    goToPose_photon(s_Swerve, endPose),
                    goToPose(s_Swerve, endPose),
                    goToPose(s_Swerve, endPose),
                    goToPose(s_Swerve, endPose)
                );
            }
            else
            {
                double distance = 0; 
                Translation2d midT = bezierPoints.get(0);
                for(int i=1; i<size; i++)
                {
                    distance += bezierPoints.get(i).getDistance(bezierPoints.get(i-1));
                    midT = bezierPoints.get(i);
                    if(totalDistance - distance <= lengthToCompare)
                        break;
                }
                Pose2d midPose = new Pose2d(midT, endPose.getRotation());
                SmartDashboard.putNumber("midPose_x", midPose.getX());
                SmartDashboard.putNumber("midPose_y", midPose.getY());
                SmartDashboard.putNumber("midPose_yaw", midPose.getRotation().getDegrees());
                
                lastMidPose2d = midPose;

                return goToPose_photon(s_Swerve, midPose);
            }
        }
        catch(Exception ex)
        {
            System.out.println("ERROR: " + ex.getMessage());
            return null;
        }
    }

    private static double getTotalDistance(List<Translation2d> waypoints) {
        double totalDistance = 0.0;

        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d currentPoint = waypoints.get(i);
            Translation2d prevPoint = waypoints.get(i - 1);

            double distance = currentPoint.getDistance(prevPoint);
            totalDistance += distance;
        }

        return totalDistance;
    }
    
    public static Command goToPose(SwerveSubsystem s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return new InstantCommand();

        // go to the endPose directly
        double endAngle=0;
        //endAngle = SmartDashboard.getNumber("endAngle", 0);
        Pose2d startPose = s_Swerve.getPose();
        endAngle = endPose.getRotation().getDegrees();

        var lastP = endPose.getTranslation();
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = endPose.getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();
        if (curPose.getDistance(lastP) < 0.05)
        {
            if (Math.abs(curAngle - lastAngle) < 1)
                return new InstantCommand();
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3, 2.5, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(endAngle)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = false;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        
        return AutoBuilder.followPath(path);
    }

    private static Command goToPoseCommand_preplanned(SwerveSubsystem s_Swerve, String pathName)
    {
        //s_Swerve.resetOdometry(s_Swerve.getPose());
        PathPlannerPath path = PathPlannerPath.fromPathFile(
            pathName
            //"Example Path"
            //"straight line x"
            // "straight line y"
            //"turn 90"
            //"Path Rotation Target"
            //"turn big 90"
            );
        //path.preventFlipping =true;
        var points = path.getAllPathPoints();
        var lastP = points.get(points.size() - 1).position;
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = path.getGoalEndState().getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();

        lastEndPose2d = new Pose2d(lastP, path.getGoalEndState().getRotation());

        if (curPose.getDistance(lastP) < 0.1)
        {
            if (Math.abs(curAngle - lastAngle) < 2)
                return new InstantCommand();
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public static void Move_X(SwerveSubsystem s_Swerve, double distance)
    {
        Pose2d curPose = s_Swerve.getPose();
        Pose2d endPose = new Pose2d(
            new Translation2d(
                curPose.getX() + distance,
                curPose.getY()
            ),
            curPose.getRotation()
        );
        goToPose(s_Swerve, endPose);
    }

    public static void Move_Y(SwerveSubsystem s_Swerve, double distance)
    {
        Pose2d curPose = s_Swerve.getPose();
        Pose2d endPose = new Pose2d(
            new Translation2d(
                curPose.getX(),
                curPose.getY() + distance
            ),
            curPose.getRotation()
        );
        goToPose(s_Swerve, endPose);
    }

    public static Translation2d rotateTranslation(Translation2d translation, Rotation2d rotation)
    {
        double cosTheta = Math.cos(rotation.getRadians());
        double sinTheta = Math.sin(rotation.getRadians());
        double x = translation.getX() * cosTheta - translation.getY() * sinTheta;
        double y = translation.getX() * sinTheta + translation.getY() * cosTheta;
        return new Translation2d(x, y);
    }

    public static Pose2d ConvertFieldPose2d(Translation2d targetTranslation, Pose2d curPose)
    {
        double theta = curPose.getRotation().getRadians();
        Translation2d trans = rotateTranslation(targetTranslation, Rotation2d.fromRadians(theta));

        SmartDashboard.putNumber("mid X: ", trans.getX());
        SmartDashboard.putNumber("mid Y: ", trans.getY());

        Translation2d endTrans = new Translation2d(
                trans.getX() + curPose.getX(),
                curPose.getY() + trans.getY()
        );
        Pose2d endPose = new Pose2d(endTrans, curPose.getRotation());
        return endPose;
    }

    public static Command LookForGamepiece(SwerveSubsystem drivebase)
    {
        boolean bFound = false;
        Command returnCmd = null;
        var targetDetection = new TargetDetection("", TargetDetection.PipeLineType.COLORED_SHAPE);
        RobotMoveTargetParameters params = targetDetection.GetRobotMoveforGamePieceviaEdgeTpu();        
        bFound = params.IsValid;
        while(!bFound)
        {
            for(int i=0; i<60; i++)
            {
                var desiredSpeeds = drivebase.getTargetSpeeds(0, 0, 
                    drivebase.getHeading().minus(Rotation2d.fromDegrees(20)));
        
                // Limit velocity to prevent tippy
                Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
                translation = SwerveMath.limitVelocity(translation, drivebase.getFieldVelocity(), drivebase.getPose(),
                    Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                    drivebase.getSwerveDriveConfiguration());
                // Make the robot move
                drivebase.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
            }
            params = targetDetection.GetRobotMoveforGamePieceviaEdgeTpu();        
            bFound = params.IsValid;
        }
        if (bFound)
        {
            Translation2d trans = params.move;
            Pose2d curPose2d = drivebase.getPose();
            Pose2d endPose = ConvertFieldPose2d(trans, curPose2d);

            SmartDashboard.putNumber("move X: ", trans.getX());
            SmartDashboard.putNumber("move Y: ", trans.getY());
            
            SmartDashboard.putNumber("curPose X: ", curPose2d.getX());
            SmartDashboard.putNumber("curPose Y: ", curPose2d.getY());

            SmartDashboard.putNumber("endPose X: ", endPose.getX());
            SmartDashboard.putNumber("endPose Y: ", endPose.getY());

            returnCmd = PathPlannerHelper.goToPose(drivebase, endPose);
        }

        return returnCmd;
    }

    public static Command GoToGPPose(SwerveSubsystem drivebase, GamePieceSubsystem gamePieceSubsystem)
    {        
        if (gamePieceSubsystem.FoundGP && (gamePieceSubsystem.GPPose != null))
        {
            SmartDashboard.putNumber("Goto Target X", gamePieceSubsystem.GPPose.getX());
            SmartDashboard.putNumber("Goto arget Y", gamePieceSubsystem.GPPose.getY());

            return goToPose(drivebase, gamePieceSubsystem.GPPose);
        }
        else
            return new InstantCommand();
    }
}
