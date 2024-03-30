package frc.team7520.robot.auto;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.PathPlannerHelper;
import frc.team7520.robot.util.RobotMoveTargetParameters;
import frc.team7520.robot.util.TargetDetection;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class PickUpGPSequence extends SequentialCommandGroup {
    static boolean bFound = false;
    static Pose2d gPPose2d = new Pose2d();
    static double x_offset = 0.3;

    public PickUpGPSequence(SwerveSubsystem drivebase) {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new InstantCommand(() -> {

                        LookForGamepiece(drivebase);
                        if (bFound)
                        {
                                SequentialCommandGroup cmds = new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                                                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.35))
                                        ),
                                        PathPlannerHelper.goToPose(drivebase, gPPose2d),
                                        new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                                        new WaitCommand(0.75),
                                        new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))                                      
                                );
                                CommandScheduler.getInstance().schedule(cmds);
                        }
                })
        );
    }

    public static boolean LookForGamepiece(SwerveSubsystem drivebase)
    {
        bFound = false;
        gPPose2d = new Pose2d();

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
            trans = new Translation2d( 
                trans.getX() + x_offset, trans.getY()
            );
            Pose2d curPose2d = drivebase.getPose();
            gPPose2d = PathPlannerHelper.ConvertFieldPose2d(trans, curPose2d);

            SmartDashboard.putNumber("move X:", trans.getX());
            SmartDashboard.putNumber("move Y:", trans.getY());

            SmartDashboard.putNumber("curPose X:", curPose2d.getX());
            SmartDashboard.putNumber("curPose Y:", curPose2d.getY());

            SmartDashboard.putNumber("endPose X:", gPPose2d.getX());
            SmartDashboard.putNumber("endPose Y:", gPPose2d.getY());
        }

        return bFound;
    }
}
