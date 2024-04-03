package frc.team7520.robot.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team7520.robot.Constants;
import frc.team7520.robot.auto.AutoIntake;
import frc.team7520.robot.auto.ShootSequence;
import frc.team7520.robot.commands.GamePieceLookUp;
import frc.team7520.robot.subsystems.gamepiece.GamePieceSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class GamePieceHelper {
    private final static GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();
    private final static TargetDetection targetDetection = new TargetDetection("", TargetDetection.PipeLineType.COLORED_SHAPE);

    public static Command PickupGP(SwerveSubsystem drivebase)
    {
        return new InstantCommand(()->{
            GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();
            Boolean isRed = isRedAlliance();
            Double theta = isRed ? 20.0 : -20.0;

            boolean bFound = gamePieceSubsystem.LookForGamepiece(drivebase);
            while (!bFound)
            {
                for(int i=0; i<60; i++)
                {
                    var desiredSpeeds = drivebase.getTargetSpeeds(0, 0, 
                        drivebase.getHeading().minus(Rotation2d.fromDegrees(theta)));
                    
                    // Limit velocity to prevent tippy
                    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
                    translation = SwerveMath.limitVelocity(translation, drivebase.getFieldVelocity(), drivebase.getPose(),
                    Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                        drivebase.getSwerveDriveConfiguration());
                    // Make the robot move
                    drivebase.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
                }
                bFound = gamePieceSubsystem.LookForGamepiece(drivebase);
                //if (bFound) iCount = 10;
            }
            if (bFound && (gamePieceSubsystem.GPPose != null))
            {
                SmartDashboard.putBoolean("lookup Target FoundGP", gamePieceSubsystem.FoundGP);
                SmartDashboard.putNumber("lookup Target X", gamePieceSubsystem.GPPose.getX());
                SmartDashboard.putNumber("lookup Target Y", gamePieceSubsystem.GPPose.getY());
            }
        });
    }

    public static Command GoToGP(SwerveSubsystem drivebase)
    {
        return new InstantCommand(()->{
            SequentialCommandGroup cmd = new SequentialCommandGroup(
                new ParallelCommandGroup(  
                        new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.35))
                    ),
                new ParallelCommandGroup(  
                    PathPlannerHelper.GoToGPPose(
                        drivebase, GamePieceSubsystem.getInstance()),
                    new WaitCommand(4)
                ),

                new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                new WaitCommand(0.75),  
                new InstantCommand(
                    () -> IntakeSubsystem.getInstance().setSpeed(0)
                ),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BackToBlueLine")),
                new ShootSequence(0.5),


                
                new GamePieceLookUp(drivebase),

                new ParallelCommandGroup(  
                    new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                    new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.35))
                ),
                new InstantCommand(()->{
                    GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();
                    Pose2d endPose = gamePieceSubsystem.GPPose;
                    if (endPose != null)
                    {
                        var gpcmd = new ParallelCommandGroup(
                            PathPlannerHelper.goToPose(drivebase, endPose),
                            new WaitCommand(4)
                        ).andThen(
                            new AutoIntake(Constants.IntakeConstants.Position.SHOOT)
                        ).andThen(
                            new WaitCommand(0.75)
                        ).andThen(
                            new InstantCommand(() -> {
                                IntakeSubsystem.getInstance().setSpeed(0);
                            })
                        ).andThen(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("BackToBlueLine"))
                        )
                        .andThen(
                            new ShootSequence(0.5)
                        );
                        CommandScheduler.getInstance().schedule(gpcmd);
                    }
                })
            );
            CommandScheduler.getInstance().schedule(cmd);
        });
    }


    private static boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
