package frc.team7520.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants;
import frc.team7520.robot.commands.GamePieceLookUp;
import frc.team7520.robot.subsystems.gamepiece.GamePieceSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.PathPlannerHelper;

public class GoToGPSequence extends SequentialCommandGroup { 
    private final static GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();

    public GoToGPSequence(SwerveSubsystem drivebase) {
        super(
            new InstantCommand(()->{
                Pose2d endPose = gamePieceSubsystem.GPPose;
                if (endPose == null) return;

                SequentialCommandGroup cmd = new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                        new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.75))
                    ),
                    PathPlannerHelper.GoToGPPose(drivebase, gamePieceSubsystem),
                    new WaitCommand(1),
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new AutoIntake(Constants.IntakeConstants.Position.SHOOT), 
                            new WaitCommand(0.5),  
                            new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
                        ),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BackToBlueLine"))
                    ),
                    new ShootSequence(1),

                    new GamePieceLookUp(drivebase),
                    new InstantCommand(()->{
                        var gpPose = gamePieceSubsystem.GPPose;
                        if (gpPose == null) return;
                        
                        var gpcmd = new SequentialCommandGroup(
                            /*
                            new ParallelCommandGroup(
                                new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.75))
                            ), */
                            PathPlannerHelper.GoToGPPose(drivebase, gamePieceSubsystem)
                            ,
                            new WaitCommand(1)
                            
                            /*,
                            new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                    new AutoIntake(Constants.IntakeConstants.Position.SHOOT), 
                                    new WaitCommand(0.5),  
                                    new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
                                ),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BackToBlueLine"))
                            ),                            
                            new ShootSequence(1)  
                            */
                        );
                        CommandScheduler.getInstance().schedule(gpcmd);
                    })
                );
                CommandScheduler.getInstance().schedule(cmd);
            })
        );
    }
}
