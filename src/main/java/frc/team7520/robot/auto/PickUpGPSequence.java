package frc.team7520.robot.auto;


import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants;
import frc.team7520.robot.commands.GamePieceLookUp;
import frc.team7520.robot.subsystems.gamepiece.GamePieceSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.PathPlannerHelper;
import frc.team7520.robot.util.RobotMoveTargetParameters;
import frc.team7520.robot.util.TargetDetection;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class PickUpGPSequence extends SequentialCommandGroup { 
    private final static GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();

    public PickUpGPSequence(SwerveSubsystem drivebase) {
        super(
            new GamePieceLookUp(drivebase, gamePieceSubsystem::GetFoundGP)
            /* 
            new InstantCommand(
                ()->{
                    gamePieceSubsystem.FoundGP = true;
                    gamePieceSubsystem.GPPose = new Pose2d(
                        2, 0, Rotation2d.fromDegrees(0)
                    );
                }
            )*/
            /* *
            .andThen(
                new InstantCommand(()->{
                    var cmd = PickUpGamePiece(
                        drivebase, 
                        gamePieceSubsystem::GetFoundGP, 
                        gamePieceSubsystem::GetGPose
                    );
                    CommandScheduler.getInstance().schedule(cmd);
                })
            )     
            */       
        );
    }

    private static Command PickUpGamePiece(
        SwerveSubsystem drivebase
        , BooleanSupplier foundGPSupplier
        , Supplier<Pose2d> gpPoseSupplier
    )
    {
        boolean foundGP = foundGPSupplier.getAsBoolean();
        Pose2d GPPose = gpPoseSupplier.get();

        SmartDashboard.putBoolean("Target FoundGP", foundGP);
        if (GPPose != null)
        {
            SmartDashboard.putNumber("Target X", GPPose.getX());
            SmartDashboard.putNumber("Target Y", GPPose.getY());
        }

        if (foundGPSupplier.getAsBoolean())
        {
            SequentialCommandGroup cmds = new SequentialCommandGroup(
                new ParallelCommandGroup(  
                    new AutoIntake(Constants.IntakeConstants.Position.INTAKE),
                    new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(-0.35))
                ),
                //PathPlannerHelper.goToPose(drivebase, gpPoseSupplier.get()),
                new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                new WaitCommand(0.75),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
                /* 
                ,
                    PathPlannerHelper.goToPose(drivebase,
                            new Pose2d(
                                    0.66, 4.41,
                                    Rotation2d.fromDegrees(-58.63)
                            )
                    ),
                    new ShootSequence()
                */
            );
            return cmds;
        }
        else
        {
            return new InstantCommand();
        }
    }
}
