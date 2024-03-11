package frc.team7520.robot.util;

// 3rd Party Package
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
// WPILIB package
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Team 7520 package
import frc.team7520.robot.Constants;
import frc.team7520.robot.commands.Intake;
import frc.team7520.robot.subsystems.Intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.team7520.robot.Robot;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RoutePlanner{
    SendableChooser<Command> m_pathChooser;
    SendableChooser<Command> m_autoChooser;

    public RoutePlanner(SendableChooser<Command> pathChooser, SendableChooser<Command> autoChooser)
    {
        m_pathChooser = pathChooser;
        m_autoChooser = autoChooser;

    }
    public void ConfigureAutoPathProfile()
    {
        // Pickup one game piece from position 1 route
        /*
        new PathPlannerAuto("AutoTest");
        new PathPlannerAuto("BlueShotOnSite");
        new PathPlannerAuto("BlueShotOnSiteLeft");
        new PathPlannerAuto("BlueShotOnSiteRight");
        new PathPlannerAuto("RedShotOnSite");
         */
        new PathPlannerAuto("MiddleSideMove");
        new PathPlannerAuto("RedLeftSideTurn");
        new PathPlannerAuto("RedRightSideTurn");
        new PathPlannerAuto("BlueLeftSideTurn");
        new PathPlannerAuto("BlueRightSideTurn");
        new PathPlannerAuto("TestAuto");

    }
    public void ConfigureManualPathProfile()
    {
        // For test and validation purpose.
        // Robot has to path the following test before it could proceed any path planner task
        m_pathChooser.setDefaultOption("X+1m",AutoBuilder.followPath(PathPlannerPath.fromPathFile("X+1m")));
        m_pathChooser.addOption("X-1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("X-1m")));
        m_pathChooser.addOption("Y+1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Y+1m")));
        m_pathChooser.addOption("Y-1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Y-1m")));
        m_pathChooser.addOption("XY+1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("XY+1m")));
        m_pathChooser.addOption("Turn+90", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Turn+90")));
        m_pathChooser.addOption("Turn-90", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Turn-90")));
        m_pathChooser.addOption("DriveCircle", AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveCircle")));
    }

    public void ConfigureAutoPathCommand(SwerveSubsystem drivebase, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem)
    {
                NamedCommands.registerCommand(
                "ResetOdometer", 
                new InstantCommand(()->{                        
                        drivebase.resetOdometry(new Pose2d());
                        Pose2d curPose = drivebase.getPose();
                        SmartDashboard.putNumber("Odometer x:", curPose.getX());
                        SmartDashboard.putNumber("Odometer y:", curPose.getY());
                        SmartDashboard.putNumber("Odometer angle:", curPose.getRotation().getDegrees());

                }));

        NamedCommands.registerCommand(
                "MyTestCommand", 
                new InstantCommand(()->{                        
                        intakeSubsystem.setPosition(
                                Rotation2d.fromDegrees(
                                        Constants.IntakeConstants.PivotConstants.Intake
                                )
                        );
                        intakeSubsystem.currPosition = Constants.Position.INTAKE;
                        intakeSubsystem.setAutoSpeed(-0.35, false);                          
                }));

        NamedCommands.registerCommand(
                "BackwardX1M", 
                new InstantCommand(()->{
                        PathPlannerHelper.Move_X(drivebase, -1.5);
                }));

        NamedCommands.registerCommand(
                "ForwardX1M", 
                new InstantCommand(()->{
                        PathPlannerHelper.Move_X(drivebase, 1.5);
                }));

        NamedCommands.registerCommand(
                "RedLeftSideTurn", 
                new InstantCommand(()->{
                        Pose2d curPose = drivebase.getPose();
                        Pose2d endPose = new Pose2d(
                                new Translation2d(-1.3, -2),
                                Rotation2d.fromDegrees(0)
                        );
                        var cmd = PathPlannerHelper.goToPose(drivebase, endPose);
                        CommandScheduler.getInstance().schedule(cmd);
                }));

        NamedCommands.registerCommand(
                "RedRightSideTurn", 
                new InstantCommand(()->{
                        Pose2d curPose = drivebase.getPose();
                        Pose2d endPose = new Pose2d(
                                new Translation2d(-2, 1),
                                Rotation2d.fromDegrees(0)
                        );
                        var cmd = PathPlannerHelper.goToPose(drivebase, endPose);
                        CommandScheduler.getInstance().schedule(cmd);
                }));



        NamedCommands.registerCommand(
                "BlueRightSideTurn", 
                new InstantCommand(()->{
                        Pose2d curPose = drivebase.getPose();
                        Pose2d endPose = new Pose2d(
                                new Translation2d(-1.3, 2),
                                Rotation2d.fromDegrees(0)
                        );
                        var cmd = PathPlannerHelper.goToPose(drivebase, endPose);
                        CommandScheduler.getInstance().schedule(cmd);
                }));

        NamedCommands.registerCommand(
                "BlueLeftSideTurn", 
                new InstantCommand(()->{
                        Pose2d curPose = drivebase.getPose();
                        Pose2d endPose = new Pose2d(
                                new Translation2d(-2, -1),
                                Rotation2d.fromDegrees(0)
                        );
                        var cmd = PathPlannerHelper.goToPose(drivebase, endPose);
                        CommandScheduler.getInstance().schedule(cmd);
                }));

        NamedCommands.registerCommand(
                "RightSideTurn", 
                new InstantCommand(()->{
                        Pose2d curPose = drivebase.getPose();
                        Pose2d endPose = new Pose2d(
                                new Translation2d(-0.7, 1.5),
                                Rotation2d.fromDegrees(0)
                        );
                        var cmd = PathPlannerHelper.goToPose(drivebase, endPose);
                        CommandScheduler.getInstance().schedule(cmd);
                }));


        NamedCommands.registerCommand(
                "ShootOnSite", 
                new InstantCommand(()->{
                        shooterSubsystem.setSpeed(1, false);
                        try {
                                Thread.sleep(1000);
                        } catch (InterruptedException e) {
                        }
                        intakeSubsystem.setAutoSpeed(0.35, false);
                        try {
                                Thread.sleep(1000);
                        } catch (InterruptedException e) {
                        }
                        intakeSubsystem.setSpeed(0, false);
                        shooterSubsystem.setSpeed(0, false);
                        intakeSubsystem.AutoMode = false;
                }));


        NamedCommands.registerCommand(
                "IntakeDown", 
                new InstantCommand(()->{
                        intakeSubsystem.setPosition(Rotation2d.fromDegrees(Constants.IntakeConstants.PivotConstants.Intake));
                        intakeSubsystem.setAutoSpeed(-0.35, false);
                }));


        NamedCommands.registerCommand(
                "IntakeSpeedStop", 
                new InstantCommand(()->{
                        intakeSubsystem.setSpeed(0, false);
                }));



        NamedCommands.registerCommand(
                "IntakeUp", 
                new InstantCommand(()->{
                        intakeSubsystem.setPosition(Rotation2d.fromDegrees(Constants.IntakeConstants.PivotConstants.Shoot));
                }));

        NamedCommands.registerCommand(
                "PhotonResetPose", 
                new InstantCommand(()->{
                        var photonPose = drivebase.GetPhotonvisionPose2d();
                        SmartDashboard.putBoolean("Photon found", (photonPose != null));
                        if (photonPose != null)
                        {
                            drivebase.resetOdometry(photonPose); 
                        }
                }));

        NamedCommands.registerCommand(
                "SetPosition_Intake", 
                new InstantCommand(()->{
                        SmartDashboard.putBoolean("SetPositio start", true);
                        intakeSubsystem.setPosition(
                                Rotation2d.fromDegrees(
                                        Constants.IntakeConstants.PivotConstants.Intake
                                )
                        );     
                        SmartDashboard.putBoolean("SetPositio end", true);               
                }));

        NamedCommands.registerCommand(
                "SetPosition_Shoot", 
                new InstantCommand(()->{
                        intakeSubsystem.setPosition(
                                Rotation2d.fromDegrees(
                                        Constants.IntakeConstants.PivotConstants.Shoot
                                )
                        );                    
                }));

        NamedCommands.registerCommand(
                "SetPosition_Amp", 
                new InstantCommand(()->{
                        intakeSubsystem.setPosition(
                                Rotation2d.fromDegrees(
                                        Constants.IntakeConstants.PivotConstants.Amp
                                )
                        );                    
                }));

        NamedCommands.registerCommand(
                "SetSpeed_Intake", 
                new InstantCommand(()->{
                        intakeSubsystem.setAutoSpeed(-0.35, false);
                }));

        NamedCommands.registerCommand(
                "SetSpeed_Shoot", 
                new InstantCommand(()->{
                        intakeSubsystem.setAutoSpeed(0.35, false);
                }));

        NamedCommands.registerCommand(
                "SetSpeed_Amp", 
                new InstantCommand(()->{
                        intakeSubsystem.setAutoSpeed(0.525, false);
                }));
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        NamedCommands.registerCommand("Shoot", new WaitCommand(1));
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public void runPath()
    {
      Command path = getPathPlanerRoute();
      if(path != null)
      {
        path.schedule();
      }
    }

    public Command getPathPlanerRoute()
    {
        return m_pathChooser.getSelected();
    }
}