package frc.team7520.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.gamepiece.GamePieceSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class GamePieceLookUp extends Command {
    private final SwerveSubsystem drivebase;
    private final GamePieceSubsystem gamePieceSubsystem = GamePieceSubsystem.getInstance();
    
    public GamePieceLookUp(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
        
        addRequirements(this.gamePieceSubsystem);
    }

    @Override
    public void initialize() {
        Boolean isRed = isRedAlliance();
        Double theta = isRed ? 30.0 : -30.0;

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
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    private boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
