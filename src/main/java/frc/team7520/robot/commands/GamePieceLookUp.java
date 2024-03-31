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
    private BooleanSupplier foundGPSupplier;

    private boolean isDone = false;
    private boolean inProcess = false;
    private int iCount = 0;
    
    public GamePieceLookUp(
        SwerveSubsystem drivebase,
        BooleanSupplier foundGPSupplier ) {
        this.drivebase = drivebase;
        this.foundGPSupplier = foundGPSupplier;
        this.isDone = false;
        
        addRequirements(this.gamePieceSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!isDone)
        {
            if (!inProcess)
            {
                inProcess = true;
                gamePieceSubsystem.LookForGamepiece(drivebase);

                if (!foundGPSupplier.getAsBoolean())
                {
                    if (iCount < 500) // max 10 seconds
                    {
                        iCount++;
                        Boolean isRed = isRedAlliance();
                        Double theta = isRed ? 20.0 : -20.0;

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
                    else
                    {
                        iCount = 0;
                        isDone = true;
                    }
                }
                else
                {
                    SmartDashboard.putBoolean("lookup Target FoundGP", gamePieceSubsystem.FoundGP);
                    SmartDashboard.putNumber("lookup Target X", gamePieceSubsystem.GPPose.getX());
                    SmartDashboard.putNumber("lookup Target Y", gamePieceSubsystem.GPPose.getY());

                    isDone = true;
                }
                inProcess = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
        //return true;
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
