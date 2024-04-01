package frc.team7520.robot.subsystems.gamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.PathPlannerHelper;
import frc.team7520.robot.util.RobotMoveTargetParameters;
import frc.team7520.robot.util.TargetDetection;

public class GamePieceSubsystem extends SubsystemBase {
    private static GamePieceSubsystem INSTANCE = new GamePieceSubsystem();
    private final static TargetDetection targetDetection = new TargetDetection("", TargetDetection.PipeLineType.COLORED_SHAPE);
    
    public boolean FoundGP = false;
    public Pose2d GPPose = null;

    @SuppressWarnings("WeakerAccess")
    public static GamePieceSubsystem getInstance() {
        return INSTANCE;
    }

    private GamePieceSubsystem() {
        FoundGP = false;
        GPPose = new Pose2d();
    }

    public boolean LookForGamepiece(SwerveSubsystem drivebase)
    {
        this.FoundGP = false;
        this.GPPose = null;

        RobotMoveTargetParameters params = targetDetection.GetRobotMoveforGamePieceviaEdgeTpu();        
        boolean bFound = params.IsValid;
        if (bFound)
        {
            Translation2d gpTranslation = params.move;
            double distance = gpTranslation.getDistance(new Translation2d(0,0));

            if (distance > 3) // if it is too far, skip
            {
                bFound = false;
            }
            else
            {
                double x_offset = 0.15; // add more distance to make sure it reach the game piece
                gpTranslation = new Translation2d( 
                    gpTranslation.getX() + x_offset, gpTranslation.getY()
                );
                Pose2d curPose2d = drivebase.getPose();
                this.GPPose = PathPlannerHelper.ConvertFieldPose2d(gpTranslation, curPose2d);
                this.FoundGP = true;

                SmartDashboard.putNumber("move Target X", GPPose.getX());
                SmartDashboard.putNumber("move Target Y", GPPose.getY());
                SmartDashboard.putBoolean("move FoundGP", FoundGP);
            }
        }

        return bFound;
    }

    public Pose2d GetGPose()
    {
        return this.GPPose;
    }

    public boolean GetFoundGP()
    {
        return this.FoundGP;
    }
}
