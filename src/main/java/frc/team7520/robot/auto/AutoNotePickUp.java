package frc.team7520.robot.auto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.Swerve;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.RobotContainer;

public class AutoNotePickUp extends SequentialCommandGroup {

    //private final Constants.IntakeConstants.Position desiredPos;

    public AutoNotePickUp() {
        super(
            new SensorAutoIntake(),
            new WaitCommand(0.2),
            new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0)),
            new AutoIntake(Constants.IntakeConstants.Position.SHOOT)
        );
        
    }
}