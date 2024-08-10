package frc.team7520.robot.auto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.Swerve;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.RobotContainer;

public class SensorAutoIntake extends Command {
    
//    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    //private final Constants.IntakeConstants.Position desiredPos;

    public SensorAutoIntake() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        // desiredPos = Constants.IntakeConstants.Position.INTAKE;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPosition(Constants.IntakeConstants.Position.INTAKE);
        intakeSubsystem.setSpeed(Constants.IntakeConstants.Position.INTAKE.getSpeed());
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !intakeSubsystem.getSwitchVal();
    }

    @Override
    public void end(boolean interrupted) {
        //new WaitCommand(0.5);
        //intakeSubsystem.setPosition(Constants.IntakeConstants.Position.SHOOT);
        //intakeSubsystem.setSpeed(0);
    }
}
