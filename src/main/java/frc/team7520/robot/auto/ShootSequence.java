package frc.team7520.robot.auto;


import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {

    public ShootSequence() {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                
                new ParallelDeadlineGroup( 
                        new SequentialCommandGroup(
                                new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                                new WaitCommand(0.6),                 
                                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0.5)),
                                new WaitCommand(0.5)
                        ),
                        new AutoShoot(1, false)
                ),
                
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
                //new AutoShoot(0, true)
                
//                new InstantCommand(() -> ShooterSubsystem.getInstance().setSpeed(0, false))
        );
    }
}
