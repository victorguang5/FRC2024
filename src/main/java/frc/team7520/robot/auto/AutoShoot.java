package frc.team7520.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;


public class AutoShoot extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final double power;

    private final boolean closedLoop;
    public AutoShoot(double power, boolean closedLoop) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);

        this.power = power;
        this.closedLoop = closedLoop;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        shooterSubsystem.setSpeed(power, closedLoop);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return closedLoop;
        

    }

    @Override
    public void end(boolean interrupted) {

    }
}
