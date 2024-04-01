package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Shooter extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier throttleSup;
    private final DoubleSupplier slowThrottleSup;
    private final BooleanSupplier invertSup;

    public Shooter(ShooterSubsystem shooterSubsystem, DoubleSupplier throttleSup, DoubleSupplier slowThrottleSup, BooleanSupplier invertSup) {
        this.shooterSubsystem = shooterSubsystem;
        this.throttleSup = throttleSup;
        this.slowThrottleSup = slowThrottleSup;
        this.invertSup = invertSup;


        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double throttle = Math.max(throttleSup.getAsDouble(), (slowThrottleSup.getAsDouble() * 0.16)) * (invertSup.getAsBoolean() ? -1 : 1) * 1;

        shooterSubsystem.setSpeed(throttle, false);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
