// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.amp.AmpSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/** An example command that uses an example subsystem. */
public class Amp extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AmpSubsystem ampSubsystem;
    private final IntSupplier posSup;
    // private final DoubleSupplier bRightManual;

    public Constants.AmpConstants.Position currPosition = Constants.AmpConstants.Position.REST;



    /**
     * Creates a new ExampleCommand.
     *
     * @param ampSubsystem The subsystem used by this command.
     */
    public Amp(AmpSubsystem ampSubsystem, IntSupplier posSup) {
        this.ampSubsystem = ampSubsystem;
        this.posSup = posSup;
        // this.bRightManual = bRightManual;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ampSubsystem);
    }

    public void handlePosition() {
        if (posSup.getAsInt() == 90) {
            ampSubsystem.setPosition(Constants.AmpConstants.Position.REST);
            currPosition = Constants.AmpConstants.Position.REST;
            return;
        }
        if (posSup.getAsInt() == 270) {
            ampSubsystem.setPosition(Constants.AmpConstants.Position.AMP);
            currPosition = Constants.AmpConstants.Position.AMP;
            return;
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        handlePosition();
        System.out.println(posSup.getAsInt());
        double rightSpeed = posSup.getAsInt();
        if (rightSpeed == 0) {
            //System.out.println("90");
            rightSpeed = 0.1;
        } else if (rightSpeed == 180) {
            rightSpeed = -0.1;
            //System.out.println("270");
        } else {
            rightSpeed = 0;
        }
        ampSubsystem.setSpeed(rightSpeed);

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
