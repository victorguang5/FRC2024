package frc.team7520.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.commands.GamePieceLookUp;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

public class PickUpGPSequence extends SequentialCommandGroup { 
    public PickUpGPSequence(SwerveSubsystem drivebase) {
        super(
            new GamePieceLookUp(drivebase)
        );
    }
}
