package frc.team7520.robot.auto;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;
import javax.lang.model.util.SimpleElementVisitor14;

import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ClimberConstants;
import frc.team7520.robot.subsystems.climber.ClimberSubsystem;

    
public class AutoClimber extends Command {
    private final ClimberSubsystem subsystem;
    private BooleanSupplier bActionButtonClicked;
    private double climberSpeed = 0.8;

    private double maxLeftOutputCurrent = 0;
    private double minLeftOutputCurrent = 100;

    private double maxRightOutputCurrent = 0;
    private double minRightOutputCurrent = 100;

    private double leftOutputCurrentCap = 150; // 77.21875
    private double rightOutputCurrentCap = 150; // 92.8175

    enum State {
        NOTHING,
        EXTEND,
        RETRACT
    }

    private State curState = State.NOTHING;
    private State prevState = State.RETRACT;


    public AutoClimber(ClimberSubsystem climberSubsystem, BooleanSupplier actionButtonClicked) {
        this.subsystem = climberSubsystem;
        this.bActionButtonClicked = actionButtonClicked;
        this.curState = State.NOTHING;
        this.prevState = State.RETRACT;
        subsystem.setZeroPos();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

int clickCount = 0;

    @Override
    public void execute() {
        if (bActionButtonClicked.getAsBoolean()) {
            clickCount++;
            SmartDashboard.putNumber("clickCount", clickCount);

            if (this.curState == State.NOTHING)
            {
                if (this.prevState == State.EXTEND)
                    this.curState = State.RETRACT;
                else
                    this.curState = State.EXTEND;
            }
            else if (this.curState == State.EXTEND)
                this.curState = State.RETRACT;
            else
                this.curState = State.EXTEND;
        }
        SmartDashboard.putString("Current State", convertState(this.curState));
        SmartDashboard.putString("Previous State", convertState(this.prevState));
        if (this.curState == State.EXTEND)
        {
            Extend();
        }
        else if (this.curState == State.RETRACT)
        {
            Retract();
        }
        else {
            Stop();
        }
    }

    void Stop()
    {
        subsystem.setLeftSpeed(0);
        subsystem.setRightSpeed(0);
        this.curState = State.NOTHING;
    }

    void DetectMaxMinOutputCurrent()
    {        
        double leftOutputCurrent = subsystem.getLeftOutputCurrent();
        double rightOutputCurrent = subsystem.getRightOutputCurrent();

        SmartDashboard.putNumber("Climber Left Output Current", leftOutputCurrent);
        SmartDashboard.putNumber("Climber Right Output Current", rightOutputCurrent);

        if (leftOutputCurrent > maxLeftOutputCurrent) maxLeftOutputCurrent = leftOutputCurrent;
        if (leftOutputCurrent < minLeftOutputCurrent) minLeftOutputCurrent = leftOutputCurrent;
        if (rightOutputCurrent > maxRightOutputCurrent) maxRightOutputCurrent = rightOutputCurrent;
        if (rightOutputCurrent < minRightOutputCurrent) minRightOutputCurrent = rightOutputCurrent;

        SmartDashboard.putNumber("Climber Left Max Output Current", maxLeftOutputCurrent);
        SmartDashboard.putNumber("Climber Left Min Output Current", minLeftOutputCurrent);
        SmartDashboard.putNumber("Climber Right Max Output Current", maxRightOutputCurrent);
        SmartDashboard.putNumber("Climber Right Min Output Current", minRightOutputCurrent);
    }

    void Extend()
    {
        double leftPosition = subsystem.getLeftPosition();
        double rightPosition = subsystem.getRightPosition();

        SmartDashboard.putNumber("Climber Left Position", leftPosition);
        SmartDashboard.putNumber("Climber Right Position", rightPosition);

        DetectMaxMinOutputCurrent();

        subsystem.setLeftArmReference(0);
        subsystem.setRightArmReference(-0);

        Boolean leftStopped = false;
        Boolean rightStopped = false;

        if (leftOutputCurrentCap > leftOutputCurrentCap)
        {
            subsystem.setLeftSpeed(0);
            leftStopped = true;
        }
        else if (leftPosition > 0)
        {
            subsystem.setLeftSpeed(-climberSpeed);
        }
        else
        {
            subsystem.setLeftSpeed(0);
            leftStopped = true;
        }

        if (rightOutputCurrentCap > rightOutputCurrentCap)
        {
            subsystem.setRightSpeed(0);
            rightStopped = true;
        }
        else if (rightPosition < 0)
        {
            subsystem.setRightSpeed(climberSpeed);
        }
        else
        {
            subsystem.setRightSpeed(0);
            rightStopped = true;
        }
        
        if (leftStopped && rightStopped)
        {
            this.curState = State.NOTHING;
            this.prevState = State.EXTEND;
            return;
        }
    }

    void Retract()
    {
        double leftPosition = subsystem.getLeftPosition();
        double rightPosition = subsystem.getRightPosition();

        SmartDashboard.putNumber("Climber Left Position", leftPosition);
        SmartDashboard.putNumber("Climber Right Position", rightPosition);

        DetectMaxMinOutputCurrent();

        subsystem.setLeftArmReference(ClimberConstants.maxPosition);
        subsystem.setRightArmReference(-ClimberConstants.maxPosition);

        Boolean leftStopped = false;
        Boolean rightStopped = false;

        if (leftOutputCurrentCap > leftOutputCurrentCap)
        {
            subsystem.setLeftSpeed(0);
            leftStopped = true;
        }
        else if (leftPosition < ClimberConstants.maxPosition)
        {
            subsystem.setLeftSpeed(climberSpeed);
        }
        else
        {
            subsystem.setLeftSpeed(0);
            leftStopped = true;
        }

        if (rightOutputCurrentCap > rightOutputCurrentCap)
        {
            subsystem.setRightSpeed(0);
            rightStopped = true;
        }
        else if (rightPosition > -ClimberConstants.maxPosition)
        {
            subsystem.setRightSpeed(-climberSpeed);
        }
        else
        {
            subsystem.setRightSpeed(0);
            rightStopped = true;
        } 
        
        if (leftStopped && rightStopped)
        {
            this.curState = State.NOTHING;
            this.prevState = State.RETRACT;
            return;
        }
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

    public String convertState(State state) {
        if (state == State.NOTHING) {
            return "NOTHING";
        } else if (state == State.EXTEND) {
            return "EXTEND";
        } else if (state == State.RETRACT) {
            return "RETRACT";
        }
        return "";
    }
}