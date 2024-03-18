// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ClimberConstants;
import frc.team7520.robot.subsystems.climber.ClimberSubsystem;
import frc.team7520.robot.subsystems.climber.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;
import com.revrobotics.CANSparkMax;

/** An example command that uses an example subsystem. */
public class Climber extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ClimberSubsystem subsystem;
    private final BooleanSupplier bExtent;
    private final BooleanSupplier bRetract;
    private final BooleanSupplier bStop;
    private final DoubleSupplier bRightManual;
    private final DoubleSupplier bLeftManual;
    private final BooleanSupplier bShift;

    private static final double motorSpeed = 0.9;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double pValue = 0.00001;

    enum State {
        NOTHING,
        EXTEND,
        RETRACT
    }

    State state = State.NOTHING;

    //public Position currPosition = Position.SHOOT;



    /**
     * Creates a new ExampleCommand.
     *
     * @param climberSubsystem The subsystem used by this command.
     */
    public Climber(ClimberSubsystem climberSubsystem, BooleanSupplier retract, BooleanSupplier extent, BooleanSupplier stop, DoubleSupplier rightManual, DoubleSupplier leftManual, BooleanSupplier shift) {
        this.subsystem = climberSubsystem;
        this.bRetract = retract;
        this.bExtent = extent;
        this.bStop = stop;
        this.bRightManual = rightManual;
        this.bLeftManual = leftManual;
        this.bShift = shift;
        subsystem.setLeftPosition(ClimberConstants.maxPosition);
        subsystem.setRightPosition(ClimberConstants.maxPosition);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climberSubsystem);
    }




    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        SmartDashboard.putString("State", convertState(this.state));
        double leftPosition = subsystem.getLeftPosition();
        double rightPosition = subsystem.getRightPosition();

        /*
        m_SparkMaxPIDController.setP(pValue);
        m_SparkMaxPIDController.setI(0);
        m_SparkMaxPIDController.setD(0);
        m_SparkMaxPIDController.setOutputRange(-0.3, 0.3);
        m_OtherSparkMaxPIDController.setP(pValue);
        m_OtherSparkMaxPIDController.setI(0);
        m_OtherSparkMaxPIDController.setD(0);
        m_OtherSparkMaxPIDController.setOutputRange(-0.3, 0.3);
        */



        if (bExtent.getAsBoolean()) {
            this.state = State.EXTEND;
            subsystem.setRightArmReference(0);
            subsystem.setLeftArmReference(0);
        } else if (bRetract.getAsBoolean()) {
            subsystem.setRightArmReference(ClimberConstants.maxPosition);
            subsystem.setLeftArmReference(ClimberConstants.maxPosition);
            this.state = State.RETRACT;
        } else if (bStop.getAsBoolean()) {
            if (bShift.getAsBoolean()) {
                subsystem.setZeroPos();
                this.state = State.NOTHING;
            } else {
                subsystem.stop();
                this.state = State.NOTHING;
            }
        }

        if (this.state == State.NOTHING) {
            double leftSpeed = bLeftManual.getAsDouble();
            double rightSpeed= bRightManual.getAsDouble();
            leftSpeed = Math.abs(leftSpeed) > 0.2 ? leftSpeed : 0;
            rightSpeed = Math.abs(rightSpeed) > 0.2 ? -rightSpeed : 0;
            subsystem.setLeftSpeed(leftSpeed);
            subsystem.setRightSpeed(rightSpeed);
            
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //subsystem.setLeftSpeed(0);
        //subsystem.setRightSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
