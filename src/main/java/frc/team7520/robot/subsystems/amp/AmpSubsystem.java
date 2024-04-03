// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.amp;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.AmpConstants;

public class AmpSubsystem extends SubsystemBase {

    public CANSparkMax pivot = new CANSparkMax(AmpConstants.CAN_ID, MotorType.kBrushless);

    private RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPID = pivot.getPIDController();

    public Rotation2d desiredPosition = Rotation2d.fromDegrees(AmpConstants.Rest);


//    public DiffEncoder diffedEncoder = new DiffEncoder(pivotAbsEncoder, pivotAbsEncoder);

    private final static AmpSubsystem INSTANCE = new AmpSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static AmpSubsystem getInstance() {
        return INSTANCE;
    }

    /** Creates a new ExampleSubsystem. */
    private AmpSubsystem() {
        this.pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(AmpConstants.degreeConversionFactor);

        pivotPID.setP(AmpConstants.kP);
        pivotPID.setI(AmpConstants.kI);
        pivotPID.setD(AmpConstants.kD);
        pivotPID.setFF(AmpConstants.kFF);

        pivotPID.setSmartMotionMaxVelocity(AmpConstants.SmartMaxVel, AmpConstants.SlotID);
        pivotPID.setSmartMotionMinOutputVelocity(AmpConstants.SmartMinVel, AmpConstants.SlotID);
        pivotPID.setSmartMotionMaxAccel(AmpConstants.SmartAccel, AmpConstants.SlotID);
        pivotPID.setSmartMotionAllowedClosedLoopError(AmpConstants.SmartErr, AmpConstants.SlotID);


        pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);


    }

    public void setPosition(AmpConstants.Position position){
        desiredPosition = position.getPosition();
        pivotPID.setReference(desiredPosition.getDegrees(), ControlType.kSmartMotion);
    }

    public boolean atPosition() {
        double currPos = pivotEncoder.getPosition();
        double targetPos = desiredPosition.getDegrees();
        double error = Math.abs(targetPos - currPos);

        return error < AmpConstants.SmartErr;
    }

    public void setSpeed(double speed) {
        pivot.set(speed);
   }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotEncoder Amp", pivotEncoder.getPosition());
        SmartDashboard.putNumber("DesiredDeg Amp", desiredPosition.getDegrees());
        SmartDashboard.putNumber("DesiredRot Amp", desiredPosition.getRotations());

    }

}
