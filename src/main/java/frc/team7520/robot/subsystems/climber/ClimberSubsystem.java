// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;

    private SparkPIDController leftClimberPID;
    private SparkPIDController rightClimberPID;

    private RelativeEncoder leftClimberEncoder;
    private RelativeEncoder rightClimberEncoder;

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1);


    private final static ClimberSubsystem INSTANCE = new ClimberSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ClimberSubsystem getInstance() {
        return INSTANCE;
    }

    /** Creates a new ExampleSubsystem. */
    private ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.climberLeftID, CANSparkMax.MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.climberRightID, CANSparkMax.MotorType.kBrushless);

        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();

        leftClimberPID = leftClimberMotor.getPIDController();
        leftClimberPID.setP(Constants.ClimberConstants.kP);
        leftClimberPID.setI(Constants.ClimberConstants.kI);
        leftClimberPID.setD(Constants.ClimberConstants.kD);
        //leftClimberPID.setFF(Constants.ClimberConstants.kFF);

        rightClimberPID = rightClimberMotor.getPIDController();
        rightClimberPID.setP(Constants.ClimberConstants.kP);
        rightClimberPID.setI(Constants.ClimberConstants.kI);
        rightClimberPID.setD(Constants.ClimberConstants.kD);
        //rightClimberPID.setFF(Constants.ClimberConstants.kFF);

        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();

        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftClimberMotor.setInverted(false);
        rightClimberMotor.setInverted(false);
    }

    public void setPosition(Rotation2d position){
        //DesiredPosition = position;
        //pivotPID.setReference(DesiredPosition.getRotations(), ControlType.kSmartMotion);
    }

    public double getLeftPosition(){
        return leftClimberEncoder.getPosition();
    }

    public double getRightPosition(){
        return rightClimberEncoder.getPosition();
    }

    public void setLeftPosition(double pos){
         leftClimberEncoder.setPosition(pos);
    }

    public void setRightPosition(double pos){
         rightClimberEncoder.setPosition(pos);
    }

    public void setRightArmReference(double pos) {
        rightClimberPID.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public void setLeftArmReference(double pos) {
        leftClimberPID.setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public void setLeftSpeed(double speed){
         leftClimberMotor.set(speed);
    }

    public void setRightSpeed(double speed){
         rightClimberMotor.set(speed);
    }

    public double getLeftSpeed(){
         return leftClimberMotor.get();
    }

    public double getRightSpeed(){
         return rightClimberMotor.get();
    }

    public void setZeroPos() {
        leftClimberEncoder.setPosition(ClimberConstants.maxPosition);
        rightClimberEncoder.setPosition(-ClimberConstants.maxPosition);
        leftClimberPID.setReference(ClimberConstants.maxPosition, CANSparkMax.ControlType.kPosition);
        rightClimberPID.setReference(ClimberConstants.maxPosition, CANSparkMax.ControlType.kPosition);
    }



    public void stop() {
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
        leftClimberPID.setReference(leftClimberEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
        rightClimberPID.setReference(rightClimberEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberL", leftClimberEncoder.getPosition());
        SmartDashboard.putNumber("ClimberR", rightClimberEncoder.getPosition());
    }
}
