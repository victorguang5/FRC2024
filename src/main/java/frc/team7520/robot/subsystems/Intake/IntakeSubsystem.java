// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.Intake;

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
import frc.team7520.robot.Constants.IntakeConstants;
import frc.team7520.robot.util.DiffEncoder;

public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax pivot = new CANSparkMax(IntakeConstants.PivotConstants.CAN_ID, MotorType.kBrushless);
    public CANSparkMax wheels = new CANSparkMax(IntakeConstants.WheelConstants.CAN_ID, MotorType.kBrushless);

    private RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPID = pivot.getPIDController();
    private final SparkPIDController wheelsPID = wheels.getPIDController();

    public final DigitalInput input = new DigitalInput(0);

    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5);

    public Rotation2d DesiredPosition = Rotation2d.fromDegrees(IntakeConstants.PivotConstants.Shoot);

    // rev through bore encoders
    public DutyCycleEncoder wheelAbsEncoder = new DutyCycleEncoder(2);
    public DutyCycleEncoder pivotAbsEncoder = new DutyCycleEncoder(1);

//    public DiffEncoder diffedEncoder = new DiffEncoder(pivotAbsEncoder, pivotAbsEncoder);

    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    public static boolean AutoMode = false;    

    public Constants.Position currPosition = Constants.Position.SHOOT;

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    /** Creates a new ExampleSubsystem. */
    private IntakeSubsystem() {
        this.pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(IntakeConstants.PivotConstants.degreeConversionFactor);

        pivotPID.setP(IntakeConstants.PivotConstants.kP);
        pivotPID.setI(IntakeConstants.PivotConstants.kI);
        pivotPID.setD(IntakeConstants.PivotConstants.kD);
        pivotPID.setFF(IntakeConstants.PivotConstants.kFF);

        pivotPID.setSmartMotionMaxVelocity(IntakeConstants.PivotConstants.SmartMaxVel, IntakeConstants.PivotConstants.SlotID);
        pivotPID.setSmartMotionMinOutputVelocity(IntakeConstants.PivotConstants.SmartMinVel, IntakeConstants.PivotConstants.SlotID);
        pivotPID.setSmartMotionMaxAccel(IntakeConstants.PivotConstants.SmartAccel, IntakeConstants.PivotConstants.SlotID);
        pivotPID.setSmartMotionAllowedClosedLoopError(IntakeConstants.PivotConstants.SmartErr, IntakeConstants.PivotConstants.SlotID);


        pivot.setIdleMode(CANSparkMax.IdleMode.kCoast);

//        pivotAbsEncoder.setDistancePerRotation(360);
        pivotAbsEncoder.setPositionOffset(0.159);

        // Wheels
        wheels.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        wheels.setInverted(true);

        wheelsPID.setP(IntakeConstants.WheelConstants.kP);
        wheelsPID.setI(IntakeConstants.WheelConstants.kI);
        wheelsPID.setD(IntakeConstants.WheelConstants.kD);
        wheelsPID.setFF(IntakeConstants.WheelConstants.kFF);

//        wheelAbsEncoder.setDistancePerRotation(360);


    }

    public void setPosition(Rotation2d position){
        DesiredPosition = position;
        pivotPID.setReference(DesiredPosition.getDegrees(), ControlType.kSmartMotion);
    }

//    public Command Shoot() {
//        return runOnce(
//                () -> {
//                    setRotation(Rotation2d.fromDegrees(IntakeConstants.PivotConstants.Shoot));
//                }).andThen(MoveIntake());
//    }
//
//    public Command Amp() {
//        return runOnce(
//                () -> {
//                    setRotation(Rotation2d.fromDegrees(IntakeConstants.PivotConstants.Amp));
//                }).andThen(MoveIntake());
//    }
//
//    public Command Intake() {
//        return runOnce(
//                () -> {
//                    setRotation(Rotation2d.fromDegrees(IntakeConstants.PivotConstants.Floor));
//                }).andThen(MoveIntake());
//    }
//
//    public Command Manual(DoubleSupplier pivot) {
//        return runOnce(
//                () -> {
//                    double pivotVal = pivot.getAsDouble();
//
//                    if(Math.abs(pivotVal) > 0.05) {
//                        DesiredPosition = DesiredPosition.plus(Rotation2d.fromDegrees(1 * pivotVal));
//                    }
//                }).andThen(MoveIntake());
//    }

//    public void moveIntake() {
//        pivotPID.setReference(DesiredPosition.getRotations(), ControlType.kSmartMotion);
//    }

    public void stop() {

        wheels.set(0);

    }

    public void setAutoSpeed(double speed) {
        setSpeed(speed, false);
        AutoMode = true;
    }

    public void setAutoSpeed(double speed, boolean closedLoop) {
        SmartDashboard.putBoolean("setAutoSpeed:", true); 
        setSpeed(speed, closedLoop);
        //AutoMode = true;
    }

    public void turnOffAutoMode() {
        AutoMode = false;
        setSpeed(0, false);
    }

    public void setSpeed(double speed) {
        //if (!AutoMode)
            setSpeed(speed, false);
    }

    public void setSpeed(double speed, boolean closedLoop) {
        //if (AutoMode) return;
        speed = slewRateLimiter.calculate(speed);

        if(closedLoop) {
            speed *= IntakeConstants.WheelConstants.MAX_RPM;

            if (speed == 0) {
                wheels.set(0);
            } else {
                wheelsPID.setReference(speed, ControlType.kVelocity);
            }
        } else {
            wheels.set(speed);
        }
    }

    public boolean getSwitchVal() {
        return input.get();
    }

    public double getDiffedEncoder(){
        return pivotAbsEncoder.get() - wheelAbsEncoder.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake Switch", input.get()); 
        SmartDashboard.putNumber("pivotEncoder", pivotEncoder.getPosition());
        SmartDashboard.putNumber("DesiredDeg", DesiredPosition.getDegrees());
        SmartDashboard.putNumber("DesiredRot", DesiredPosition.getRotations());
        SmartDashboard.putNumber("diffedEncoder", getDiffedEncoder());
        SmartDashboard.putNumber("PivotAbsEncoder", pivotAbsEncoder.get());
        SmartDashboard.putNumber("wheelsAbsEncoder", wheelAbsEncoder.get());
        /* 
        if(AutoMode == true && input.get() == false)
        {
            SmartDashboard.putBoolean("setPosition Shoot", true);   
            setPosition(Rotation2d.fromDegrees(Constants.IntakeConstants.PivotConstants.Shoot));
            currPosition = Constants.Position.SHOOT;
            setSpeed(0);
            AutoMode = false;
        }*/
    }
}
 