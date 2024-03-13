package frc.team7520.robot.subsystems.shooter;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;

    private SparkPIDController leftShooterPID;
    private SparkPIDController rightShooterPID;

    private RelativeEncoder leftShooterEncoder;
    private RelativeEncoder rightShooterEncoder;

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(100);

    /**
     * The Singleton instance of this shooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    /**
     * Returns the Singleton instance of this shooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code shooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this shooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        /*    leftShooterMotor = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterPID = leftShooterMotor.getPIDController();
    leftShooterPID.setP(Constants.kShooterP);
    leftShooterPID.setI(Constants.kShooterI);
    leftShooterPID.setD(Constants.kShooterD);
    leftShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    rightShooterPID = rightShooterMotor.getPIDController();
    rightShooterPID.setP(Constants.kShooterP);
    rightShooterPID.setI(Constants.kShooterI);
    rightShooterPID.setD(Constants.kShooterD);
    rightShooterPID.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);*/

        leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.shooterLeftID, CANSparkMax.MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.shooterRightID, CANSparkMax.MotorType.kBrushless);

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        leftShooterPID = leftShooterMotor.getPIDController();
        leftShooterPID.setP(Constants.ShooterConstants.kP);
        leftShooterPID.setI(Constants.ShooterConstants.kI);
        leftShooterPID.setD(Constants.ShooterConstants.kD);
        leftShooterPID.setFF(Constants.ShooterConstants.kFF);

        rightShooterPID = rightShooterMotor.getPIDController();
        rightShooterPID.setP(Constants.ShooterConstants.kP);
        rightShooterPID.setI(Constants.ShooterConstants.kI);
        rightShooterPID.setD(Constants.ShooterConstants.kD);
        rightShooterPID.setFF(Constants.ShooterConstants.kFF);

        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);
    }

    public void setSpeed(double speed, boolean closedLoop) {
        speed = mSpeedLimiter.calculate(speed);

        if (closedLoop) {
            speed *= Constants.ShooterConstants.MAX_RPM;

            leftShooterPID.setReference(speed, CANSparkBase.ControlType.kVelocity);
            rightShooterPID.setReference(speed, CANSparkBase.ControlType.kVelocity);
        } else {
            leftShooterMotor.set(speed);
            rightShooterMotor.set(speed);
        }
    }

    public void stop() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }
}

