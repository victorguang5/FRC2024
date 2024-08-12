// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.Constants.AmpConstants;
import frc.team7520.robot.Constants.IntakeConstants;
import frc.team7520.robot.Constants.IntakeConstants.Position;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team7520.robot.auto.AutoIntake;
import frc.team7520.robot.auto.AutoNotePickUp;
import frc.team7520.robot.auto.AutoShoot;
import frc.team7520.robot.auto.ShootSequence;
import frc.team7520.robot.commands.AbsoluteDrive;
import frc.team7520.robot.commands.Climber;
import frc.team7520.robot.commands.Intake;
import frc.team7520.robot.commands.Shooter;

import frc.team7520.robot.commands.Amp;
import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.climber.ClimberSubsystem;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.amp.AmpSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

import java.io.File;
import java.util.function.BooleanSupplier;

//Temporary Imports BY RObin
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import static frc.team7520.robot.subsystems.LED.candle;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
        public NetworkTableInstance inst = NetworkTableInstance.getDefault();
        public NetworkTable table = inst.getTable("noteTable");

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name

        //"Detections" supplies all notes detected, MaxConfObj gives only one
        public StringTopic StrTopic = inst.getStringTopic("/noteTable/Detections");
        public static boolean speakerRoutineActivateShooter = false;


    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/Swerve3Neo"), StrTopic);

    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private final AmpSubsystem ampSubsystem = AmpSubsystem.getInstance();

    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private final LED LEDSubsystem = LED.getInstance();

    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final Intake intake = new Intake(intakeSubsystem,
            operatorController::getRightBumper,
            operatorController::getYButton,
            operatorController::getAButton,
            operatorController::getBButton,
            operatorController::getXButton
        );

        private final Amp amp = new Amp(ampSubsystem,
                operatorController::getPOV);

    public Shooter shooter;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        registerAutos();

        CameraServer.startAutomaticCapture();

        // Configure the trigger bindings
        configureBindings();

        // Left joystick is the angle of the robot
        AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                () -> MathUtil.applyDeadband(-driverController.getLeftY()/1.75,
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftX()/1.75,
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY(),
                driverController::getRightBumper,
                driverController::getLeftBumper,
                () -> false,
                StrTopic
        );

         shooter = new Shooter(shooterSubsystem,
                operatorController::getLeftTriggerAxis,
                operatorController::getRightTriggerAxis,
                operatorController::getLeftBumper
        );


        Climber climber = new Climber(climberSubsystem,
                () -> false,
                () -> false,
                operatorController::getStartButton,
                operatorController::getRightY,
                operatorController::getLeftY,
                operatorController::getBackButton
        );
        // Intake intake = new Intake(intakeSubsystem,
        //         operatorController::getRightBumper,
        //         operatorController::getYButton,
        //         operatorController::getAButton,
        //         operatorController::getBButton,
        //         operatorController::getXButton,
        //         intakeSubsystem::getSwitchVal
        // );

        // Old drive method
        // like in video games
        // Easier to learn, harder to control
        // Not tested not used
        TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRawAxis(2), () -> true);

        drivebase.setDefaultCommand(closedAbsoluteDrive);
        ampSubsystem.setDefaultCommand(amp);
        shooterSubsystem.setDefaultCommand(shooter);
        intakeSubsystem.setDefaultCommand(intake);
        climberSubsystem.setDefaultCommand(climber);
        LEDSubsystem.setDefaultCommand(LEDSubsystem.idle());

        candle.animate(LEDSubsystem.idleAnimation);
    }

    private void registerAutos(){

        registerNamedCommands();

        autoChooser.setDefaultOption("Safe auto", drivebase.getPPAutoCommand("safe", true));
        autoChooser.addOption("Amp", drivebase.getPPAutoCommand("Amp", true));
        autoChooser.addOption("Test", drivebase.getPPAutoCommand("test", true));
        autoChooser.addOption("BotToCentBot", drivebase.getPPAutoCommand("BotToCentBot", true));
        autoChooser.addOption("MidToCentTop", drivebase.getPPAutoCommand("MidToCentTop", true));
        autoChooser.addOption("TopToCentTop", drivebase.getPPAutoCommand("TopToCentTop", true));
        autoChooser.addOption("2Note", drivebase.getPPAutoCommand("2NoteMid", true));
        autoChooser.addOption("3NoteMid.Note1", drivebase.getPPAutoCommand("3NoteMid.Note1", true));
        autoChooser.addOption("3NoteMid.Note3", drivebase.getPPAutoCommand("3NoteMid.Note3", true));
        autoChooser.addOption("4Note", drivebase.getPPAutoCommand("4Note", true));
        autoChooser.addOption("4Note(StraightReturn)", drivebase.getPPAutoCommand("4Note(StraightReturn)", true));
        autoChooser.addOption("2NoteSpeakerS.Note3", drivebase.getPPAutoCommand("2NoteSpeakerS.Note3", true));
        autoChooser.addOption("3NoteSpeakerC.Note2.SpeakerC.Note5.SpeakerC", drivebase.getPPAutoCommand("3NoteSpeakerC.Note2.SpeakerC.Note5.SpeakerC", true));

        // 1note shoot and Speaker Source side to note8 parking
        autoChooser.addOption("SpeakerS.Note8", drivebase.getPPAutoCommand("SpeakerS.Note8", true));

        // 2note Ampside
        autoChooser.addOption("SpeakerA.Note1.SpeakerA", drivebase.getPPAutoCommand("SpeakerA.Note1.SpeakerA", true));
        // 2note Ampside with move to center
        autoChooser.addOption("SpeakerA.Note1.SpeakerA.Note4", drivebase.getPPAutoCommand("SpeakerA.Note1.SpeakerA.Note4", true));
        autoChooser.addOption("SpeakerS.Note8.SpeakerS", drivebase.getPPAutoCommand("SpeakerS.Note8.SpeakerS", true));
        autoChooser.addOption("4NoteButFaster", drivebase.getPPAutoCommand("4NoteButFaster", true));

        // Troll Auto
        autoChooser.addOption("TrollAuto3NoteFeed", drivebase.getPPAutoCommand("TrollAuto3NoteFeed", true));

 
        SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        NamedCommands.registerCommand("shoot", new ShootSequence());
        NamedCommands.registerCommand("log", new InstantCommand(() -> System.out.println("eeeeeeeeeeeeeeeeeeeeeeeee")));
        NamedCommands.registerCommand("intakeOut", new AutoIntake(Position.INTAKE));
        NamedCommands.registerCommand("intake", new InstantCommand(() -> intakeSubsystem.setSpeed(Position.INTAKE.getSpeed())));
        NamedCommands.registerCommand("stopIntaking", new InstantCommand(() -> intakeSubsystem.setSpeed(0)));
        NamedCommands.registerCommand("intakeIn", new AutoIntake(Position.SHOOT));
        NamedCommands.registerCommand("stopShoot", new AutoShoot(0, false));
        NamedCommands.registerCommand("AutoNotePickUp", new AutoNotePickUp());

    }



    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Zero gyro
        new JoystickButton(driverController, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(drivebase::zeroGyro));
        // X/Lock wheels
        new JoystickButton(driverController, XboxController.Button.kX.value)
                .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock)));

        /* // Item below is not to be used for a drive base using ABSOLUTE COORDINATES on field
        new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(drivebase::resetOdometry));
        */
        

        /* // Items are not working as intended and are overriding the important led indication of having a note, GREEN
        
        new Trigger(() -> intake.currPosition == Position.INTAKE)
                .and(new JoystickButton(operatorController, XboxController.Button.kRightBumper.value))
                .onTrue(new RepeatCommand(LEDSubsystem.intaking()))
                .onFalse(LEDSubsystem.idle());

        new Trigger(() -> intake.currPosition == Position.INTAKE)
                .and(new JoystickButton(operatorController, XboxController.Button.kX.value))
                .whileTrue(new RepeatCommand(LEDSubsystem.intaking()))
                .onFalse(LEDSubsystem.idle());
        */

        new Trigger(intakeSubsystem::getSwitchVal)
                .whileFalse(new RepeatCommand(LEDSubsystem.noteIn()))
                .onTrue(LEDSubsystem.idle());

        
        /* OTF Path simple testing */
        // new JoystickButton(driverController, XboxController.Button.kY.value)
        //         .onTrue(new InstantCommand(() -> {
        //                 var cmd = AutoBuilder.followPath(drivebase.robinPath());
        //                 cmd.schedule();}
        //                 ));

        /* OTF Path with timed event markers */
        // new JoystickButton(driverController, XboxController.Button.kY.value)
        //         .onTrue(new InstantCommand(() -> {
        //                 var cmd = AutoBuilder.followPath(drivebase.seanOTFPath());
        //                 cmd.schedule();}
        //                 ));

        /* OTF Path using sensor feedback */
        new JoystickButton(driverController, XboxController.Button.kY.value).and(intakeSubsystem::getSwitchVal)
                .onTrue(notePickUp());

        //new JoystickButton(driverController, XboxController.Button.kY.value)
        //        .onTrue(automaticShoot());

        new Trigger(() -> speakerRoutineActivateShooter)
                .onTrue(new ParallelCommandGroup(
                        new ShootSequence(),
                        new InstantCommand(() -> {speakerRoutineActivateShooter = false;})
                ));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new AutoShoot(0.7, false))),
                        autoChooser.getSelected()
                ),
                new InstantCommand(() -> shooterSubsystem.setDefaultCommand(shooter))
        ).finallyDo((boolean interupted) -> {
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                drivebase.setGyro(drivebase.getHeading().minus(Rotation2d.fromDegrees(180)));
            }

            shooterSubsystem.setDefaultCommand(shooter);
        });
        
        /* 
        //Robin's 1m Path Test 
        PathPlannerPath path = PathPlannerPath.fromPathFile("OneMeterByRobinTest");
        return AutoBuilder.followPath(path);
        */
        
    }

    public void activateSpeakerRoutine() {
        speakerRoutineActivateShooter = true;
    }

    /**
     * Runs OTF path to note and full intake sequence using sensor in parallel
     * @return the command for autoNotePickUp
     */
    public Command notePickUp() {
        return new InstantCommand(() -> {
                var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(0));
                cmd.schedule();                
        }); 
    }

    /**
     * Runs OTF path to speaker position and shoot sequence, timed
     * @return
     */
    public Command automaticShoot() {
        
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(1));
                        cmd.schedule();                
                })
        );
                
    }

    public void teleOpInit() {

        shooterSubsystem.setDefaultCommand(shooter);

    }
}
