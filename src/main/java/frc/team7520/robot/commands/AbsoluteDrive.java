// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private final BooleanSupplier CCWSpin, CWSpin;
    private boolean initRotation = false;

    /**
     * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve            The swerve drivebase subsystem.
     * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
     *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
     * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
     *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
     *                          looking through the driver station glass.
     * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
     *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
     *                          no deadband.  Positive is towards the left wall when looking through the driver station
     *                          glass.
     * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
     *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
     *                          with no deadband. Positive is away from the alliance wall.
     */
    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
                         DoubleSupplier headingVertical) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        this.CCWSpin = () -> false;
        this.CWSpin = () -> false;

        addRequirements(swerve);
    }

    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
                         DoubleSupplier headingVertical, BooleanSupplier CWSpin, BooleanSupplier CCWSpin) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        this.CCWSpin = CCWSpin;
        this.CWSpin = CWSpin;

        addRequirements(swerve);
    }


    @Override
    public void initialize() {
        initRotation = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        ChassisSpeeds desiredSpeeds;

        if (CWSpin.getAsBoolean()) {
            desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), swerve.getHeading().minus(Rotation2d.fromDegrees(20)));
        } else if (CCWSpin.getAsBoolean()) {
            desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), swerve.getHeading().plus(Rotation2d.fromDegrees(20)));
        } else {
            // Get the desired chassis speeds based on a 2 joystick module.
            // Hongtao slow down the speed by half
            if( headingHorizontal.getAsDouble() < 0.1 && headingVertical.getAsDouble() < 0.1)
            {
                desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble()/2, vY.getAsDouble()/2, swerve.getHeading());
            }
            // Get target speed and heading from joystick
            else
            {
                desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble()/2, vY.getAsDouble()/2,
                    headingHorizontal.getAsDouble()/2,
                    headingVertical.getAsDouble());
            }

            SmartDashboard.putNumber("desiredSpeed.headingH", headingHorizontal.getAsDouble());
            SmartDashboard.putNumber("desiredSpeed.headingV", headingVertical.getAsDouble());
            SmartDashboard.putNumber("desiredSpeed.x", desiredSpeeds.vxMetersPerSecond);
            SmartDashboard.putNumber("desiredSpeed.y", desiredSpeeds.vyMetersPerSecond);
            SmartDashboard.putNumber("desiredSpeed.omega", desiredSpeeds.omegaRadiansPerSecond);
            SmartDashboard.putNumber("Robot Yaw",swerve.getHeading().getDegrees());
            SmartDashboard.putNumber("Odometer.X", swerve.getPose().getX());
            SmartDashboard.putNumber("Odometer.Y", swerve.getPose().getY());
            SmartDashboard.putNumber("Odometer.Angle", swerve.getPose().getRotation().getRadians());
            SmartDashboard.putNumber("targetAngle", Math.atan2(headingHorizontal.getAsDouble(),headingVertical.getAsDouble()));
            SmartDashboard.putNumber("Odometer.Angle", swerve.getPose().getRotation().getDegrees());
            
        }

        // Prevent Movement After Auto
        if (initRotation) {
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
                // Get the curretHeading
                Rotation2d firstLoopHeading = swerve.getHeading();

                // Set the Current Heading to the desired Heading
                desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
            }
            //Dont Init Rotation Again
            initRotation = false;
        }

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Stop the robot from spinning after the auto movement
        // If no button press, then robot should stay still
        double omega = (Math.abs(headingHorizontal.getAsDouble()) < 0.1 
        && (Math.abs(headingVertical.getAsDouble()) < 0.1)
        && CCWSpin.getAsBoolean()
        && CWSpin.getAsBoolean())? 0:desiredSpeeds.omegaRadiansPerSecond;
        //double omega = desiredSpeeds.omegaRadiansPerSecond;
        // Make the robot move
       swerve.drive(translation, omega, true);

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
