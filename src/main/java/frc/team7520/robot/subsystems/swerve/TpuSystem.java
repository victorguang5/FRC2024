/* ------------------------------------------------------------------------------------------------------------------------------------
 * TpuSystem
 * Robin Yan
 * 08/02/2024
 * A class used to organize the information recieved from the NetworkTables published by the Raspberry Pi + TPU.
 * The TpuSystem object requires a StringTopic, a topic found under the noteTable table, to read and decode.
 * This class analyzes the string and isolates key information about a detected note.
 * Using the information and some math, it calculates an estimated distance to the note in terms of X and Y (following FRC coordinates)
 * ------------------------------------------------------------------------------------------------------------------------------------ */

package frc.team7520.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import frc.team7520.robot.Constants;

import java.io.File;
import java.util.List;

public class TpuSystem {
  // the subscriber is an instance variable so its lifetime matches that of the class
  final StringSubscriber NOTE_INFO;
  private double xPos, yPos, height, width, confidence, area = -1;

  /*
   * Terminology being used here:
   * Bottom - very bottom of the screen/stream being processed. Angle is 47.2° from horizontal
   * Top - very top of the stream whose angle is near 0° to the horizontal, parallel to the ground
   * Center - refered to both in terms right to left and top to bottom. In terms of top to bottom, center is located 23.2° from horizontal
   */
  final private double SCREEN_WIDTH  = 640; // A stream is constructed were (0,0) is the top left corner, and (640,480) is the bottom right
  final private double SCREEN_HEIGHT = 480;
  final private double X_DPP = 58 / SCREEN_WIDTH; // Degrees per pixel. Measured by 29° from center of screen to side
  final private double Y_DPP = 0.1; // Degrees per pixel. Measured as bottom being 47.2° from the top horizontal. Top angle is therefore 0.8° above horiztonal
  final private double CAM_HEIGHT = 0.6858; // In meters off the floor, AKA 27 inches. We assume the camera is located directly above robot center
  final private double DISTANCE_AT_BOTTOM = 0.508; // In meters forward of center of robot, AKA 20 inches
  final private double X_CENTER = SCREEN_WIDTH/2; // 320 pixel from the left side
  private double relativeXDistance = 0; // Position of a note estimated from data receieved from NetworkTables. Follows FRC coordinates system
  private double relativeYDistance = 0; // In meters

  final private double YDISTANCE_OFFSET = 0.15; // In meters to the left. What is actually calculated as relativeYDistance is 0.15m off physical measurements
  final private double XDISTANCE_REDUCTION = 0.4; // To prevent robot from running over note's location, stopping it in front of note in position for pick up

  private Translation2d noteLocation;

  /**
   * 
   * @param detection the StringTopic that gives a details of ONE note. Does not currently support multiple note detection.
   */
  public TpuSystem(StringTopic detection) {
    NOTE_INFO = detection.subscribe("[{'x':None,'y':None,'w':None,'h':None,'conf':0}]");
  }

  /**
   * Periodically checks if a note is detected
   */
  public void periodic() {
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    String val = NOTE_INFO.get();

    boolean found = translateInfo(val);
    if (found) {
      area = width*height;
      noteLocation = trigonemtricCalculatedDistance();
    } else {
      xPos = -1;
      yPos = -1;
      area = -1;
      confidence = -1;
      noteLocation = new Translation2d(0,0);
    }

    SmartDashboard.putString("Coordinates", xPos + ", " + yPos);
    SmartDashboard.putNumber("Area", area);
    SmartDashboard.putNumber("Note C", confidence);
  }

  /**
   * Closes the subscriber, though not always necessary needed
   */
  public void close() {
    // stop subscribing
    NOTE_INFO.close();
  }

  /**
   * Reads the value suscribed off the NetworkTables for note detection by breaking a larger string into isolated values. Currently not able to read multiple note detection.
   * @param value
   * @return
   */
  private boolean translateInfo(String value) {
    try {
      if (value.charAt(6) == 'N') {
        return false;
      }
        boolean firstInd = false;
        int[] indexes = new int[10]; 
        int count = 0;

        for (int i = 0; i < value.length(); i++) {
            char c = value.charAt(i);
            if (c >= '0' && c <= '9' || c == '.') {
                if (!firstInd) {
                    firstInd = true;
                    indexes[count] = i;
                    count++;
                }
            } else {
                if (firstInd) {
                    firstInd = false;
                    indexes[count] = i;
                    count++;
                }
            }
        }

        xPos = Double.parseDouble(value.substring(indexes[0], indexes[1]));
        yPos = Double.parseDouble(value.substring(indexes[2], indexes[3]));
        height = Double.parseDouble(value.substring(indexes[4], indexes[5]));
        width = Double.parseDouble(value.substring(indexes[6], indexes[7]));
        confidence = Double.parseDouble(value.substring(indexes[8], indexes[9]));
        return true;

    } catch (NumberFormatException e) {
        System.out.println("NUMBER FORMAT EXCEPTION");
        return false;
    } catch (IndexOutOfBoundsException e) {
      System.out.println("INDEX OUT OF BOUNDS");
      return false;
    }
  }

  /**
   * Uses found measurements and trigonometry to estimate the location of a note relative to the robot center
   * @return a Translation2d of the note detected
   */
  private Translation2d trigonemtricCalculatedDistance() {
    relativeXDistance = CAM_HEIGHT/(Math.tan(Math.toRadians(yPos*Y_DPP)));
    relativeYDistance = Math.hypot(relativeXDistance, CAM_HEIGHT)*Math.tan(Math.toRadians((X_CENTER-xPos)*(X_DPP)));
    return new Translation2d(relativeXDistance - XDISTANCE_REDUCTION, relativeYDistance + YDISTANCE_OFFSET);
  }

  public Translation2d getNoteLocation() {
    if (noteLocation == null) {
      return new Translation2d(0,0);
    }
    return noteLocation;
  }
}