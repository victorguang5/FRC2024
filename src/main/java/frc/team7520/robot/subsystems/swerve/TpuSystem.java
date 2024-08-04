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
  final StringSubscriber noteInfo;
  private double xPos, yPos, height, width, confidence, area = -1;

  public TpuSystem(StringTopic detection) {
    // start subscribing; the return value must be retained.
    // the parameter is the default value if no value is available when get() is called
    noteInfo = detection.subscribe("[{'x':None,'y':None,'w':None,'h':None,'conf':0}]");

    // subscribe options may be specified using PubSubOption. 
    /** Consider polling and queues for consistently tracking the same note, avoid noise and randoms */
    //dblSub = dblTopic.subscribe(0.0, PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(10));

    // subscribeEx provides the options of using a custom type string.
    // Using a custom type string for types other than raw and string is not recommended.
    //dblSub = dblTopic.subscribeEx("double", 0.0);
  }

  /**
   * Periodically checks
   */
  public void periodic() {
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    String val = noteInfo.get();

    boolean found = translateInfo(val);
    if (found) {
      area = width*height;
    } else {
      xPos = -1;
      yPos = -1;
      area = -1;
      confidence = -1;
    }
    // get the most recent value; if no value has been published, returns
    // the passed-in default value
    //double val = dblSub.get(-1.0);

    // subscribers also implement the appropriate Supplier interface, e.g. DoubleSupplier
    //double val = dblSub.getAsDouble();

    // get the most recent value, along with its timestamp
    //TimestampedDouble tsVal = dblSub.getAtomic();

    // read all value changes since the last call to readQueue/readQueueValues
    // readQueue() returns timestamps; readQueueValues() does not.
    //TimestampedDouble[] tsUpdates = dblSub.readQueue();
    //double[] valUpdates = dblSub.readQueueValues();

    SmartDashboard.putString("Coordinates", xPos + ", " + yPos);
    SmartDashboard.putNumber("Area", area);
    SmartDashboard.putNumber("Note C", confidence);
  }

  // often not required in robot code, unless this class doesn't exist for
  // the lifetime of the entire robot program, in which case close() needs to be
  // called to stop subscribing
  public void close() {
    // stop subscribing
    noteInfo.close();
  }

  /**
   * Reads the value suscribed off the NetworkTables for note detection, 
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
}