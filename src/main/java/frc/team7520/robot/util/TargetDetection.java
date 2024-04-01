// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.util;

import java.io.IOException;
import java.util.Optional;

//import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import com.fasterxml.jackson.databind.util.JSONPObject;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team7520.robot.Constants;

//import frc.team7520.robot.Constants;
import java.io.IOException;

/** Add your docs here. */
public class TargetDetection {

    private PhotonCamera camera;
    private final PipeLineType type;
    private boolean IsOpen = false;
    private boolean already_turned_to_face_target = false;
    private String camera_name;
    AprilTagFieldLayout aprilTagFieldLayout;
    boolean Apriltaglayoutload = false;
    private boolean edgeTPUflag = false;

    double image_width = 1280.0;  // pixels
    double image_height = 720.0;  // pixels    
 
    double game_piece_radius = 0.15;
    double robot_half_width = 0.27;
    double intake_offset = 0;//0.4;

    double targetHeightMeters = 0;
    double cameraHeightMeters = 0.65;

    double pixels_per_degree_horizontal;
    double pixels_per_degree_vertical;
    double cameraPitchRadians;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    private class BoundryOffset {
        public double x;
        public double y;
    }

    private class PhotonVisonData {
        public boolean is_vaild;
        // 2D Mode
        public double yaw;
        public double pitch;
        public double area;
        public double skew;
        public double x_dist_2d;
        public double y_dist_2d;
        // 3D Mode
        public int april_tag_id;
        public double x_distance;
        public double y_distance;
        public double z_distance;
        public double x_rotate;
        public double y_rotate;
        public double z_rotate;
        public double angle_rotate;
        public double ambiguity;
    }

    public TargetDetection(String camera_name, PipeLineType type) {
        this.type = type;
        this.camera_name = camera_name;
        if(camera_name == "" && type == PipeLineType.COLORED_SHAPE) {
            edgeTPUflag = true;
            /* if only know diagonal FOV, calculate Horizontal&Vertical via below       
            double diagonalFoV = 1.5707963; //90.0;
            double diagonalAspect = Math.hypot(image_width, image_height);
            double horizontalView = Math.atan(Math.tan(diagonalFoV / 2) * (image_width / diagonalAspect)) * 2;
            double verticalView = Math.atan(Math.tan(diagonalFoV / 2) * (imageHeight / diagonalAspect)) * 2;
            */
            // Horizontal FOV
            double fov_horizontal_degrees = 82.149220;
            pixels_per_degree_horizontal = (image_width / fov_horizontal_degrees);
            
            // Vertical FOV
            double fov_vertical_degrees = 52.233845;        
            pixels_per_degree_vertical = (image_height / fov_vertical_degrees);

            // to know installation angle, need to measure distance with camera,
            // laptop camera: 1280 X 720, vertical move to see the bottom, the distance to camera 
            double measure_bottom_distance = 0.39;
            measure_bottom_distance = SmartDashboard.getNumber("Camera-Bottom-Distance", 0.39);
            System.out.printf("Camera Bottom Distance: %f\n", measure_bottom_distance);
            double install_angle = Math.atan(measure_bottom_distance / cameraHeightMeters);    
            cameraPitchRadians = -(1.5707963 - install_angle -  Math.toRadians(fov_vertical_degrees) + Math.toRadians(fov_vertical_degrees/2));
        } else {
            camera = new PhotonCamera(camera_name);
            if(camera.isConnected()) {
                IsOpen = true;
            } else {
                System.out.printf("Open Camera %s Fail !!!\n", camera_name);
            }
        }
    }

    public RobotMoveTargetParameters GetSwerveTrainMoveParameters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        PhotonVisonData data = GetPVTargetData();
        if(data.is_vaild) {
            double tmp_radian = Math.abs(data.z_rotate);
            double need_turn_radian = (3.1415926 - tmp_radian);
            BoundryOffset offset = GetOffsetBaseOnApriltagID(data.april_tag_id);
            if(tmp_radian > 3.05 && tmp_radian < 3.2) {
             //   System.out.println("Already face to ArpilTag, no need to turn");
                para.turn = Rotation2d.fromRadians(0);                
                para.move = new Translation2d(data.x_distance - offset.x, data.y_distance + offset.y);
               // System.out.printf("move: x:%f, y:%f\n", data.x_distance - offset.x, data.y_distance + offset.y);
            } else {
                para.turn = (data.z_rotate > 0) ? (Rotation2d.fromRadians(-need_turn_radian)) : (Rotation2d.fromRadians(need_turn_radian));
                
                double trz = (3.1415926 - data.z_rotate);
                double mx = (data.x_distance - data.y_distance * Math.tan(trz)) * Math.cos(trz) - offset.x;
                double my = Math.abs(
                                     data.y_distance / Math.cos(trz) + 
                                     Math.sin(trz) * (data.x_distance - data.y_distance * Math.tan(trz))
                                     );
                double h = Math.abs(Math.tan(trz) * data.x_distance);
                
                if(data.z_rotate > 0) {
                    if(data.y_distance >= 0) {
                        para.move = new Translation2d(mx, my + offset.y);
                    //    System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", -need_turn_radian, mx, my, my + offset.y);
                    } else {
                        if(Math.abs(data.y_distance) <= h) {
                            para.move = new Translation2d(mx, my + offset.y);
                       //     System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", -need_turn_radian, mx, my, my + offset.y);
                        } else {
                            para.move = new Translation2d(mx, -my + offset.y);
                       //     System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", -need_turn_radian, mx, -my, -my + offset.y);
                        }
                    }
                } else if (data.z_rotate < 0) {
                    if(data.y_distance <= 0) {
                        para.move = new Translation2d(mx, -my + offset.y);
                    //    System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", need_turn_radian, mx, -my, -my + offset.y);
                    } else {
                        if(Math.abs(data.y_distance) <= h) {
                            para.move = new Translation2d(mx, -my + offset.y);
                       //     System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", need_turn_radian, mx, -my, -my + offset.y);
                        } else {
                            para.move = new Translation2d(mx, my + offset.y);
                         //   System.out.printf("turn: %f, mx:%f, my:%f, y:%f\n", need_turn_radian, mx, my, my + offset.y);
                        }
                    }
                } 

/*
                double y_dist = data.y_distance;
                double z_degree = (3.14159265 - data.z_rotate);
                double x1 = y_dist * (Math.tan(z_degree));
                        
                if(Math.cos(z_degree) == 0) {
                    // normally shouldn't be, but just protect 
                    System.out.println("should not be here, but hit here, means 90 degree, still can see target, think over later");
                    return para;
                }
                double x2 = y_dist / (Math.cos(z_degree));
                double x5 = (data.x_distance - x1);
                double x4 = x5 * (Math.cos(z_degree));
                BoundryOffset offset = GetOffsetBaseOnApriltagID(data.april_tag_id);
                //double x6 = (x4 - Constants.SPEAKER_SUBWOOFER_WIDTH);
                double x6 = (x4 - offset.x);
                double x3 = x5 * (Math.sin(z_degree));
                if(x6 == 0) {
                    // normally shoultn't be, but just protect 
                    System.out.println("should not be here, just in case, think over later if still happen");
                    return para;
                }
                // direction: first get radian
               // double move_degree = (Math.atan((x2 + x3) / x6));
                // convert to degree for verify
               // double move_degree_unit_degree = Math.toDegrees(move_degree);
               // double move_distance = Math.sqrt(Math.pow((x2+x3), 2) +  Math.pow(x6, 2));
                //para.move = new Translation2d(x1, x4);
                para.move = new Translation2d(x6, x2 + x3 + offset.y);
                */
			}
            para.IsValid = true;
        } else {
            System.out.println("Target Data invalid");
        }
        return para;
    }

    // Get PhotonVison Target Data
    private PhotonVisonData GetPVTargetData() {
        PhotonVisonData target_data = new PhotonVisonData();
        target_data.is_vaild = false;
        if(!IsOpen) {
            System.out.printf("Check PhotonVison Camera %s, which is NOT open\n", camera_name);
            camera = new PhotonCamera(camera_name);
            if(camera.isConnected()) {
                IsOpen = true;
                System.out.printf("PhotonVison Camera %s, opened now\n", camera_name);
            } else {    
                return target_data;
            }
        }

        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get information from target.
            // The yaw of the target in degrees (positive right).            
            target_data.yaw = target.getYaw();
            
            // The pitch of the target in degrees (positive up).
            target_data.pitch = target.getPitch();
            
            // The area (how much of the camera feed the bounding box takes up) as a percent (0-100).            
            target_data.area = target.getArea();

            if(type == PipeLineType.APRIL_TAG) {
                
                // The ID of the detected fiducial marker.
                target_data.april_tag_id = target.getFiducialId();
                
                // How ambiguous the pose of the target is must less than 0.2 
                target_data.ambiguity = target.getPoseAmbiguity();

                /* Get the transform that maps camera space 
                   (X = forward, Y = left, Z = up) to object/fiducial tag space 
                   (X forward, Y left, Z up) with the lowest reprojection error.
                */
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                
                target_data.x_distance = bestCameraToTarget.getX();
                target_data.y_distance = bestCameraToTarget.getY();
                target_data.z_distance = bestCameraToTarget.getZ();

                Rotation3d trans_3d_rotate = bestCameraToTarget.getRotation();
                target_data.x_rotate = trans_3d_rotate.getX();
                target_data.y_rotate = trans_3d_rotate.getY();
                target_data.z_rotate = trans_3d_rotate.getZ();
                target_data.angle_rotate = trans_3d_rotate.getAngle();

                // alternate Target, normally don't use it.
                //Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
                target_data.is_vaild = true;
                System.out.printf("AprilTag Raw Data: yaw:%f,pitch:%f,area:%f, 3D: X:%f,Y:%f,Z:%f, RX:%f,RY:%f,RZ:%f,RW:%f, ID:%d, ambiguity:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.x_distance,
                            target_data.y_distance,
                            target_data.z_distance,
                            target_data.x_rotate,
                            target_data.y_rotate,
                            target_data.z_rotate,
                            target_data.angle_rotate,
                            target_data.april_tag_id, 
                            target_data.ambiguity);
            } else if(type == PipeLineType.COLORED_SHAPE) {               
                // The skew of the target in degrees (counter-clockwise positive).
                target_data.skew = target.getSkew(); 
                target_data.is_vaild = true;
                System.out.printf("ColoredShape Raw Data: yaw:%f,pitch:%f,area:%f,skew:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.skew);
            } else {
                System.out.println("Currently NOT support");
            }
        } else {
            System.out.println("Target is NOT found");
        }

        return target_data;
    }

	public RobotMoveTargetParameters GetRobotMoveforGamePieceviaEdgeTpu() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        String maxConfString = "None";
        if(true) {
            try {
                StringSubscriber maxConfSub;
                NetworkTable NT_table = NetworkTableInstance.getDefault().getTable("/noteTable");    
                maxConfSub = NT_table.getStringTopic("MaxConfObj").subscribe(new String ("None"));        
                maxConfString = maxConfSub.get();
            } catch (Exception e) {
                e.printStackTrace();
                System.out.println("No game piece detection!");
                return para;
            }
        }

        //maxConfString = "\"{'x': 247, 'y': 217, 'w': 184, 'h': 41, 'conf': 0.68359375}\""; 		
        System.out.printf("maxConfString: %s\n", maxConfString);
        if(maxConfString.contains("None"))  {
            System.out.printf("maxConf String include None: %s\n", maxConfString);
            return para;
        }
		// split center X, Y. format like below:
		// {"x": None, "y": None, "w": None, "h": None, "conf": 0}
		String[] parts = maxConfString.split(",");
		String[] x_Str = parts[0].split(":"); // or substring
        if(!x_Str[1].equals("None")) {
		    double centerX = Double.parseDouble(x_Str[1]);
		    String[] y_Str = parts[1].split(":"); // or substring
            if(!y_Str[1].equals("None")) {
		        double centerY = Double.parseDouble(y_Str[1]);
             
                System.out.printf("CenterX: %f, centerY:%f\n", centerX, centerY);
        
                // Calculate yaw/pitch         
                double yaw = (centerX - image_width / 2) / pixels_per_degree_horizontal;
                double pitch = Math.toRadians((image_height / 2 - centerY) / pixels_per_degree_vertical);
                
                // calculate Move distance
                double move_x = (targetHeightMeters - cameraHeightMeters)/ Math.tan(cameraPitchRadians + pitch);    
                double move_y = Math.sqrt(cameraHeightMeters * cameraHeightMeters + 
                                        move_x * move_x) * Math.tan(Math.toRadians(yaw));
                                        
                move_y = -move_y;
                System.out.printf("move_X: %f, move_Y:%f\n", move_x, move_y);
                double x_distance = (move_x - game_piece_radius - intake_offset);
                double y_distance = (move_y  - robot_half_width);
                
                if(x_distance < 0) x_distance = 0;       
                
                para.move = new Translation2d(x_distance, y_distance);
                para.turn = new Rotation2d(0);                
				para.IsValid = true;
                System.out.printf("Target via TPU X: %f\n", para.move.getX());
                System.out.printf("Target via TPU Y: %f\n", para.move.getY());            
            }
        } else {
            System.out.println("Game Piece Info from TPU invalid");   
        }

        return para;
	}

    public RobotMoveTargetParameters GetRobotMoveforGamePiece() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        PhotonVisonData data = GetPVTargetData();
        if(data.is_vaild) {

            // this code is from Deo to calculate target distance
            double targetDistance =
                  PhotonUtils.calculateDistanceToTargetMeters(
                          Constants.CameraConstants.cameraHeight,
                          Constants.CameraConstants.targetHeight, 
                          Constants.CameraConstants.cameraPitch,
                          Units.degreesToRadians(data.pitch));
            targetDistance -= Constants.CameraConstants.goalDistance;
          //Calculate Angle.
          double targetAngle = data.yaw;
          double half_pi = (Math.PI / 2);
          double camera_to_robot_center_distance = 0.35;
         // double carema_install_angle = (23 * 3.1415926 / 180);
          double carema_install_angle = (23 * Math.PI / 180);

          //Calculate translation2d.
          Translation2d targetTranslation = PhotonUtils.estimateCameraToTargetTranslation(targetDistance, Rotation2d.fromDegrees(-data.yaw));
          Translation2d offset = new Translation2d(-camera_to_robot_center_distance * Math.sin(half_pi - carema_install_angle), 
                                                   -camera_to_robot_center_distance * Math.cos(half_pi - carema_install_angle));
          Translation2d targetTranslastionPlusOffset = targetTranslation.plus(offset);
         // para.move = targetTranslation;
          para.move = targetTranslastionPlusOffset;

         // below not sure if it is degree or radian
         // para.turn = Rotation2d.fromRadians(targetAngle)

         // here have to add carema_install_angle;
          para.turn = Rotation2d.fromDegrees(data.yaw);
          
          //SmartDashboard.putNumber("targetDistance", targetDistance);
          //SmartDashboard.putNumber("targetAngle", targetAngle);
          //SmartDashboard.putNumber("targetTranslationX", targetTranslation.getX());
          //SmartDashboard.putNumber("targetTranslationY", targetTranslation.getY());
          System.out.printf("Distance %f\n", targetDistance);
          para.IsValid = true;
        }

        return para;       
    }

    // Get Robot trun to face target paramters
    public RobotMoveTargetParameters GetRobotTurnParamters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        PhotonVisonData data = GetPVTargetData();
        if(data.is_vaild) {
            double tmp_radian = Math.abs(data.z_rotate);
            double need_turn_radian = (3.1415926 - tmp_radian);
            if(tmp_radian > 3.05 && tmp_radian < 3.2) {
                System.out.println("Already face to ArpilTag, no need to turn");
                para.TurnRadian_swerve = 0;
                para.TurnRadian_tank = 1.5707963; // 90 degree
                para.turn = Rotation2d.fromRadians(0);
                para.move = new Translation2d(0, 0);
                para.IsValid = true;
            } else {
                System.out.println("Nee to check turn ......");
                /* this Y distance need to adjust, basically the detect is not accaray if Y is too small
                if(Math.abs(data.y_distance) < 0.1) {
                    // if Y is too small, RZ not stable, need YAW assit
                    System.out.println("Y too small, need to move and recheck");
                    return para;
                } */
                para.TurnRadian_swerve = (data.z_rotate > 0) ? -need_turn_radian : need_turn_radian;
                // here 1.5707963: pi/2
                para.TurnRadian_tank = (data.z_rotate > 0) ? (need_turn_radian - 1.5707963) : (1.5707963 - need_turn_radian);
                para.turn = (data.z_rotate > 0) ? (Rotation2d.fromRadians(-need_turn_radian)) : (Rotation2d.fromRadians(need_turn_radian));
                System.out.printf("Need to turn (postive left), swerve: %f, tank:%f\n", para.TurnRadian_swerve, para.TurnRadian_tank);
                double small_distance = (data.z_rotate > 0) ? 0.5 : -0.5;
                para.move = new Translation2d(0, small_distance);
                para.IsValid = true;
            }
            already_turned_to_face_target = true;        
        } else {
            System.out.println("Target Data invalid before detect facing");
        }
        return para;
    }

    private BoundryOffset GetOffsetBaseOnApriltagID(int ApriltagID) {
        BoundryOffset para = new BoundryOffset();
        para.x = 0;
        para.y = 0;
        Optional<DriverStation.Alliance> team = DriverStation.getAlliance();
        if(team.isPresent()) {
             // different moving strategy based on AprilTag ID 
            if(team.get() == DriverStation.Alliance.Red) {
                switch (ApriltagID) {
                    case 9: //src right
                        para.x = 0;
                        para.y = (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                    case 10: // src left
                        para.x = 0;
                        para.y = -(Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH); break;
                    case 3: //speaker
                        para.x = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;
                        para.y = (Constants.CameraConstants.SPEAKER_APRIL_TAG_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                    case 4: //speaker
                        para.x = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;    break;
                    case 5: //amp
                        para.x = 0;    break;
                    case 11: //stage
                        para.x = 0;    break;
                    case 12: //stage
                        para.x = 0;    break;
                    case 13: //stage
                        para.x = 0;    break;
                    default:
                        System.out.printf("Not support this AprilTag ID:%d\n", ApriltagID); break;
                }
            } else if (team.get() == DriverStation.Alliance.Blue){
                switch (ApriltagID) {
                    case 6: // AMP center
                        para.x = 0;    break;
                    case 14: // speaker
                        para.x = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;    break;
                    case 8: //speaker
                        para.x = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;
                        para.y = -(Constants.CameraConstants.SPEAKER_APRIL_TAG_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);   break;
                    case 1: //src right
                        para.x = 0;
                        para.y = (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                    case 2: //src left
                        para.x = 0;
                        para.y = -(Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break; 
                    case 7: //stage
                        para.x = 0;    break;
                    case 15: //stage
                        para.x = 0;    break;
                    case 16: //stage
                        para.x = 0;    break;
                    default:
                        System.out.printf("Not support this AprilTag ID:%d\n", ApriltagID);
                }
            } else {
                System.out.println("Alliance team Unknown");
            }
        }
        return para;
    }

    // Get Robot move to target paramters
    public RobotMoveTargetParameters GetRobotMoveToTargetParamters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        if(already_turned_to_face_target) {
            PhotonVisonData data = GetPVTargetData();
            if(data.is_vaild) {
                Optional<DriverStation.Alliance> team = DriverStation.getAlliance();
                if(team.isPresent()) {
                    // different moving strategy based on AprilTag ID 
                    double boundary_distance = 0;
                    double y_dist = data.y_distance;
                    if(team.get() == DriverStation.Alliance.Red) {
                        switch (data.april_tag_id) {
                            case 9: //src right
                                boundary_distance = 0;
                                y_dist += (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                            case 10: // src left
                                boundary_distance = 0;
                                y_dist -= (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH); break;
                            case 3: //speaker
                                boundary_distance = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;
                                y_dist += (Constants.CameraConstants.SPEAKER_APRIL_TAG_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                            case 4: //speaker
                                boundary_distance = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;    break;
                            case 5: //amp
                                boundary_distance = 0;    break;
                            case 11: //stage
                                boundary_distance = 0;    break;
                            case 12: //stage
                                boundary_distance = 0;    break;
                            case 13: //stage
                                boundary_distance = 0;    break;
                            default:
                                System.out.printf("Not support this AprilTag ID:%d\n", data.april_tag_id);
                                return para;
                        }
                    } else if (team.get() == DriverStation.Alliance.Blue){
                        switch (data.april_tag_id) {
                            case 6: // AMP center
                                boundary_distance = 0;    break;
                            case 14: // speaker
                                boundary_distance = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;    break;
                            case 8: //speaker
                                boundary_distance = Constants.CameraConstants.SPEAKER_SUBWOOFER_WIDTH;
                                y_dist -= (Constants.CameraConstants.SPEAKER_APRIL_TAG_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);   break;
                            case 1: //src right
                                boundary_distance = 0;
                                y_dist += (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break;
                            case 2: //src left
                                boundary_distance = 0;
                                y_dist -= (Constants.CameraConstants.SRC_WIDTH + Constants.CameraConstants.APRILTAG_WIDTH);    break; //switch back case 7 and 14 LATER, just for testing purposes
                            case 7: //stage
                                boundary_distance = 0;    break;
                            case 15: //stage
                                boundary_distance = 0;    break;
                            case 16: //stage
                                boundary_distance = 0;    break;
                            default:
                                System.out.printf("Not support this AprilTag ID:%d\n", data.april_tag_id);
                                return para;
                        }
                    } else {
                        System.out.println("Alliance team Unknown");
                        return para;
                    }
                    System.out.printf("Boundary: %f,Y Distance: %f\n", boundary_distance, y_dist);
                    double x1 = (data.x_distance - boundary_distance);                
                    para.MoveRadian = (-1 * (Math.atan(x1 / y_dist)));
                    para.MoveDistance = Math.sqrt(Math.pow(x1, 2) +  Math.pow(y_dist, 2));
                    para.MoveForward = x1;
                    para.MoveUpward = y_dist;
                    para.move = new Translation2d(x1, y_dist);
                    para.IsValid = true;
                    already_turned_to_face_target = false;
                } else {
                    System.out.println("Allicance Team NOT Correct");
                }
            } else {
                System.out.println("Target Data invalid before detect moving");
            }
        } else {
            System.out.println("Can NOT move to target before turn to face target");
        }
        return para;
    }

    public Pose2d GetCurrentRobotFieldPose() {
        for(int x = 0; x < 3; x++) {
            try {
            
                if(!Apriltaglayoutload) {
                    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);                
                    System.out.printf("Tried to load, x %d\n", x);
                    Apriltaglayoutload = true;
                    break;
                }
            }
                catch (IOException e) {
                e.printStackTrace();
                System.out.println("Load April Tag Layout Error");
            }
        }
        var result = camera.getLatestResult();
        // camera for robot center x, y, z
        double CAMERA_POS_FOR_ROBOT_X = -0.3; //0.4
        double CAMERA_POS_FOR_ROBOT_Y = 0;
        double CAMERA_POS_FOR_ROBOT_Z = 0.67;
        double CAMERA_POS_FOR_ROBOT_ROLL = -0.0483;
        double CAMERA_POS_FOR_ROBOT_PITCH = 0.085;
        double CAMERA_POS_FOR_ROBOT_YAW = 3.1415986;

        Transform3d cameraToRobot = new Transform3d(CAMERA_POS_FOR_ROBOT_X, 
                                                    CAMERA_POS_FOR_ROBOT_Y, 
                                                    CAMERA_POS_FOR_ROBOT_Z, 
                                                    new Rotation3d(CAMERA_POS_FOR_ROBOT_ROLL,
                                                    CAMERA_POS_FOR_ROBOT_PITCH,
                                                    CAMERA_POS_FOR_ROBOT_YAW));
        
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(), 
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
                        cameraToRobot);
            return robotPose.toPose2d();            
        } 
        return null;
    }
}
