package frc.robot;

import java.io.IOException;
import java.text.DecimalFormat;



import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;

public class PhotonVision extends SubsystemBase {
    PhotonPoseEstimator photonPoseEstimator;
    GenericEntry robotPosTrue;
    AprilTagFieldLayout aprilTagFieldLayout;
    PiCamera camera1, camera2, camera3;
    PiCamera[] cameras;
    DecimalFormat gayDecimalFormat = new DecimalFormat("##.000");

    public PhotonVision() {
        ShuffleboardTab aprilTags = Shuffleboard.getTab("April Tags");
        robotPosTrue = aprilTags.add("True Robot Pose", "null").getEntry();
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            // Create pose estimator
           // photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, PhotonVision.photonCamera1, VisionConstants.robotToCam);
        }catch(IOException e) {}

        camera1 = new PiCamera(SensorConstants.camera1, aprilTagFieldLayout, SensorConstants.robotToCam);
        camera2 = new PiCamera(SensorConstants.camera2, aprilTagFieldLayout, SensorConstants.robotToCam);
        camera3 = new PiCamera(SensorConstants.camera3, aprilTagFieldLayout, SensorConstants.robotToCam);

        PiCamera[] cameras = {camera1, camera2, camera3};
        this.cameras = cameras;
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     *
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }*/

    /*public void resetOdometryToAprilTags(SwerveSubsystem swerveSubsystem) {
        double[] robotPos = getAvgRobotPose();
        double percievedX = swerveSubsystem.getPose().getX();
        double percievedY = swerveSubsystem.getPose().getY();

        if(robotPos == null)
            return;
        if(robotPos[0] > percievedX + (percievedX * 0.1) || robotPos[1] > percievedY + (percievedY * 0.1)) 
            return;
        swerveSubsystem.resetOdometry(new Pose2d(robotPos[0],robotPos[1],new Rotation2d(swerveSubsystem.getHeading())));
    }*/

    public double[] getAvgRobotPose() {
        double[] robotPos = {0,0};
        int numOfTargets = 0;

        for(int i = 0; i < cameras.length; i++){
            if(cameras[i].getAprilTagID() != -1) {
                Pose3d camPos = cameras[i].getRobotFieldPose();
                robotPos[0] = robotPos[0] + camPos.getX();
                robotPos[1] = robotPos[1] + camPos.getY();
                numOfTargets++;
            } 
        }

        if(numOfTargets != 0){
            robotPos[0] = robotPos[0]/numOfTargets;
            robotPos[1] = robotPos[1]/numOfTargets;
            robotPosTrue.setString("X: "+gayDecimalFormat.format((robotPos[0])+" Y: "+gayDecimalFormat.format((robotPos[1]))));
            return robotPos;
            
        }else {
            System.out.println("Can't find pos because no target seen!");
            return null;
        }
        
    }

    @Override
    public void periodic() {
        camera1.updateShuffleBoard();
        camera2.updateShuffleBoard();
        camera3.updateShuffleBoard();
    }   
}
