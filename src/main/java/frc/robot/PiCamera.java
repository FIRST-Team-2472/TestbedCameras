package frc.robot;

import java.text.DecimalFormat;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PiCamera {
    PhotonPipelineResult result;
    PhotonCamera camera;
    PhotonTrackedTarget target;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobotPos;
    DecimalFormat gayDecimalFormat = new DecimalFormat("##.000");
    GenericEntry aprilTagID,robotPos;
    
    public PiCamera (String cameraID, AprilTagFieldLayout aprilTagFieldLayout, Transform3d cameraToRobotPos) {
        camera = new PhotonCamera(cameraID);
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.cameraToRobotPos = cameraToRobotPos;

        ShuffleboardTab aprilTags = Shuffleboard.getTab("April Tags");

        aprilTagID = aprilTags.add(cameraID+" Tag ID", 0).getEntry();
        robotPos = aprilTags.add(cameraID+" Robot Pos", "null").getEntry();
    }

    public Pose3d getRobotFieldPose() {
        result = camera.getLatestResult();
        if(result.hasTargets()){
            target = result.getBestTarget();
            Pose3d temp = aprilTagFieldLayout.getTagPose(getAprilTagID()).get();
            return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),temp,cameraToRobotPos);
        }else return new Pose3d();
    }

    public int getAprilTagID() {
        result = camera.getLatestResult();
        if(result.hasTargets()){
            target = result.getBestTarget();
            return target.getFiducialId();
        }else return -1;
    }

    public void updateShuffleBoard() {
        aprilTagID.setInteger(getAprilTagID());
        Pose3d temp = getRobotFieldPose();
        
        robotPos.setString("X: "+gayDecimalFormat.format(temp.getX())+" Y: "+gayDecimalFormat.format(temp.getY())+" Z: "+gayDecimalFormat.format(temp.getZ()));
    }
}
