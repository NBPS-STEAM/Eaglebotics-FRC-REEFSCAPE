package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.LimelightHelpers;

public class LimeLocalizationSubsystem{
  private String name= "";
  private String out="";
  public double time=0;
  public double[] stdev = new double[0];

  public LimeLocalizationSubsystem(String name){
    this.name=name;
    out= this.name.concat(" pose");
  }

  private SwerveSubsystem sd;
  
  public void init(SwerveSubsystem sd){
      this.sd=sd;
  }


  public  Vector<N3> getstdev() {//janky system that adjusts how much we trust cameras based on distance and number of tags seen
    // According to: https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification
    // stdev_mt1 | MT1 Standard Deviation [x, y, z, roll, pitch, yaw] (meters, degrees)
    if (stdev.length >= 2) {
      return VecBuilder.fill(stdev[0], stdev[1], 9999999);
    } else {
      return VecBuilder.fill(0.9, 0.9, 9999999);
    }
    
    /* LimelightHelpers.PoseEstimate mt2=LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (mt2.tagCount>1) {
      return VecBuilder.fill(0.65,0.65,999999);
    }else if(mt2.avgTagDist>5){
        return VecBuilder.fill(0.9,0.9,999999);
    }else{
      if(mt2.avgTagDist>4){
        return VecBuilder.fill(0.85,0.85,999999);
      }else if(mt2.avgTagDist>3){
          return VecBuilder.fill(0.75,0.75,999999);
      }else if(mt2.avgTagDist>2){
        return VecBuilder.fill(0.65,0.65,999999);
      }else{
        return VecBuilder.fill(0.6,0.6,999999);
      }
    } */
  }

    
  public Optional<Pose2d> getPose(){
    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    if(Math.abs(sd.pigeon.getAngularVelocityYWorld().getValue().in(DegreesPerSecond)) > 360) return Optional.empty();

    LimelightHelpers.SetRobotOrientation(name, sd.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if(mt1.tagCount == 0) return Optional.empty();
    
    //SwerveDriveSubsystem.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    time=mt1.timestampSeconds;
    stdev = LimelightHelpers.getLimelightNTDoubleArray(name, "stdev_mt1");
    SmartDashboard.putString(out, mt1.pose.toString());
    return Optional.of(mt1.pose);
  }
    
}
