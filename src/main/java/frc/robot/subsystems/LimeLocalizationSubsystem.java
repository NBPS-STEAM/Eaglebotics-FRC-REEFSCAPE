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

  public LimeLocalizationSubsystem(String name){
    this.name=name;
    out= this.name.concat(" pose");
  }

  private SwerveSubsystem sd;
  
  public void init(SwerveSubsystem sd){
      this.sd=sd;
  }


  public  Vector<N3> getstdev() {//janky system that adjusts how much we trust cameras based on distance and number of tags seen
    LimelightHelpers.PoseEstimate mt2=LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (mt2.tagCount>1) {
      return VecBuilder.fill(0.65,0.65,0.999999);
    }else if(mt2.avgTagDist>5){
        return VecBuilder.fill(0.9,0.9,0.99999);
    }else{
      if(mt2.avgTagDist>4){
        return VecBuilder.fill(0.85,0.85,0.999999);
      }else if(mt2.avgTagDist>3){
          return VecBuilder.fill(0.75,0.75,0.999999);
      }else if(mt2.avgTagDist>2){
        return VecBuilder.fill(0.65,0.65,0.999999);
      }else{
        return VecBuilder.fill(0.6,0.6,0.999999);
      }
    }
  }

    
  public Optional<Pose2d> getPose(){
    boolean doRejectUpdate=false;
    LimelightHelpers.SetRobotOrientation(name, sd.swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if(Math.abs(sd.pigeon.getAngularVelocityYWorld().getValue().in(DegreesPerSecond)) > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      //SwerveDriveSubsystem.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      time=mt2.timestampSeconds;
      SmartDashboard.putString(out, mt2.pose.toString());
      return Optional.of(mt2.pose);
    }
    return Optional.empty();
  }
    
}
