package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.List;
import java.util.stream.Collectors;
import java.util.ArrayList;
import java.util.Comparator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TelePathingCommands {

    SwerveSubsystem swerve;

    public TelePathingCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command GoToReefCommand(int index) {

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                1.75*Math.cos(index * Math.PI/3) + 4.5,
                1.75*Math.sin(index * Math.PI/3) + 4.0,
                Rotation2d.fromDegrees(180 + index*60))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Turns the path into a command
        return AutoBuilder.followPath(path);

    }


    //It looks really bad and complex, because it is
    //but basically it takes the target point, the robot position, and determines
    //whether or not to go in a straight line (canGoStraight).
    //If it can't go straight, it will determine the fastest direction to go around the reef (goClockwise)
    //(By the way I mixed up clockwise and counterclockwise so its actually the opposite of the variable names)
    //After determining the direction, it loops through the corners of the reef to see which are
    //between the current angle from the reef and the target angle
    //then sorts them into a list by distance from the robot
    //then creates a list of poses (I'm not sure but the robot might end up backwards as it circles the reef)

    //There's probably a mistake, but I did test it in IntelliJ and it seemed alright
    //There was also probably a simpler way but oh well
    
    public Command GoToReefSmartCommand(int index) {

        PathPlannerPath path;
     

        double roboX = swerve.getPose().getX();
        double roboY = swerve.getPose().getY();

        double targX = 1.75 * Math.cos(index * Math.PI / 3) + 4.5;
        double targY = 1.75 * Math.sin(index * Math.PI / 3) + 4.0;

        double slope = -Math.tan(index * Math.PI / 3);
        System.out.println(slope);

        double pointDifference = roboX - (targX + slope*(roboY-targY));
        System.out.println(pointDifference);

        slope = index * Math.PI / 3;

        boolean canGoStraight;
        if (index <= 1 || index == 5) {
            canGoStraight = pointDifference > 0;
        } else {
            canGoStraight = pointDifference < 0;
        }


        if (canGoStraight) {


            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                1.75*Math.cos(index * Math.PI/3) + 4.5,
                1.75*Math.sin(index * Math.PI/3) + 4.0,
                Rotation2d.fromDegrees(180 + index*60))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );


        } else {

            double angle = Math.atan2(roboY - 4.0, roboX - 4.5);
            if (angle < 0) {
                angle += Math.PI*2;
            }
            boolean goClockwise;

            if (angle > slope) {
                double clockwise = Math.abs(angle - (slope + 2 * Math.PI));
                double counterCW = Math.abs(angle - slope);

                goClockwise = clockwise < counterCW;
            } else {
                double clockwise = Math.abs(angle - slope);
                double counterCW = Math.abs(angle - (slope - 2 * Math.PI));
                System.out.println(angle - (slope - 2 * Math.PI));

                goClockwise = clockwise < counterCW;
            }

            ArrayList<Double> angleList = new ArrayList<>();


            if (goClockwise) {
                if (angle > slope) {
                    System.out.println(1);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) > angle || (i * Math.PI / 3 + Math.PI/6) < slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                } else {
                    System.out.println(2);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) > angle && (i * Math.PI / 3 + Math.PI/6) < slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                }
            } else {
                if (angle > slope) {
                    System.out.println(3);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) < angle && (i * Math.PI / 3 + Math.PI/6) > slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                } else {
                    System.out.println(4);
                    for (int i = 0; i < 6; i++) {
                        if ((i * Math.PI / 3 + Math.PI/6) < angle || (i * Math.PI / 3 + Math.PI/6) > slope) {
                            angleList.add(i * Math.PI / 3 + Math.PI/6);
                        }
                    }
                }
            }

            ArrayList<Double> sortedAngleList = angleList.stream()
                    .sorted(Comparator.comparing(ang -> Math.hypot(2.15 * Math.cos(ang) + 4.5 - roboX, 2.15 * Math.sin(ang) + 4.0 - roboY)))
                    .collect(Collectors.toCollection(ArrayList::new));
            
            ArrayList<Pose2d> poseList = new ArrayList<>();
            for (Double ang : sortedAngleList) {
                poseList.add(new Pose2d(
                    2.15 * Math.cos(ang) + 4.5,
                    2.15 * Math.sin(ang) + 4.0,
                    Rotation2d.fromRadians(ang + Math.PI/2)
                ));
            }
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseList);

            PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    
            // Create the path using the waypoints created above
            path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, Rotation2d.fromDegrees(180 + index*60)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
        }
        

        return AutoBuilder.followPath(path);

    }

}