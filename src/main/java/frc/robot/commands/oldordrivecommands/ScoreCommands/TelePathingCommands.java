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
    //If it can't go straight, it will determine the fastest direction to go around the reef (goCounterCW)
    //After determining the direction, it loops through the corners of the reef to see which are
    //between the current angle from the reef and the target angle
    //then sorts them into a list by distance from the robot
    //then creates a list of poses (I'm not sure but the robot might end up backwards as it circles the reef)

    //There's probably a mistake, but I did test it in IntelliJ and it seemed alright
    //There was also probably a simpler way but oh well
    
    public Command GoToReefSmartCommand(int index) {

        PathPlannerPath path;

        //finDist is the distance from the final point of the path to the center of the reef
        //waypntDist is the distance from the waypoints the robot uses to circle the reef from the center of the reef
        double finDist = 1.75;
        double waypntDist = 2.15;
     

        //The current position of the robot
        double roboX = swerve.getPose().getX();
        double roboY = swerve.getPose().getY();

        //The final position the robot will end at ((4.5, 4.0) is the center of the reef)
        double targX = finDist * Math.cos(index * Math.PI / 3) + 4.5;
        double targY = finDist * Math.sin(index * Math.PI / 3) + 4.0;

        //This calculates a line, parallel to the reef,
        //and centered at the targetpoint. Then it calculates the distance
        //of the x position of the robot from the x position of the line
        //at the y position of the robot (The x(y))
        double tangent = -Math.tan(index * Math.PI / 3);
        double pointDifference = roboX - (targX + tangent*(roboY-targY));

        double slope = index * Math.PI / 3;

        //If the index is 0, 1, or 5, the robot needs to be to the right
        //of the line created above to be able to get to the final point
        //in a straight line. 
        boolean canGoStraight;
        if (index <= 1 || index == 5) {
            canGoStraight = pointDifference > 0;
        } else {
            canGoStraight = pointDifference < 0;
        }


        //The code before the 'else' is basically the same as the simple path
        if (canGoStraight) {


            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                finDist*Math.cos(index * Math.PI/3) + 4.5,
                finDist*Math.sin(index * Math.PI/3) + 4.0,
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

            //Creates the angle from the center of the reef to the robot
            //counterCW is counter clockwise
            double angle = Math.atan2(roboY - 4.0, roboX - 4.5);
            if (angle < 0) {
                angle += Math.PI*2;
            }
            boolean goCounterCW;

            //Determines which direction to go. Honestly I'm not 100% sure why it works
            if (angle > slope) {
                double counterCW = Math.abs(angle - (slope + 2 * Math.PI));
                double clockwise = Math.abs(angle - slope);

                goCounterCW = counterCW < clockwise;
            } else {
                double clockwise = Math.abs(angle - slope);
                double counterCW = Math.abs(angle - (slope - 2 * Math.PI));
                System.out.println(angle - (slope - 2 * Math.PI));

                goCounterCW = clockwise < counterCW;
            }

            ArrayList<Double> angleList = new ArrayList<>();


            //This is the really rough part, but it works.
            //Based on the angles and whether it can go clockwise, it chooses which code to run
            //Each one is fairly similar with slightly different parameters.
            //It loops through each possible index point, and determines whether its 'in the way'
            //by seeing if its between the angle of the robot and the angle of the target point.
            //If so, it adds it to angleList
            if (goCounterCW) {
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

            //This converts the angles gotten above into their distances from the robot
            //then it sorts them by value (smallest/closest first).
            //It does not actually change the values, as the final list is still a list of angles
            ArrayList<Double> sortedAngleList = angleList.stream()
                    .sorted(Comparator.comparing(ang -> Math.hypot(waypntDist * Math.cos(ang) + 4.5 - roboX, waypntDist * Math.sin(ang) + 4.0 - roboY)))
                    .collect(Collectors.toCollection(ArrayList::new));
            
            //Creates a list of poses from each angle from sortedAngleList
            //as well as adding the final point
            ArrayList<Pose2d> poseList = new ArrayList<>();
            for (Double ang : sortedAngleList) {
                poseList.add(new Pose2d(
                    waypntDist * Math.cos(ang) + 4.5,
                    waypntDist * Math.sin(ang) + 4.0,
                    Rotation2d.fromRadians(ang + Math.PI/2)
                ));
            }
            poseList.add(new Pose2d(
                finDist*Math.cos(index * Math.PI/3) + 4.5,
                finDist*Math.sin(index * Math.PI/3) + 4.0,
                Rotation2d.fromDegrees(180 + index*60))
            );

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