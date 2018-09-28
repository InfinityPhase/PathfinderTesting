package main.java;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Brain {
    public static void main( String[] args ) {
        PathGenerator path = new PathGenerator( new Waypoint[] {
            new Waypoint( 0, 0, 0 ),
            new Waypoint( 6, -6, 0 ),
            new Waypoint( 12, 0, 0 )
        }, true );
    }


    private static class PathGenerator {
        private Trajectory path;
        private EncoderFollower leftFollower;
        private EncoderFollower rightFollower;
        private java.util.Timer timer;

        public PathGenerator( Waypoint[] waypoints ) {
            this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), false );
        }

        public PathGenerator( Waypoint[] waypoints, boolean reversePath ) {
            this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), reversePath );
        }

        public PathGenerator( Trajectory path ) {
            this( path, false );
        }

        public PathGenerator( Trajectory path, boolean reverseDrive ) {
            this.path = path;
            generateTankPath( reverseDrive );
        }

        private void generateTankPath( boolean reverseDrive ) {
            TankModifier modifier = new TankModifier(path).modify( Calibration.Pathfinder.ROBOT_WIDTH, Calibration.PATHFINDER_CONFIG );

            if( reverseDrive ) {
                Trajectory left = Pathfinder.reverseTrajectory( modifier.getLeftTrajectory( ) );
                Trajectory right = Pathfinder.reverseTrajectory( modifier.getRightTrajectory( ) );
                leftFollower = new EncoderFollower( left );
                rightFollower = new EncoderFollower( right );

                for( Trajectory.Segment s : left.segments ) {
                    System.out.println(Pathfinder.boundHalfDegrees( Pathfinder.r2d( s.heading ) ) );
                }
            } else {
                leftFollower = new EncoderFollower( modifier.getLeftTrajectory() );
                rightFollower = new EncoderFollower( modifier.getRightTrajectory() );
            }
        }

    }

}



