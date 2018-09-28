package main.java;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Brain {
    public static void main( String[] args ) {
        PathFollower path = new PathFollower( new Waypoint[] {
            new Waypoint( 0, 0, 0 ),
            new Waypoint( 6, -6, 0 ),
            new Waypoint( 12, 0, 0 )
        }, true );
    }


    private static class PathFollower {
        private Trajectory path;
        private EncoderFollower leftFollower;
        private EncoderFollower rightFollower;
        private java.util.Timer timer;

        public PathFollower( Waypoint[] waypoints ) {
            this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), false );
        }

        public PathFollower( Waypoint[] waypoints, boolean reversePath ) {
            this( Pathfinder.generate( waypoints, Calibration.PATHFINDER_CONFIG ), reversePath );
        }

        public PathFollower( Trajectory path ) {
            this( path, false );
        }

        public PathFollower( Trajectory path, boolean reverseDrive ) {
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

///*
//        class PathFollowTask extends TimerTask {
//            private double prevAngleError=0;
//            private PathFollowSide side;
//            private double next_dt = 0;
//
//            private double clamp(double value, double low, double high) {
//                return Math.max(low, Math.min(value, high));
//            }
//
//            PathFollowTask (PathFollowSide side) {
//                this.side=side;
//            }
//
//            public void run() {
//                tankDrive.activate();
//                int encoderPos;
//                double rawPow;
//                Trajectory.Segment prev_seg;
//                Trajectory.Segment seg;
//                if (side==PathFollowSide.LEFT) {
//                    encoderPos = robot.drive.leftClicks.get();
//                    rawPow = leftFollower.calculate(encoderPos);
//                    prev_seg = leftFollower.prevSegment();
//                    seg = leftFollower.getSegment();
//                } else {//if side==PathFollowSide.RIGHT
//                    encoderPos=robot.drive.rightClicks.get();
//                    rawPow = rightFollower.calculate(encoderPos);
//                    prev_seg = rightFollower.prevSegment();
//                    seg = rightFollower.getSegment();
//                }
//
//                next_dt = seg.dt;
//
//                // Calculated curvature scales with velocity
//                // Keeping old implied scaling since faster movement implies more curvature correction
//                // Use harmonic mean because curvature is 1/radius
//                double scaleVel=2*leftFollower.getSegment().velocity*rightFollower.getSegment().velocity;
//                if (scaleVel!=0) {
//                    scaleVel/=(leftFollower.getSegment().velocity+rightFollower.getSegment().velocity);
//                }
//                double curvature = PathFinderUtil.getScaledCurvature(prev_seg,seg,scaleVel);
//
//                double normcurv=PathFinderUtil.getNormalizedCurvature(prev_seg, seg);
//                //System.out.println("Normcurv is "+normcurv+" for "+side.toString());
//
//                // Raw heading stuff here due to side selections
//                double degreeHeading = AutonMovement.clicksToDegrees(Calibration.DRIVE_PROPERTIES,
//                    robot.drive.leftClicks.get()-robot.drive.rightClicks.get());
//                //System.out.print("Current heading is "+degreeHeading);
//
//                // Both headings are the same
//                double desiredHeading = leftFollower.getHeading();
//                desiredHeading=Pathfinder.r2d(desiredHeading);
//                desiredHeading=Pathfinder.boundHalfDegrees(desiredHeading);
//                desiredHeading*=-1;
//                // Pathfinder heading is counterclockwise math convention
//                // We are using positive=right clockwise convention
//
//                double angleError = desiredHeading-degreeHeading;
//                System.out.println("Angle error is "+angleError);
//
//                // Convert back into radians for consistency
//                angleError = Pathfinder.d2r(angleError);
//                double dAngleError=angleError-prevAngleError;
//                dAngleError/=seg.dt;
//
//                double kappa_val=Calibration.Pathfinder.K_KAPPA*curvature;
//                double pTheta_val=Calibration.Pathfinder.K_PTHETA_0/(Calibration.Pathfinder.K_PTHETA_DECAY*normcurv*normcurv+1);
//                pTheta_val*=angleError;
//                double dTheta_val=Calibration.Pathfinder.K_DTHETA_0/(Calibration.Pathfinder.K_DTHETA_DECAY*normcurv*normcurv+1);
//                dTheta_val*=dAngleError;
//
//                dTheta_val=clamp(dTheta_val,-pTheta_val,pTheta_val);
//
//				/*
//				double deshed=-Pathfinder.boundHalfRadians(leftFollower.getHeading());
//				System.out.println("Equal "+(deshed-Pathfinder.d2r(desiredHeading)));
//				*/
//
//                if (side==PathFollowSide.LEFT) {
//                    System.out.println(rawPow+kappa_val
//                        +pTheta_val+dTheta_val);
//                } else { // RIGHT side
//                    System.out.println(rawPow-kappa_val
//                        -pTheta_val-dTheta_val);
//                }
//                prevAngleError=angleError;
//
//                if( side==PathFollowSide.LEFT ) {
//                    timer.schedule( new PathFollowTask( PathFollowSide.LEFT), (long) ( 1000*getNextdt() ) );
//                } else if( side == PathFollowSide.RIGHT ) {
//                    timer.schedule( new PathFollowTask( PathFollowSide.RIGHT), (long) (1000*getNextdt()) );
//                }
//            }
//
//            double getNextdt() {
//                return next_dt;
//            }
//        }
//
//        public void begin() {
//            System.out.println("ERROR: Begin");
//            timer = new java.util.Timer();
//            robot.drive.resetSensors();
//            System.out.println("ERROR: left is "+robot.drive.leftClicks.get());
//            leftFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
//                Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
//            rightFollower.configurePIDVA(Calibration.Pathfinder.KP, Calibration.Pathfinder.KI, Calibration.Pathfinder.KD,
//                Calibration.Pathfinder.KV, Calibration.Pathfinder.KA);
//            leftFollower.configureEncoder(0, 250, 0.12732); // 5 in diameter
//            rightFollower.configureEncoder(0, 250, 0.12732);
//            leftFollower.reset();
//            rightFollower.reset();
//            timer.schedule(new PathFollowTask(PathFollowSide.LEFT), 0);
//            timer.schedule(new PathFollowTask(PathFollowSide.RIGHT), 0);
//            tankDrive.activate();
//            timeElapsed.start();
//        }
//
//        public boolean run() {
//            tankDrive.activate();
//            return timeElapsed.runUntil(0.6, new Runnable() {
//                @Override
//                public void run() {
//                    boolean targetReached = (leftFollower.isFinished() && rightFollower.isFinished());
//                    if (!targetReached) {
//                        timeElapsed.reset();
//                        PIDTargetPulse.update(true);
//                    } else {
//                        PIDTargetPulse.update(false);
//                    }
//                    if (PIDTargetPulse.isFallingEdge()) {
//                        System.out.println("========Finished========");
//                    }
//                }
//            });
//        }
//
//        public void end() {
//            timer.cancel();
//            System.out.println("ERROR: end");
//            leftFollower.reset();
//            rightFollower.reset();
//            timeElapsed.stopAndReset();
//        }
    }

}



