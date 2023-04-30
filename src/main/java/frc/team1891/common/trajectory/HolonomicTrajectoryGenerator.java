// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;

public class HolonomicTrajectoryGenerator {
    private static final HolonomicTrajectory DO_NOTHING_TRAJECTORY =
        new HolonomicTrajectory(List.of(new HolonomicTrajectory.State()));
    private static BiConsumer<String, StackTraceElement[]> errorFunc;

    /** Private constructor because this is a utility class. */
    private HolonomicTrajectoryGenerator() {}

    private static void reportError(String error, StackTraceElement[] stackTrace) {
        if (errorFunc != null) {
           errorFunc.accept(error, stackTrace);
        } else {
            MathSharedStore.reportError(error, stackTrace);
        }
    }

    /**
     * Set error reporting function. By default, DriverStation.reportError() is used.
     *
     * @param func Error reporting function, arguments are error and stackTrace.
     */
    public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
        errorFunc = func;
    }

    /**
     * Generates a trajectory from the given control vectors and config. This method uses clamped
     * cubic splines -- a method in which the exterior control vectors and interior waypoints are
     * provided. The headings are automatically determined at the interior points to ensure continuous
     * curvature.
     *
     * @param initial The initial control vector.
     * @param interiorWaypoints The interior waypoints.
     * @param end The ending control vector.
     * @param config The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static HolonomicTrajectory generateHolonomicTrajectory(
        Spline.ControlVector initial,
        List<Translation2d> interiorWaypoints,
        Spline.ControlVector end,
        TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        // Clone the control vectors.
        var newInitial = new Spline.ControlVector(initial.x, initial.y);
        var newEnd = new Spline.ControlVector(end.x, end.y);

        // Change the orientation if reversed.
        if (config.isReversed()) {
            newInitial.x[1] *= -1;
            newInitial.y[1] *= -1;
            newEnd.x[1] *= -1;
            newEnd.y[1] *= -1;
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points =
                splinePointsFromSplines(
                    SplineHelper.getCubicSplinesFromControlVectors(
                        newInitial, interiorWaypoints.toArray(new Translation2d[0]), newEnd));
        } catch (MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return DO_NOTHING_TRAJECTORY;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return HolonomicTrajectoryParameterizer.timeParameterizeHolonomicTrajectory(
            points,
            config.getConstraints(),
            config.getStartVelocity(),
            config.getEndVelocity(),
            config.getMaxVelocity(),
            config.getMaxAcceleration(),
            config.isReversed());
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses clamped cubic
     * splines -- a method in which the initial pose, final pose, and interior waypoints are provided.
     * The headings are automatically determined at the interior points to ensure continuous
     * curvature.
     *
     * @param start The starting pose.
     * @param interiorWaypoints The interior waypoints.
     * @param end The ending pose.
     * @param config The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static HolonomicTrajectory generateHolonomicTrajectory(
        Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
        var controlVectors =
            SplineHelper.getCubicControlVectorsFromWaypoints(
                start, interiorWaypoints.toArray(new Translation2d[0]), end);

        // Return the generated trajectory.
        return generateHolonomicTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config);
    }

    /**
     * Generates a trajectory from the given quintic control vectors and config. This method uses
     * quintic hermite splines -- therefore, all points must be represented by control vectors.
     * Continuous curvature is guaranteed in this method.
     *
     * @param controlVectors List of quintic control vectors.
     * @param config The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static HolonomicTrajectory generateHolonomicTrajectory(
        ControlVectorList controlVectors, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
        final var newControlVectors = new ArrayList<Spline.ControlVector>(controlVectors.size());

        // Create a new control vector list, flipping the orientation if reversed.
        for (final var vector : controlVectors) {
            var newVector = new Spline.ControlVector(vector.x, vector.y);
            if (config.isReversed()) {
                newVector.x[1] *= -1;
                newVector.y[1] *= -1;
            }
            newControlVectors.add(newVector);
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points =
                splinePointsFromSplines(
                    SplineHelper.getQuinticSplinesFromControlVectors(
                        newControlVectors.toArray(new Spline.ControlVector[] {})));
        } catch (MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return DO_NOTHING_TRAJECTORY;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return HolonomicTrajectoryParameterizer.timeParameterizeHolonomicTrajectory(
            points,
            config.getConstraints(),
            config.getStartVelocity(),
            config.getEndVelocity(),
            config.getMaxVelocity(),
            config.getMaxAcceleration(),
            config.isReversed());
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
     * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature is
     * guaranteed in this method.
     *
     * @param waypoints List of waypoints..
     * @param config The configuration for the trajectory.
     * @return The generated trajectory.
     */
    @SuppressWarnings("LocalVariableName")
    public static HolonomicTrajectory generateHolonomicTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        List<Pose2d> newWaypoints = new ArrayList<>();
        if (config.isReversed()) {
            for (Pose2d originalWaypoint : waypoints) {
                newWaypoints.add(originalWaypoint.plus(flip));
            }
        } else {
          newWaypoints.addAll(waypoints);
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints));
        } catch (MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return DO_NOTHING_TRAJECTORY;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return HolonomicTrajectoryParameterizer.timeParameterizeHolonomicTrajectory(
            points,
            config.getConstraints(),
            config.getStartVelocity(),
            config.getEndVelocity(),
            config.getMaxVelocity(),
            config.getMaxAcceleration(),
            config.isReversed());
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses quintic hermite
     * splines -- therefore, all points must be represented by Pose2d objects. Continuous curvature is
     * guaranteed in this method.
     *
     * @param waypoints List of waypoints
     * @param headings Headings with each waypoint
     * @param config The configuration for the trajectory
     * @return The generated trajectory.
     */
    @SuppressWarnings("LocalVariableName")
        public static HolonomicTrajectory generateHolonomicTrajectory(List<Pose2d> waypoints, List<Rotation2d> headings, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        List<Pose2d> newWaypoints = new ArrayList<>();
        List<Rotation2d> newHeadings = new ArrayList<>();
        if (config.isReversed()) {
            for (Pose2d originalWaypoint : waypoints) {
                newWaypoints.add(originalWaypoint.plus(flip));
            }
            for (Rotation2d originalHeading : headings) {
                newHeadings.add(originalHeading.plus(flip.getRotation()));
            }
        } else {
          newWaypoints.addAll(waypoints);
          newHeadings.addAll(headings);
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints));
        } catch (MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return DO_NOTHING_TRAJECTORY;
        }

        ArrayList<Integer> timePoints = new ArrayList<>();
        int j = 0;
        for (int i = 0; i < points.size(); i++) {
            PoseWithCurvature point = points.get(i);
            if (point.poseMeters.equals(newWaypoints.get(j))) {
                timePoints.add(i);
                j++;
            }
        }

        ArrayList<Pair<Rotation2d, Integer>> timeBoundHeadings = new ArrayList<>();
        for (int i = 0; i < newWaypoints.size(); i++) {
            timeBoundHeadings.add(new Pair<>(newHeadings.get(i), timePoints.get(i)));
        }
        List<Rotation2d> interpolatedHeadings = AngleInterpolator.interpolate(
            timeBoundHeadings
        );

        // System.out.println("Debug ----> interpolatedHeadings:");
        // for (int i = 0; i < interpolatedHeadings.size(); i++) {
        //     // System.out.println(points.get(i).poseMeters);
        //     System.out.println(interpolatedHeadings.get(i).getRadians());
        // }


        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return HolonomicTrajectoryParameterizer.timeParameterizeHolonomicTrajectory(
            points,
            interpolatedHeadings,
            config.getConstraints(),
            config.getStartVelocity(),
            config.getEndVelocity(),
            config.getMaxVelocity(),
            config.getMaxAcceleration(),
            config.isReversed());
    }

    /**
     * Generate spline points from a vector of splines by parameterizing the splines.
     *
     * @param splines The splines to parameterize.
     * @return The spline points for use in time parameterization of a trajectory.
     * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
     *     with approximately opposing headings)
     */
    public static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines) {
        // Create the vector of spline points.
        var splinePoints = new ArrayList<PoseWithCurvature>();

        // var first = splines[0].getPoint(0.0).poseMeters;
        // var second = splines[0].getPoint(0.0).poseMeters;
        // var secondToLast = splines[0].getPoint(0.0).poseMeters;
        // var last = splines[0].getPoint(0.0).poseMeters;

        // Add the first point to the vector.
        splinePoints.add(splines[0].getPoint(0.0));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        for (final var spline : splines) {
            var points = SplineParameterizer.parameterize(spline);

            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            splinePoints.addAll(points.subList(1, points.size()));
        }

        return splinePoints;
    }

    // Work around type erasure signatures
    public static class ControlVectorList extends ArrayList<Spline.ControlVector> {
        public ControlVectorList(int initialCapacity) {
         super(initialCapacity);
        }

        public ControlVectorList() {
          super();
        }

        public ControlVectorList(Collection<? extends Spline.ControlVector> collection) {
          super(collection);
        }
    }

    public static class AngleInterpolator {
        private AngleInterpolator() {}

        @SafeVarargs
        public static List<Rotation2d> interpolate(Pair<Rotation2d, Integer> ... headings) {
            List<CoefficientWave> coefficients = new ArrayList<>();

            coefficients.add(new CoefficientWave(null, headings[0], headings[1]));
            for (int i = 1; i < headings.length-1; i++) {
                coefficients.add(new CoefficientWave(headings[i-1], headings[i], headings[i+1]));
            }
            coefficients.add(new CoefficientWave(headings[headings.length-2], headings[headings.length-1], null));

            List<Rotation2d> rotations = new ArrayList<>();
            for (int i = 0; i <= headings[headings.length-1].getSecond(); i++) {
                double angle = 0;
                for (int j = 0; j < coefficients.size(); j++) {
                    angle += coefficients.get(j).get(i, headings[j].getFirst().getRadians());
                }
                rotations.add(new Rotation2d(angle));
            }
            
            return rotations;
        }

        public static List<Rotation2d> interpolate(ArrayList<Pair<Rotation2d, Integer>> headings) {
            List<CoefficientWave> coefficients = new ArrayList<>();

            coefficients.add(new CoefficientWave(null, headings.get(0), headings.get(1)));
            for (int i = 1; i < headings.size()-1; i++) {
                coefficients.add(new CoefficientWave(headings.get(i-1), headings.get(i), headings.get(i+1)));
            }
            coefficients.add(new CoefficientWave(headings.get(headings.size()-2), headings.get(headings.size()-1), null));

            List<Rotation2d> rotations = new ArrayList<>();
            for (int i = 0; i <= headings.get(headings.size()-1).getSecond(); i++) {
                double angle = 0;
                for (int j = 0; j < coefficients.size(); j++) {
                    angle += coefficients.get(j).get(i, headings.get(j).getFirst().getRadians());
                }
                // Confine to [0, 2pi)
                angle %= 2 * Math.PI;
                angle = angle < 0 ? angle + 2 * Math.PI : angle;
                rotations.add(new Rotation2d(angle));
            }
            
            return rotations;
        }

        
        /**
         * This is the janky part.  Each point will have a sine wave, with only 1 crest (0 everywhere else) where
         * this value has influence.
         */
        public static class CoefficientWave {
            int leftLength, rightLength;
            int location;

            boolean rotClockWise, rotCounterClockWise;
            public CoefficientWave(int leftLength, int location, int rightLength) {
                this.leftLength = leftLength;
                this.location = location;
                this.rightLength = rightLength;
            }

            public CoefficientWave(Pair<Rotation2d, Integer> left, Pair<Rotation2d, Integer> here, Pair<Rotation2d, Integer> right) {
                if (left == null) {
                    this.leftLength = 0;
                } else {
                    this.leftLength = here.getSecond() - left.getSecond();
                    tryInvert(left.getFirst(), here.getFirst());
                }
                this.location = here.getSecond();
                if (right == null) {
                    this.rightLength = 0;
                } else {
                    this.rightLength = right.getSecond() - here.getSecond();
                }
            }

            public double get(int t, double rotation) {
                if (t < location) {
                    return left(t, rotation);
                } else if (t > location) {
                    return right(t, rotation);
                } else {
                    return rotation;
                }
            }

            private double left(int t, double rotation) {
                if (location - t <= leftLength) {
                    return 1/2. * (Math.cos((t-location) * (Math.PI / leftLength)) + 1) * (rotation + (rotCounterClockWise ? Math.PI * 2 : 0) + (rotClockWise ? -Math.PI * 2 : 0));
                } else {
                    return 0;
                }
            }

            private double right(int t, double rotation) {
                if (t - location <= rightLength) {
                    return 1/2. * (Math.cos((t-location) * (Math.PI / rightLength)) + 1) * rotation;
                } else {
                    return 0;
                }
            }

            private void tryInvert(Rotation2d r1, Rotation2d r2) {
                double a1 = r1.getRadians();
                double a2 = r2.getRadians();
                rotCounterClockWise = Math.abs(2*Math.PI - (a1 - a2)) < Math.abs(a1 - a2);
                rotClockWise = Math.abs(2*Math.PI - (a2 - a1)) < Math.abs(a1 - a2);
            }
        }
    }
}
