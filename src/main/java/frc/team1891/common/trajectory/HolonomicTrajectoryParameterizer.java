// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

import java.util.ArrayList;
import java.util.List;

/** Class used to parameterize a trajectory by time. */
public final class HolonomicTrajectoryParameterizer {
    /** Private constructor because this is a utility class. */
    private HolonomicTrajectoryParameterizer() {}

    /**
     * Parameterize the trajectory by time. This is where the velocity profile is generated.
     *
     * <p>The derivation of the algorithm used can be found <a
     * href="http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf">here</a>.
     *
     * @param points Reference to the spline points.
     * @param constraints A vector of various velocity and acceleration. constraints.
     * @param startVelocityMetersPerSecond The start velocity for the trajectory.
     * @param endVelocityMetersPerSecond The end velocity for the trajectory.
     * @param maxVelocityMetersPerSecond The max velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
     * @param reversed Whether the robot should move backwards. Note that the robot will still move
     *     from a -&gt; b -&gt; ... -&gt; z as defined in the waypoints.
     * @return The trajectory.
     */
    public static HolonomicTrajectory timeParameterizeHolonomicTrajectory(
            List<PoseWithCurvature> points,
            List<TrajectoryConstraint> constraints,
            double startVelocityMetersPerSecond,
            double endVelocityMetersPerSecond,
            double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq,
            boolean reversed) {
        var constrainedStates = new ArrayList<ConstrainedState>(points.size());
        var predecessor =
            new ConstrainedState(
                points.get(0),
                0,
                startVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq);

        // Translational movement
        // Forward pass
        for (int i = 0; i < points.size(); i++) {
            constrainedStates.add(new ConstrainedState());
            var constrainedState = constrainedStates.get(i);
            constrainedState.pose = points.get(i);

            // Begin constraining based on predecessor.
            double ds =
                constrainedState
                    .pose
                    .poseMeters
                    .getTranslation()
                    .getDistance(predecessor.pose.poseMeters.getTranslation());
            constrainedState.distanceMeters = predecessor.distanceMeters + ds;

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global
                // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
                constrainedState.maxVelocityMetersPerSecond =
                    Math.min(
                        maxVelocityMetersPerSecond,
                        Math.sqrt(
                            predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond
                                + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0));

                constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
                constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;

                // At this point, the constrained state is fully constructed apart from
                // all the custom-defined user constraints.
                for (final var constraint : constraints) {
                    constrainedState.maxVelocityMetersPerSecond =
                        Math.min(
                            constrainedState.maxVelocityMetersPerSecond,
                            constraint.getMaxVelocityMetersPerSecond(
                                constrainedState.pose.poseMeters,
                                constrainedState.pose.curvatureRadPerMeter,
                                constrainedState.maxVelocityMetersPerSecond));
                    }

                    // Now enforce all acceleration limits.
                    enforceAccelerationLimits(reversed, constraints, constrainedState);

                    if (ds < 1E-6) {
                        break;
                    }

                    // If the actual acceleration for this state is higher than the max
                    // acceleration that we applied, then we need to reduce the max
                    // acceleration of the predecessor and try again.
                    double actualAcceleration =
                        (constrainedState.maxVelocityMetersPerSecond
                                * constrainedState.maxVelocityMetersPerSecond
                                - predecessor.maxVelocityMetersPerSecond
                                * predecessor.maxVelocityMetersPerSecond)
                            / (ds * 2.0);

                    // If we violate the max acceleration constraint, let's modify the
                    // predecessor.
                    if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1E-6) {
                        predecessor.maxAccelerationMetersPerSecondSq =
                            constrainedState.maxAccelerationMetersPerSecondSq;
                    } else {
                    // Constrain the predecessor's max acceleration to the current
                    // acceleration.
                    if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
                        predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration;
                    }
                    // If the actual acceleration is less than the predecessor's min
                    // acceleration, it will be repaired in the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }

        // // Rotational movement
        // // Forward pass
        // for (int i = 0; i < points.size(); i++) {
        //     constrainedStates.add(new ConstrainedState());
        //     var constrainedState = constrainedStates.get(i);
        //     constrainedState.pose = points.get(i);

        //     // Begin constraining based on predecessor.
        //     double a = constrainedState.pose.poseMeters.getRotation().getRadians();
        //     double b = predecessor.pose.poseMeters.getRotation().getRadians();
        //     // Smallest difference between angles
        //     double da = Math.min(Math.min(Math.abs(a-b), Math.abs(2*Math.PI - a + b)), Math.abs(2*Math.PI - b + a));
        //     constrainedState.angleRadians = predecessor.angleRadians + da;

        //     // We may need to iterate to find the maximum end velocity and common
        //     // acceleration, since acceleration limits may be a function of velocity.
        //     while (true) {
        //         // Enforce global max velocity and max reachable velocity by global
        //         // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
        //         constrainedState.maxAngularVelocityRadiansPerSecond =
        //             Math.min(
        //                 maxAngularVelocityRadiansPerSecond,
        //                 Math.sqrt(
        //                     predecessor.maxAngularVelocityRadiansPerSecond * predecessor.maxAngularVelocityRadiansPerSecond
        //                         + predecessor.maxAngularAccelerationRadiansPerSecondSq * da * 2.0));

        //         constrainedState.minAngularAccelerationRadiansPerSecondSq = -maxAngularAccelerationRadiansPerSecondSq;
        //         constrainedState.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;

        //         // At this point, the constrained state is fully constructed apart from
        //         // all the custom-defined user constraints.
        //         for (final var constraint : constraints) {
        //             constrainedState.maxAngularVelocityRadiansPerSecond =
        //                 Math.min(
        //                     constrainedState.maxAngularVelocityRadiansPerSecond,
        //                     constraint.getMaxAngularVelocityRadiansPerSecond(
        //                         constrainedState.pose.poseMeters,
        //                         constrainedState.maxAngularVelocityRadiansPerSecond));
        //             }

        //             // Now enforce all acceleration limits.
        //             enforceAccelerationLimits(reversed, constraints, constrainedState);

        //             if (da < 1E-6) {
        //                 break;
        //             }

        //             // If the actual acceleration for this state is higher than the max
        //             // acceleration that we applied, then we need to reduce the max
        //             // acceleration of the predecessor and try again.
        //             double actualAcceleration =
        //                 (constrainedState.maxAngularVelocityRadiansPerSecond
        //                         * constrainedState.maxAngularVelocityRadiansPerSecond
        //                         - predecessor.maxAngularVelocityRadiansPerSecond
        //                         * predecessor.maxAngularVelocityRadiansPerSecond)
        //                     / (da * 2.0);

        //             // If we violate the max acceleration constraint, let's modify the
        //             // predecessor.
        //             if (constrainedState.maxAngularAccelerationRadiansPerSecondSq < actualAcceleration - 1E-6) {
        //                 predecessor.maxAngularAccelerationRadiansPerSecondSq =
        //                     constrainedState.maxAngularAccelerationRadiansPerSecondSq;
        //             } else {
        //             // Constrain the predecessor's max acceleration to the current
        //             // acceleration.
        //             if (actualAcceleration > predecessor.minAngularAccelerationRadiansPerSecondSq) {
        //                 predecessor.maxAngularAccelerationRadiansPerSecondSq = actualAcceleration;
        //             }
        //             // If the actual acceleration is less than the predecessor's min
        //             // acceleration, it will be repaired in the backward pass.
        //             break;
        //         }
        //     }
        //     predecessor = constrainedState;
        // }
        

        var successor =
            new ConstrainedState(
                points.get(points.size() - 1),
                constrainedStates.get(constrainedStates.size() - 1).distanceMeters,
                endVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq);

        // Translational movement
        // Backward pass
        for (int i = points.size() - 1; i >= 0; i--) {
        var constrainedState = constrainedStates.get(i);
        double ds = constrainedState.distanceMeters - successor.distanceMeters; // negative

        while (true) {
            // Enforce max velocity limit (reverse)
            // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
            double newMaxVelocity =
                Math.sqrt(
                    successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond
                        + successor.minAccelerationMetersPerSecondSq * ds * 2.0);

            // No more limits to impose! This state can be finalized.
            if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) {
            break;
            }

            constrainedState.maxVelocityMetersPerSecond = newMaxVelocity;

            // Check all acceleration constraints with the new max velocity.
            enforceAccelerationLimits(reversed, constraints, constrainedState);

            if (ds > -1E-6) {
            break;
            }

            // If the actual acceleration for this state is lower than the min
            // acceleration, then we need to lower the min acceleration of the
            // successor and try again.
            double actualAcceleration =
                (constrainedState.maxVelocityMetersPerSecond
                            * constrainedState.maxVelocityMetersPerSecond
                        - successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond)
                    / (ds * 2.0);

            if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1E-6) {
            successor.minAccelerationMetersPerSecondSq =
                constrainedState.minAccelerationMetersPerSecondSq;
            } else {
            successor.minAccelerationMetersPerSecondSq = actualAcceleration;
            break;
            }
        }
        successor = constrainedState;
        }

        // // Rotational movement
        // // Backward pass
        // for (int i = points.size() - 1; i >= 0; i--) {
        //     var constrainedState = constrainedStates.get(i);
        //     double a = successor.pose.poseMeters.getRotation().getRadians();
        //     double b = constrainedState.pose.poseMeters.getRotation().getRadians();
        //     // Smallest difference between angles
        //     double da = Math.min(Math.min(Math.abs(a-b), Math.abs(2*Math.PI - a + b)), Math.abs(2*Math.PI - b + a));
        
        //     while (true) {
        //         // Enforce max velocity limit (reverse)
        //         // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
        //         double newMaxVelocity =
        //             Math.sqrt(
        //                 successor.maxAngularVelocityRadiansPerSecond * successor.maxAngularVelocityRadiansPerSecond
        //                     + successor.minAngularAccelerationRadiansPerSecondSq * da * 2.0);
        
        //         // No more limits to impose! This state can be finalized.
        //         if (newMaxVelocity >= constrainedState.maxAngularVelocityRadiansPerSecond) {
        //             break;
        //         }
        
        //         constrainedState.maxAngularVelocityRadiansPerSecond = newMaxVelocity;
        
        //         // Check all acceleration constraints with the new max velocity.
        //         enforceAccelerationLimits(reversed, constraints, constrainedState);
        
        //         if (da > -1E-6) {
        //             break;
        //         }
        
        //         // If the actual acceleration for this state is lower than the min
        //         // acceleration, then we need to lower the min acceleration of the
        //         // successor and try again.
        //         double actualAcceleration =
        //             (constrainedState.maxAngularVelocityRadiansPerSecond
        //                         * constrainedState.maxAngularVelocityRadiansPerSecond
        //                     - successor.maxAngularVelocityRadiansPerSecond * successor.maxAngularVelocityRadiansPerSecond)
        //                 / (da * 2.0);
        
        //         if (constrainedState.minAngularAccelerationRadiansPerSecondSq > actualAcceleration + 1E-6) {
        //             successor.minAngularAccelerationRadiansPerSecondSq =
        //                 constrainedState.minAngularAccelerationRadiansPerSecondSq;
        //         } else {
        //             successor.minAngularAccelerationRadiansPerSecondSq = actualAcceleration;
        //             break;
        //         }
        //     }
        //     successor = constrainedState;
        // }

        // Now we can integrate the constrained states forward in time to obtain our
        // trajectory states.
        var states = new ArrayList<HolonomicTrajectory.State>(points.size());
        double timeSeconds = 0.0;
        double distanceMeters = 0.0;
        double velocityMetersPerSecond = 0.0;
        // double angleRadians = points.get(0).poseMeters.getRotation().getRadians();
        // double angularVelocityRadiansPerSecond = 0.0;

        for (int i = 0; i < constrainedStates.size(); i++) {
            final var state = constrainedStates.get(i);

            // Calculate the change in position between the current state and the previous
            // state.
            double ds = state.distanceMeters - distanceMeters;

            // // Calculcate the change in angle between the current state and the previos state.
            // double da = state.angleRadians - angleRadians;

            // Calculate the acceleration between the current state and the previous
            // state.
            double accel =
                (state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond
                        - velocityMetersPerSecond * velocityMetersPerSecond)
                    / (ds * 2);

            // // Calculate the angular acceleration between the current state and the previous state.
            // double angularAccel = (state.maxAngularVelocityRadiansPerSecond * state.maxAngularVelocityRadiansPerSecond
            //             - angularVelocityRadiansPerSecond * angularVelocityRadiansPerSecond)
            //         / (da * 2);
            // // This is the point where I think the ability to include angular movement falls apart.  So the angle is just
            // // thrown in at each time stamp ignoring whatever calculations were made above. RIP.

            // Calculate dt based on ds
            double dt = 0.0;
            if (i > 0) {
                states.get(i - 1).accelerationMetersPerSecondSq = reversed ? -accel : accel;
                if (Math.abs(accel) > 1E-6) {
                    // v_f = v_0 + a * t
                    dt = (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel;
                } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
                    // delta_x = v * t
                    dt = ds / velocityMetersPerSecond;
                } else {
                    throw new TrajectoryGenerationException(
                        "Something went wrong at iteration " + i + " of time parameterization.");
                }
            }

            velocityMetersPerSecond = state.maxVelocityMetersPerSecond;
            distanceMeters = state.distanceMeters;
            // angleRadians = state.angleRadians;

            timeSeconds += dt;

            states.add(
                new HolonomicTrajectory.State(
                    timeSeconds,
                    reversed ? -velocityMetersPerSecond : velocityMetersPerSecond,
                    reversed ? -accel : accel,
                    state.pose.poseMeters,
                    new Rotation2d(),
                    state.pose.curvatureRadPerMeter));
        }

        return new HolonomicTrajectory(states);
    }

    
    /**
     * Parameterize the trajectory by time. This is where the velocity profile is generated.
     *
     * <p>The derivation of the algorithm used can be found <a
     * href="http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf">here</a>.
     *
     * @param points Reference to the spline points.
     * @param constraints A vector of various velocity and acceleration. constraints.
     * @param startVelocityMetersPerSecond The start velocity for the trajectory.
     * @param endVelocityMetersPerSecond The end velocity for the trajectory.
     * @param maxVelocityMetersPerSecond The max velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
     * @param reversed Whether the robot should move backwards. Note that the robot will still move
     *     from a -&gt; b -&gt; ... -&gt; z as defined in the waypoints.
     * @return The trajectory.
     */
    public static HolonomicTrajectory timeParameterizeHolonomicTrajectory(
            List<PoseWithCurvature> points,
            List<Rotation2d> headings,
            List<TrajectoryConstraint> constraints,
            double startVelocityMetersPerSecond,
            double endVelocityMetersPerSecond,
            double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq,
            boolean reversed) {
        var constrainedStates = new ArrayList<ConstrainedState>(points.size());
        var predecessor =
            new ConstrainedState(
                points.get(0),
                0,
                startVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq);

        // Translational movement
        // Forward pass
        for (int i = 0; i < points.size(); i++) {
            constrainedStates.add(new ConstrainedState());
            var constrainedState = constrainedStates.get(i);
            constrainedState.pose = points.get(i);

            // Begin constraining based on predecessor.
            double ds =
                constrainedState
                    .pose
                    .poseMeters
                    .getTranslation()
                    .getDistance(predecessor.pose.poseMeters.getTranslation());
            constrainedState.distanceMeters = predecessor.distanceMeters + ds;

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global
                // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
                constrainedState.maxVelocityMetersPerSecond =
                    Math.min(
                        maxVelocityMetersPerSecond,
                        Math.sqrt(
                            predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond
                                + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0));

                constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
                constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;

                // At this point, the constrained state is fully constructed apart from
                // all the custom-defined user constraints.
                for (final var constraint : constraints) {
                    constrainedState.maxVelocityMetersPerSecond =
                        Math.min(
                            constrainedState.maxVelocityMetersPerSecond,
                            constraint.getMaxVelocityMetersPerSecond(
                                constrainedState.pose.poseMeters,
                                constrainedState.pose.curvatureRadPerMeter,
                                constrainedState.maxVelocityMetersPerSecond));
                    }

                    // Now enforce all acceleration limits.
                    enforceAccelerationLimits(reversed, constraints, constrainedState);

                    if (ds < 1E-6) {
                        break;
                    }

                    // If the actual acceleration for this state is higher than the max
                    // acceleration that we applied, then we need to reduce the max
                    // acceleration of the predecessor and try again.
                    double actualAcceleration =
                        (constrainedState.maxVelocityMetersPerSecond
                                * constrainedState.maxVelocityMetersPerSecond
                                - predecessor.maxVelocityMetersPerSecond
                                * predecessor.maxVelocityMetersPerSecond)
                            / (ds * 2.0);

                    // If we violate the max acceleration constraint, let's modify the
                    // predecessor.
                    if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1E-6) {
                        predecessor.maxAccelerationMetersPerSecondSq =
                            constrainedState.maxAccelerationMetersPerSecondSq;
                    } else {
                    // Constrain the predecessor's max acceleration to the current
                    // acceleration.
                    if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
                        predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration;
                    }
                    // If the actual acceleration is less than the predecessor's min
                    // acceleration, it will be repaired in the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }        

        var successor =
            new ConstrainedState(
                points.get(points.size() - 1),
                constrainedStates.get(constrainedStates.size() - 1).distanceMeters,
                endVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq);

        // Translational movement
        // Backward pass
        for (int i = points.size() - 1; i >= 0; i--) {
        var constrainedState = constrainedStates.get(i);
        double ds = constrainedState.distanceMeters - successor.distanceMeters; // negative

        while (true) {
            // Enforce max velocity limit (reverse)
            // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
            double newMaxVelocity =
                Math.sqrt(
                    successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond
                        + successor.minAccelerationMetersPerSecondSq * ds * 2.0);

            // No more limits to impose! This state can be finalized.
            if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) {
            break;
            }

            constrainedState.maxVelocityMetersPerSecond = newMaxVelocity;

            // Check all acceleration constraints with the new max velocity.
            enforceAccelerationLimits(reversed, constraints, constrainedState);

            if (ds > -1E-6) {
            break;
            }

            // If the actual acceleration for this state is lower than the min
            // acceleration, then we need to lower the min acceleration of the
            // successor and try again.
            double actualAcceleration =
                (constrainedState.maxVelocityMetersPerSecond
                            * constrainedState.maxVelocityMetersPerSecond
                        - successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond)
                    / (ds * 2.0);

            if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1E-6) {
            successor.minAccelerationMetersPerSecondSq =
                constrainedState.minAccelerationMetersPerSecondSq;
            } else {
            successor.minAccelerationMetersPerSecondSq = actualAcceleration;
            break;
            }
        }
        successor = constrainedState;
        }

        // Now we can integrate the constrained states forward in time to obtain our
        // trajectory states.
        var states = new ArrayList<HolonomicTrajectory.State>(points.size());
        double timeSeconds = 0.0;
        double distanceMeters = 0.0;
        double velocityMetersPerSecond = 0.0;
        // double angleRadians = points.get(0).poseMeters.getRotation().getRadians();
        // double angularVelocityRadiansPerSecond = 0.0;

        for (int i = 0; i < constrainedStates.size(); i++) {
            final var state = constrainedStates.get(i);
            Rotation2d directionOfMovement = state.pose.poseMeters.getRotation();
            state.pose.poseMeters = new Pose2d(state.pose.poseMeters.getTranslation(), headings.get(i));

            // Calculate the change in position between the current state and the previous
            // state.
            double ds = state.distanceMeters - distanceMeters;

            // Calculate the acceleration between the current state and the previous
            // state.
            double accel =
                (state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond
                        - velocityMetersPerSecond * velocityMetersPerSecond)
                    / (ds * 2);

            // Calculate dt based on ds
            double dt = 0.0;
            if (i > 0) {
                states.get(i - 1).accelerationMetersPerSecondSq = reversed ? -accel : accel;
                if (Math.abs(accel) > 1E-6) {
                    // v_f = v_0 + a * t
                    dt = (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel;
                } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
                    // delta_x = v * t
                    dt = ds / velocityMetersPerSecond;
                } else {
                    throw new TrajectoryGenerationException(
                        "Something went wrong at iteration " + i + " of time parameterization.");
                }
            }

            velocityMetersPerSecond = state.maxVelocityMetersPerSecond;
            distanceMeters = state.distanceMeters;

            timeSeconds += dt;

            states.add(
                new HolonomicTrajectory.State(
                    timeSeconds,
                    reversed ? -velocityMetersPerSecond : velocityMetersPerSecond,
                    reversed ? -accel : accel,
                    state.pose.poseMeters,
                    // new Pose2d(state.pose.poseMeters.getTranslation(), heading),
                    directionOfMovement,
                    state.pose.curvatureRadPerMeter));
        }

        return new HolonomicTrajectory(states);
    }

    private static void enforceAccelerationLimits(
            boolean reverse, List<TrajectoryConstraint> constraints, ConstrainedState state) {
        for (final var constraint : constraints) {
            // Translational movement
            double factor = reverse ? -1.0 : 1.0;
            final var minMaxAccel =
                constraint.getMinMaxAccelerationMetersPerSecondSq(
                    new Pose2d(state.pose.poseMeters.getTranslation(), new Rotation2d(Math.PI)),
                    state.pose.curvatureRadPerMeter,
                    state.maxVelocityMetersPerSecond * factor);

            if (minMaxAccel.minAccelerationMetersPerSecondSq
                > minMaxAccel.maxAccelerationMetersPerSecondSq) {
                throw new TrajectoryGenerationException(
                    "The constraint's min acceleration "
                        + "was greater than its max acceleration.\n Offending Constraint: "
                        + constraint.getClass().getName()
                        + "\n If the offending constraint was packaged with WPILib, please file a bug"
                        + " report.");
            }

            state.minAccelerationMetersPerSecondSq =
                Math.max(
                    state.minAccelerationMetersPerSecondSq,
                    reverse
                        ? -minMaxAccel.maxAccelerationMetersPerSecondSq
                        : minMaxAccel.minAccelerationMetersPerSecondSq);

            state.maxAccelerationMetersPerSecondSq =
                Math.min(
                    state.maxAccelerationMetersPerSecondSq,
                    reverse
                        ? -minMaxAccel.minAccelerationMetersPerSecondSq
                        : minMaxAccel.maxAccelerationMetersPerSecondSq);


        //     // Rotational movement
        //     final var minMaxAngularAccel =
        //     constraint.getMinMaxAngularAccelerationRadiansPerSecondSq(
        //         state.pose.poseMeters,
        //         state.maxAngularVelocityRadiansPerSecond);

        //     if (minMaxAngularAccel.minAngularAccelerationRadiansPerSecondSq
        //         > minMaxAngularAccel.maxAngularAccelerationRadiansPerSecondSq) {
        //         throw new TrajectoryGenerationException(
        //             "The constraint's min acceleration "
        //                 + "was greater than its max acceleration.\n Offending Constraint: "
        //                 + constraint.getClass().getName()
        //                 + "\n If the offending constraint was packaged with WPILib, please file a bug"
        //                 + " report.");
        //     }

        //     state.minAngularAccelerationRadiansPerSecondSq =
        //         Math.max(
        //             state.minAngularAccelerationRadiansPerSecondSq,
        //             minMaxAngularAccel.minAngularAccelerationRadiansPerSecondSq);

        //     state.maxAccelerationMetersPerSecondSq =
        //         Math.min(
        //             state.maxAccelerationMetersPerSecondSq,
        //             minMaxAngularAccel.maxAngularAccelerationRadiansPerSecondSq);
        }
    }

    @SuppressWarnings("MemberName")
    private static class ConstrainedState {
        PoseWithCurvature pose;
        double distanceMeters;
        double maxVelocityMetersPerSecond;
        double minAccelerationMetersPerSecondSq;
        double maxAccelerationMetersPerSecondSq;

        ConstrainedState(
                PoseWithCurvature pose,
                double distanceMeters,
                double maxVelocityMetersPerSecond,
                double minAccelerationMetersPerSecondSq,
                double maxAccelerationMetersPerSecondSq) {
            this.pose = pose;
            this.distanceMeters = distanceMeters;
            this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
            this.minAccelerationMetersPerSecondSq = minAccelerationMetersPerSecondSq;
            this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
        }

        ConstrainedState() {
            pose = new PoseWithCurvature();
        }
    }

    @SuppressWarnings("serial")
    public static class TrajectoryGenerationException extends RuntimeException {
        public TrajectoryGenerationException(String message) {
            super(message);
        }
    }
}
