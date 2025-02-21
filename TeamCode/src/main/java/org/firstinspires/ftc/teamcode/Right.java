package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "Right", group = "Autonomous", preselectTeleOp = "Main")
public class Right extends LinearOpMode {
    public class Lift {
        private final DcMotorEx lift;
        private final DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "slide");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2 = hardwareMap.get(DcMotorEx.class, "slide2");
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.75);
                    lift2.setPower(0.75);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftFull implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.75);
                    lift2.setPower(0.75);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3500.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }

        public Action liftFull() {
            return new LiftFull();
        }

        public class LiftPlace implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.75);
                    lift2.setPower(-0.75);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1600.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    return false;
                }
            }
        }

        public Action liftPlace() {
            return new LiftPlace();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    lift2.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 25.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                    sleep(500);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-12.07, 63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder place = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-4, 35.34, Math.toRadians(90.00)), Math.toRadians(270.00), new TranslationalVelConstraint(60));

        TrajectoryActionBuilder reverse = place.endTrajectory().fresh()
                .strafeTo(new Vector2d(-4.66, 31), new TranslationalVelConstraint(60));

        TrajectoryActionBuilder forward = reverse.endTrajectory().fresh()
                .lineToY(50, new TranslationalVelConstraint(60));

        TrajectoryActionBuilder pickUp = forward.endTrajectory().fresh()
                .splineTo(new Vector2d(-37, 40), Math.toRadians(-24.70), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(-38.57, 26), Math.toRadians(270), new TranslationalVelConstraint(30))
                .splineTo(new Vector2d(-50, 10), Math.toRadians(270.00), new TranslationalVelConstraint(30));

        TrajectoryActionBuilder bucket = pickUp.endTrajectory().fresh()
                .turnTo(Math.toRadians(220))
                .strafeTo(new Vector2d(57, 61.5), new TranslationalVelConstraint(60));

        TrajectoryActionBuilder park = bucket.endTrajectory().fresh()
                .splineTo(new Vector2d(28.79, -9.64), Math.toRadians(180.00));

        TrajectoryActionBuilder push1 = forward.endTrajectory().fresh()
                .splineTo(new Vector2d(-20.00, 43.65), Math.toRadians(225.10), new TranslationalVelConstraint(60))
                .splineTo(new Vector2d(-27.39, 20.65), Math.toRadians(256.22), new TranslationalVelConstraint(60))
                .splineToConstantHeading(new Vector2d(-44, 15), Math.toRadians(280), new TranslationalVelConstraint(60))
                .strafeTo(new Vector2d(-46, 22.27), new TranslationalVelConstraint(60))
                .lineToY(50)
                .waitSeconds(0.25)
                .lineToY(64);

        TrajectoryActionBuilder push2 = push1.endTrajectory().fresh()
                .splineTo(new Vector2d(-50, -3.5), Math.toRadians(280))
                .strafeTo(new Vector2d(-50, 22.27))
                .lineToY(62.93);

        TrajectoryActionBuilder place2 = push1.endTrajectory().fresh()
                .lineToY(60)
                .splineToLinearHeading(new Pose2d(-8, 39.34, Math.toRadians(90.00)), Math.toRadians(270.00));

        TrajectoryActionBuilder pick1 = forward.endTrajectory().fresh()
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50.94, 60), Math.toRadians(90))
                .waitSeconds(0.25)
                .lineToY(64);

        TrajectoryActionBuilder place3 = pick1.endTrajectory().fresh()
                .lineToY(60)
                .splineToLinearHeading(new Pose2d(-12, 39.34, Math.toRadians(90.00)), Math.toRadians(270.00));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                place.build(),
                                lift.liftUp()
                        ),
                        reverse.build(),
                        lift.liftPlace(),
                        new ParallelAction(
                                forward.build(),
                                lift.liftDown()
                        ),
                        push1.build(),
                        new ParallelAction(
                                lift.liftUp(),
                                new SequentialAction(
                                        place2.build(),
                                        lift.liftUp()
                                )
                        ),
                        reverse.build(),
                        lift.liftPlace(),
                        new ParallelAction(
                                forward.build(),
                                lift.liftDown()
                        ),
                        pick1.build(),
                        new ParallelAction(
                                lift.liftUp(),
                                new SequentialAction(
                                        place3.build(),
                                        lift.liftUp()
                                )
                        ),
                        reverse.build(),
                        lift.liftPlace(),
                        new ParallelAction(
                                forward.build(),
                                lift.liftDown()
                        )

                )
        );
    }
}