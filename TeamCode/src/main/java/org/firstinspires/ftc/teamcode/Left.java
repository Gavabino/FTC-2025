package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Left", group = "Autonomous", preselectTeleOp = "Main")
public class Left extends LinearOpMode {
    public class Lift {
        private final DcMotorEx lift;
        private final Servo clawArm;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "slide");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawArm = hardwareMap.get(Servo.class, "clawArm");
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.75);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2000.0) {
                    return true;
                } else {
                    lift.setPower(0);
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
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3400.0) {
                    return true;
                } else {
                    lift.setPower(0);
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
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1400.0) {
                    return true;
                } else {
                    lift.setPower(0);
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
                    clawArm.setPosition(0.4);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    clawArm.setPosition(0.15);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Park {
        private final DcMotorEx slider;

        public Park(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, "climb1");
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setDirection(DcMotorSimple.Direction.FORWARD);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slider.setPower(-0.75);
                    initialized = true;
                }

                double pos = slider.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -3000) {
                    return true;
                } else {
                    slider.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
    }

    public class Claw {
        private final Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.35);
                sleep(1000);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.05);
                sleep(1000);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class ClawArm {
        private final Servo clawArm;
        private final ClawRotation clawRotation = new ClawRotation(hardwareMap);

        public ClawArm(HardwareMap hardwareMap) {
            clawArm = hardwareMap.get(Servo.class, "clawArm");
        }

        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawArm.setPosition(0.65);
                sleep(1000);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmMiddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawArm.setPosition(0.4);
                sleep(1000);
                return false;
            }
        }
        public Action armMiddle() {
            return new ArmMiddle();
        }

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawArm.setPosition(0.20);
                sleep(1000);
                return false;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }
    }

    public class ClawRotation {
        private final Servo clawRotation;

        public ClawRotation(HardwareMap hardwareMap) {
            clawRotation = hardwareMap.get(Servo.class, "clawRotation");
        }

        public class ClawDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawRotation.setPosition(0.51);
                sleep(1000);
                return false;
            }
        }
        public Action clawDown() {
            return new ClawDown();
        }

        public class ClawUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawRotation.setPosition(0.61);
                sleep(1000);
                return false;
            }
        }
        public Action clawUp() {
            return new ClawUp();
        }
    }

    public class Bucket {
        private final Servo bucket;

        public Bucket(HardwareMap hardwareMap) {
            bucket = hardwareMap.get(Servo.class, "bucketServo");
        }

        public class ResetBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(0.613);
                sleep(1000);
                return false;
            }
        }
        public Action resetBucket() {
            return new ResetBucket();
        }

        public class EmptyBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(0.69);
                sleep(1000);
                return false;
            }
        }
        public Action emptyBucket() {
            return new EmptyBucket();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(36.96, 63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Park slider = new Park(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawArm clawArm = new ClawArm(hardwareMap);
        Bucket bucketServo = new Bucket(hardwareMap);
        ClawRotation clawRotation = new ClawRotation(hardwareMap);

        TrajectoryActionBuilder place = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-4.66, 35.34, Math.toRadians(90.00)), Math.toRadians(270.00));

        TrajectoryActionBuilder reverse = place.endTrajectory().fresh()
                .strafeTo(new Vector2d(-4.66, 31));

        TrajectoryActionBuilder forward = reverse.endTrajectory().fresh()
                .lineToY(50);

        TrajectoryActionBuilder pickUp = forward.endTrajectory().fresh()
                .splineTo(new Vector2d(16.55, 49.66), Math.toRadians(-24.70))
                .splineTo(new Vector2d(46.48, 43.5), Math.toRadians(270.00));

        TrajectoryActionBuilder bucket = pickUp.endTrajectory().fresh()
                .turnTo(Math.toRadians(220))
                .strafeTo(new Vector2d(57.5, 62));

        TrajectoryActionBuilder park = bucket.endTrajectory().fresh()
                .splineTo(new Vector2d(28.79, -9.64), Math.toRadians(180.00));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        place.build(),
                        lift.liftUp(),
                        reverse.build(),
                        lift.liftPlace(),
                        forward.build(),
                        lift.liftDown(),
                        pickUp.build(),
                        clawRotation.clawDown(),
                        claw.openClaw(),
                        clawArm.armDown(),
                        claw.closeClaw(),
                        new ParallelAction(
                                bucketServo.resetBucket(),
                                claw.closeClaw(),
                                new SequentialAction(
                                        clawArm.armUp(),
                                        clawRotation.clawUp(),
                                        claw.openClaw()
                                )
                        ),
                        bucket.build(),
                        clawRotation.clawDown(),
                        lift.liftFull(),
                        bucketServo.emptyBucket()

                )
        );
    }
}