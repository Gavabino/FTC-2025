/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    public class Presets {
        private final DcMotor armSlider;
        private final Servo clawRotationVertical;
        private final Servo topClaw;
        private final Servo clawOperation;
        private final Servo armServo;

        public Presets(HardwareMap hardwareMap) {
            armSlider = hardwareMap.get(DcMotor.class, "armSlider");
            armSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSlider.setDirection(DcMotorSimple.Direction.FORWARD);
            clawRotationVertical = hardwareMap.get(Servo.class, "clawRotationVertical");
            topClaw = hardwareMap.get(Servo.class, "topClaw");
            clawOperation = hardwareMap.get(Servo.class, "clawOperation");
            armServo = hardwareMap.get(Servo.class, "armServo");
        }

        public class ResetSlider implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //flip claw
                //rotate claw
                //bring in slider
                if (!initialized) {
                    armSlider.setPower(-1);
                    initialized = true;
                }

                double pos = armSlider.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) {
                    return true;
                } else {
                    armSlider.setPower(0);
                    sleep(500);
                    return false;
                }
            }
        }

        public Action resetSlider() {
            return new ResetSlider();
        }

        public class ResetVerticalClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawRotationVertical.setPosition(0.9);
                return false;
            }
        }

        public Action resetVerticalClaw() {
            return new ResetVerticalClaw();
        }

        public class OpenArmClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawOperation.setPosition(0.35);
                return false;
            }
        }

        public Action openArmClaw() {
            return new OpenArmClaw();
        }

        public class CloseTopClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.35);
                sleep(100);
                return false;
            }
        }

        public Action closeTopClaw() {
            return new CloseTopClaw();
        }

        public class OpenTopClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                topClaw.setPosition(0.05);
                sleep(100);
                return false;
            }
        }

        public Action openTopClaw() {
            return new OpenTopClaw();
        }

        public class PlacingClawPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(0.69);
                return false;
            }
        }

        public Action placingClawPosition() {
            return new PlacingClawPosition();
        }

        public class GrabbingClawPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(0.565);
                return false;
            }
        }

        public Action grabbingClawPosition() {
            return new GrabbingClawPosition();
        }


    }

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "rearLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rearRight");
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        DcMotor slide2 = hardwareMap.get(DcMotor.class, "slide2");
        DcMotor armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        Servo clawOperation = hardwareMap.get(Servo.class, "clawOperation");
        Servo armServo = hardwareMap.get(Servo.class, "armServo");
        CRServo clawRotationHorizontal = hardwareMap.get(CRServo.class, "clawRotationHorizontal");
        Servo topClaw = hardwareMap.get(Servo.class, "topClaw");
        Servo clawRotationVertical = hardwareMap.get(Servo.class, "clawRotationVertical");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Presets presets = new Presets(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            double slideZeroPower = -0.1;
            while (opModeIsActive()) {

                //Gamepad 1

                if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x > 0.5) {
                    rightFront.setPower(-0.75);
                    leftFront.setPower(0);
                    rightRear.setPower(0);
                    leftRear.setPower(-0.75);
                } else if (gamepad1.left_stick_y > 0.5 && gamepad1.left_stick_x < -0.5) {
                    rightFront.setPower(0);
                    leftFront.setPower(-0.75);
                    rightRear.setPower(-0.75);
                    leftRear.setPower(0);
                } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x < -0.5) {
                    rightFront.setPower(0.75);
                    leftFront.setPower(0);
                    rightRear.setPower(0);
                    leftRear.setPower(0.75);
                } else if (gamepad1.left_stick_y < -0.5 && gamepad1.left_stick_x > 0.5) {
                    rightFront.setPower(0);
                    leftFront.setPower(0.75);
                    rightRear.setPower(0.75);
                    leftRear.setPower(0);
                } else if (gamepad1.left_stick_y > 0.5) {
                    rightFront.setPower(-0.75);
                    leftFront.setPower(-0.75);
                    rightRear.setPower(-0.75);
                    leftRear.setPower(-0.75);
                } else if (gamepad1.left_stick_y < -0.5) {
                    rightFront.setPower(0.75);
                    leftFront.setPower(0.75);
                    rightRear.setPower(0.75);
                    leftRear.setPower(0.75);
                } else if (gamepad1.left_stick_x > 0.5) {
                    rightFront.setPower(-0.75);
                    leftFront.setPower(0.75);
                    rightRear.setPower(0.75);
                    leftRear.setPower(-0.75);
                } else if (gamepad1.left_stick_x < -0.5) {
                    rightFront.setPower(0.75);
                    leftFront.setPower(-0.75);
                    rightRear.setPower(-0.75);
                    leftRear.setPower(0.75);
                } else if (gamepad1.right_bumper) {
                    rightFront.setPower(-0.75);
                    leftFront.setPower(0.75);
                    rightRear.setPower(-0.75);
                    leftRear.setPower(0.75);
                } else if (gamepad1.left_bumper) {
                    rightFront.setPower(0.75);
                    leftFront.setPower(-0.75);
                    rightRear.setPower(0.75);
                    leftRear.setPower(-0.75);
                } else {
                    rightFront.setPower(0);
                    leftFront.setPower(0);
                    rightRear.setPower(0);
                    leftRear.setPower(0);
                }
                if (gamepad1.right_stick_y > 0.5) {
                    slide.setPower(-1);
                    slide2.setPower(-1);
                } else if (gamepad1.right_stick_y < -0.5) {
                    slide.setPower(1);
                    slide2.setPower(1);
                } else {
                    slide.setPower(slideZeroPower);
                    slide2.setPower(slideZeroPower);
                }
                if (gamepad1.a) {
                    slideZeroPower = -0.75;
                }

                //Gamepad 2

                if (gamepad2.right_stick_button) {
                    armServo.setPosition(0.69);
                }

                if (gamepad2.left_stick_button) {
                    armServo.setPosition(0.565);
                }

                if (gamepad2.left_trigger > 0.5) {
                    topClaw.setPosition(0.35);
                }
                if (gamepad2.right_trigger > 0.5) {
                    topClaw.setPosition(0.05);
                }

                if (gamepad2.right_stick_y > 0.5) {
                    armSlider.setPower(-1);
                } else if (gamepad2.right_stick_y < -0.5) {
                    armSlider.setPower(1);
                } else {
                    armSlider.setPower(0);
                }

                if (gamepad2.dpad_up) {
                    clawRotationVertical.setPosition(0.90);
                } else if (gamepad2.dpad_down) {
                    clawRotationVertical.setPosition(0.10);
                }
                if (gamepad2.dpad_left) {
                    clawRotationHorizontal.setPower(0.30);
                } else if (gamepad2.dpad_right) {
                    clawRotationHorizontal.setPower(-0.30);
                } else {
                    clawRotationHorizontal.setPower(0);
                }

                if (gamepad2.left_bumper) {
                    clawOperation.setPosition(0.35);
                } else if (gamepad2.right_bumper) {
                    clawOperation.setPosition(0.05);
                }
                if (gamepad2.x) {
                    Actions.runBlocking(new ParallelAction(
                            presets.resetSlider(),
                            presets.resetVerticalClaw(),
                            presets.grabbingClawPosition(),
                            presets.openTopClaw()
                    ));
                }
                if (gamepad2.a) {
                    Actions.runBlocking(new SequentialAction(
                            presets.closeTopClaw(),
                            presets.openArmClaw(),
                            presets.placingClawPosition()
                    ));
                }
            }
        }
    }
}
