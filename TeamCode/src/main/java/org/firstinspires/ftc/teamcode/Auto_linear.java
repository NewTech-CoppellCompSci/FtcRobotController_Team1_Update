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

import android.os.SystemClock;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "red ducks", group = "Jackson")
//@Disabled
public class Auto_linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Rev2mDistanceSensor sideBackDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx susanWheel;
    private DcMotorEx armSlide;

    private final int tolerance = 30;
    private final double maxTime = 5;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POW = 1.0;     // Maximum rotational position
    static final double MIN_POW = 0.0;     // Minimum rotational position\
    double power = .5;
    double position = .5;// Start at halfway position

    @Override
    public void runOpMode() {


        wheelFL = hardwareMap.get(DcMotorEx.class, "left_fwd_drive");
        wheelFR = hardwareMap.get(DcMotorEx.class, "right_fwd_drive");
        wheelBL = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        wheelBR = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        susanWheel = hardwareMap.get(DcMotorEx.class, "susanWheel");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");

        sideBackDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSideLeft");
        sideRightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSideRight");

        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        susanWheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelFR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);


        waitForStart();
        //travel(0, 2100, -1050);
        //travel(0, 1500, 1000);
        //turn(0, 1000, 850);

//        turn(0, 2100, 850);
//        travel(90, 2100, 850);
//        turn(0, 2100, 850);
//        travel(90, 2100, 850);
//        turn(0, 2100, 850);
//        travel(90, 2100, 850);
        //rotateRight(10,5);
      travelUntilDistanceAway(-90, .7, sideRightDistanceSensor,30);
      travelUntilDistance(0, .65, sideBackDistanceSensor, 15);
      travelUntilDistance(90, .35, sideRightDistanceSensor, 27);
      lazySusan(700, -1200);
      travelUntilDistanceAway(-90, .7, sideRightDistanceSensor,30);
      travelUntilDistanceAway(180, .7, sideBackDistanceSensor, 100);
      turn(0,1000,850);
      travelUntilDistance(90, .7,sideRightDistanceSensor,25);
      travel(90,2000,4000);





    }

    public void travel(int angle, int power, int target){
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //true angle
        double trueAngle = (angle - 90);

        //number of 45 degree angles from 0 (unit Circle) (in radians)
        double num45s = Math.toRadians(trueAngle / 45);
//        if (angle == 0){
//            num45s = 7;
//        }


        //calculates main angle
        double r = Math.hypot(Math.cos(Math.toRadians(trueAngle)), Math.sin(Math.toRadians(trueAngle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(trueAngle)), Math.cos(Math.toRadians(trueAngle))) - ((Math.PI / 4) + ((Math.PI / 4) * num45s));

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = r * Math.cos(robotAngle);

        //sets target position
        wheelFL.setTargetPosition(v1 > 0 ? target : -1 * target); //v1 > 0 ? target : -1 * target
        wheelFR.setTargetPosition(v2 > 0 ? target : -1 * target);
        wheelBL.setTargetPosition(v3 > 0 ? target : -1 * target);
        wheelBR.setTargetPosition(v4 > 0 ? target : -1 * target);

        // Switch to RUN_TO_POSITION mode
        wheelFL.setTargetPositionTolerance(tolerance);
        wheelFR.setTargetPositionTolerance(tolerance);
        wheelBL.setTargetPositionTolerance(tolerance);
        wheelBR.setTargetPositionTolerance(tolerance);
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        //change the power for each wheel
        //this should make it turn at v1-v4 ticks per second.

        wheelFL.setVelocity(v1 * power);
        wheelFR.setVelocity(v2 * power);
        wheelBL.setVelocity(v3 * power);
        wheelBR.setVelocity(v4 * power);

        telemetry.addData("power ", "%.1f power", v1);
        telemetry.addData("power ", "%.1f power", v2);
        telemetry.addData("power ", "%.1f power", v3);
        telemetry.addData("power ", "%.1f power", v4);
        telemetry.update();

        while(wheelFL.isBusy()  ||  wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.addData("Back Left", wheelBL.getCurrentPosition());
            telemetry.addData("Back Right", wheelBR.getCurrentPosition());
            telemetry.addData("Front Left", wheelFL.getCurrentPosition());
            telemetry.addData("Front Right", wheelFR.getCurrentPosition());
            telemetry.update();


        }
        telemetry.addData("Status", "done");
    }

    public void travelUntilDistance(int angle, double power, Rev2mDistanceSensor sensor, int distanceInCm) {
        double r = Math.hypot(Math.cos(Math.toRadians(angle)), Math.sin(Math.toRadians(angle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))) - Math.PI / 4;
        double rightX = 0;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;


        wheelFL.setPower(-v1 * power);
        wheelFR.setPower(-v2 * power);
        wheelBL.setPower(v3 * power);
        wheelBR.setPower(v4 * power);

        while (true) {
            if (sensor.getDistance(DistanceUnit.CM) <= distanceInCm) {
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                break;
            }
        }

    }

    public void travelUntilDistanceAway(int angle, double power, Rev2mDistanceSensor sensor, int distanceInCm) {
        double r = Math.hypot(Math.cos(Math.toRadians(angle)), Math.sin(Math.toRadians(angle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))) - Math.PI / 4;
        double rightX = 0;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;


        wheelFL.setPower(-v1 * power);
        wheelFR.setPower(-v2 * power);
        wheelBL.setPower(v3 * power);
        wheelBR.setPower(v4 * power);

        while (true) {
            if (sensor.getDistance(DistanceUnit.CM) >= distanceInCm) {
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                break;
            }
        }
    }

    public void turn(int angle, int power, int target){
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.FORWARD);

        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //true angle
        double trueAngle = (angle - 90);

        //number of 45 degree angles from 0 (unit Circle) (in radians)
        double num45s = Math.toRadians(trueAngle / 45);
//        if (angle == 0){
//            num45s = 7;
//        }


        //calculates main angle
        double r = Math.hypot(Math.cos(Math.toRadians(trueAngle)), Math.sin(Math.toRadians(trueAngle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(trueAngle)), Math.cos(Math.toRadians(trueAngle))) - ((Math.PI / 4) + ((Math.PI / 4) * num45s));

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle);
        final double v2 = r * Math.sin(robotAngle);
        final double v3 = r * Math.sin(robotAngle);
        final double v4 = r * Math.cos(robotAngle);

        //sets target position
        wheelFL.setTargetPositionTolerance(tolerance);
        wheelFR.setTargetPositionTolerance(tolerance);
        wheelBL.setTargetPositionTolerance(tolerance);
        wheelBR.setTargetPositionTolerance(tolerance);
        wheelFL.setTargetPosition(v1 > 0 ? target : -1 * target); //v1 > 0 ? target : -1 * target
        wheelFR.setTargetPosition(v2 > 0 ? target : -1 * target);
        wheelBL.setTargetPosition(v3 > 0 ? target : -1 * target);
        wheelBR.setTargetPosition((v4 > 0 ? target : -1 * target));

        // Switch to RUN_TO_POSITION mode
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //change the power for each wheel
        //this should make it turn at v1-v4 ticks per second.

        wheelFL.setVelocity(v1 * power);
        wheelFR.setVelocity(v2 * power);
        wheelBL.setVelocity(v3 * power);
        wheelBR.setVelocity(v4 * power);

        telemetry.addData("power ", "%.1f power", v1);
        telemetry.addData("power ", "%.1f power", v2);
        telemetry.addData("power ", "%.1f power", v3);
        telemetry.addData("power ", "%.1f power", v4);
        telemetry.update();


        double startRuntime = runtime.seconds();
        while((wheelFL.isBusy()  ||  wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()) || (runtime.seconds() - startRuntime >= maxTime) ) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.addData("Back Left", wheelBL.getCurrentPosition());
            telemetry.addData("Back Right", wheelBR.getCurrentPosition());
            telemetry.addData("Front Left", wheelFL.getCurrentPosition());
            telemetry.addData("Front Right", wheelFR.getCurrentPosition());
            telemetry.addData("startTime", startRuntime);
            telemetry.addData("currentTime", runtime.seconds());
            telemetry.addLine( ""+(runtime.seconds() - startRuntime));
            telemetry.update();


        }
        telemetry.addData("Status", "done");
    }


    public void lazySusan(int power, int target) {
        susanWheel.setTargetPosition(target);
        susanWheel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        susanWheel.setVelocity(power);
        while(susanWheel.isBusy()){

        }
    }

}

