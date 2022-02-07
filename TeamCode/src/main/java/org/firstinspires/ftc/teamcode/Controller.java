/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
/*
    @Autonomous = this is for Autonomous mode
    @TeleOp = this is for User Controlled mode

    name = the name that will display on the Driver Hub
    group = allows you to group OpModes
 */
@TeleOp(name="DriverControl", group="Wesley")
//@Disabled  This way it will run on the robot
public class Controller extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx

    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */

    //Touch Sensors
    //private DigitalChannel intakeSensor;

    //Motors
    private Rev2mDistanceSensor sideLeftDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx susanWheel;
    private DcMotorEx armSlide;
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;

    //Servos
    //private Servo gripper;

    //Variables
    private int[] armLevelPosition = {0, 260, 650, 995};
    private boolean isGrabbing = false;
    private int armLevel;
    private double speedMod;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;
    private boolean rumbleLevel = true;
    private boolean gripperToggle = false;
    private double rotation = 0;
    //double susanPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "left_fwd_drive");
        wheelFR = hardwareMap.get(DcMotorEx.class, "right_fwd_drive");
        wheelBL = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        wheelBR = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        susanWheel = hardwareMap.get(DcMotorEx.class, "susanWheel");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeWheelLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeWheelRight");

        //Touch Sensors
        //intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeTouchSensor");

        //Distance Sensors
//        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBack");
//        sideDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSide");

        //Servos
        //gripper = hardwareMap.get(Servo.class, "gripperServo");

        sideLeftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSideLeft");
        sideRightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSideRight");
        /*
            Set up motors so they run without the encoders
            This way they run freely.  They won't go to a specific position or count the number of rotations
            It will now run with range from -1.0 to 1.0

            See Documentation for other encoder modes
            https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
         */

        //Motor Encoders
        //Wheels
        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Lazy Suzan Wheel
        susanWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Intake Wheels
        intakeLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Arm Slide (Forklift) Encoder
        armSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setTargetPosition(260);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setTargetPositionTolerance(50);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized balls");
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        previousRunTime = getRuntime();


//        //move entire robot forward at 0.5 speed
//        wheelFL.setPower(-0.5);
//        wheelFR.setPower(-0.5);
//        wheelBL.setPower(0.5);
//        wheelBR.setPower(0.5);
//
//        //loop for 3 seconds
//        while (runtime.milliseconds() <= 3000){}
//
//        wheelFL.setPower(0);
//        wheelFR.setPower(0);
//        wheelBL.setPower(0);
//        wheelBR.setPower(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        //Mecanum Wheels Code
        //https://ftccats.github.io/software/ProgrammingMecanumWheels.html

        //Get game controller input

        double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);

        //make calculations based upon the input
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        rotation += 1 * rightX;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        //change the power for each wheel
        wheelFL.setPower(-v1 * speedMod);
        wheelFR.setPower(-v2 * speedMod);
        wheelBL.setPower(v3 * speedMod);
        wheelBR.setPower(v4 * speedMod);

        //Lazy Susan
        double susanPower = 0;

        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
            speedMod = .25;
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            speedMod = 0.5;
            gamepad1.rumble(.5, .5, 1000);
            gamepad2.rumble(.5, .5, 1000);

        } else {
            speedMod = 1;
//            gamepad1.stopRumble();
//            gamepad2.stopRumble();

        }


        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            susanWheel.setPower(.6);
            susanPower = 1.0;
            telemetry.addData("Status", "dpad left");
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            susanWheel.setPower(-.6);
            telemetry.addData("Status", "bumper right");
        } else {
            susanWheel.setPower(0);
        }

        //Grabber
//

        //Intake Wheels
        // if (intakeSensor.getState()) {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        //gamepad2.rumble(1000);
        // }
        // else{
        if (gamepad2.left_bumper) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad2.right_bumper) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
        //  }

        /*
        Arm Slide/Elevator
        Level of the Arm Slide 0-3
        */
//        if (gamepad1.dpad_up || gamepad2.dpad_up) {
//            armSlide.setTargetPosition(armLevelPosition[3]);
//        }
//        if (gamepad1.dpad_right || gamepad2.dpad_up) {
//            armSlide.setTargetPosition(armLevelPosition[2]);
//        }
//        if (gamepad1.dpad_left || gamepad2.dpad_up) {
//            armSlide.setTargetPosition(armLevelPosition[1]);
//        }
//        if (gamepad1.dpad_down || gamepad2.dpad_down) {
//            armSlide.setTargetPosition(armLevelPosition[0]);
//            armSlide.setVelocity(650);
//        }
        if ((gamepad1.dpad_up || gamepad2.dpad_up) && (armLevel < armLevelPosition.length - 1) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {
            rumbleLevel = true;
            previousRunTime = getRuntime();
            armLevel++;
        }
        if ((gamepad1.dpad_down || gamepad2.dpad_down) && (armLevel > 0) && (getRuntime() - previousRunTime >= inputDelayInSeconds)) {
            rumbleLevel = true;
            previousRunTime = getRuntime();
            armLevel--;
        }
        //armSlide.setPower(100);
        armSlide.setVelocity(1000);


            //Gripper Data

            if (getRuntime() - previousRunTime >= inputDelayInSeconds + .25 && rumbleLevel) {
                rumbleLevel = false;
            }
            armSlide.setTargetPosition(armLevelPosition[armLevel]);
            armSlide.setTargetPositionTolerance(armLevelPosition[armLevel]);


            telemetry.addData("Left Trigger Position", gamepad1.left_trigger);


            //Arm Slide Data
            telemetry.addData("velocity", armSlide.getVelocity());
            telemetry.addData("slidePosition", armSlide.getCurrentPosition());
            telemetry.addData("is at target", !armSlide.isBusy());
            //Arm Slide Data
            telemetry.addData("Target Slide Position", armLevelPosition[armLevel]);
            telemetry.addData("Slide Position", armSlide.getCurrentPosition());
            telemetry.addData("Velocity", armSlide.getVelocity());
            telemetry.addData("is at target", !armSlide.isBusy());
            telemetry.addData("Tolerance: ", armSlide.getTargetPositionTolerance());

            // Show the elapsed game time and power for each wheel.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "wheelFL (%.2f), front right (%.2f), back left (%.2f),  right (%.2f)", wheelFL, wheelFR, wheelBL, wheelBR);

//        telemetry.addData("range", String.format("%.3f cm", sideDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range edited", sideDistanceSensor.getDistance(DistanceUnit.CM));

            telemetry.update();
        }


        /*
         * Code to run ONCE after the driver hits STOP
         */

        /*
         * Code to run ONCE after the driver hits STOP
         */



    //@Override
    void Stop(){
        armSlide.setTargetPosition(0);
    }
}
