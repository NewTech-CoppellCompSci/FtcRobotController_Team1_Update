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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Encoder Testing Auton", group="Hunter")
//@Disabled

public class Auto_Encoder_Test extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
   Declare motors to type DcMotorEx

   Documentation:
   https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
    */
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;

    /*
        max speed for the drive motors.
        I think max is 600
     */
    private double maxDriveTicsPerSec = 5000;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFL  = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR  = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL  = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR  = hardwareMap.get(DcMotorEx.class, "wheelBR");


        /*
            Set up motors so they run without the encoders
            This way they run freely.  They won't go to a specific position or count the number of rotations
            It will now run with range from -1.0 to 1.0

            See Documentation for other encoder modes
            https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
         */
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // In RUN_USING_ENCODER mode, they all need to run in reverse
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Encoder Tolerance
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

        //set PIDFCoefficients
        wheelFL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelFR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBL.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        wheelBR.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        rout1();

    }

    private void rout1(){

        drive(0, 1, 0, 1000, "Drive Forward");
        while(wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()){}

        drive(1, 0, 0, 1000, "Drive Right");
        while(wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()){}

        drive(.5, .5, 0, 1000, "Forward Left");
        while(wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()){}

        drive(0, 0, 1, 1000, "Spin Right");
        while(wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()){}

        drive(.5, .5, .05, 1000, "curve");
        while(wheelFL.isBusy() || wheelFR.isBusy() || wheelBL.isBusy() || wheelBR.isBusy()){}

    }





    /*
        you're mimicking the joystick controls
        leftX = left stick x position -1 <= x <= 1
        leftY = left stick y position -1 <= y <= 1
        rightX = right stick x position -1 <= x <= 1
        tics = max number of tics
     */
    public void drive(double leftX, double leftY, double rightX, int tics, String routDescription){

        //switch x and y since the robot is sideways
        double temp = leftX;
        leftX = leftY;
        leftY = temp;

        //reset all wheels to 0
        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Mimic controller input
        double r = Math.hypot(leftX, leftY);
        double robotAngle = Math.atan2(leftY, rightX) - Math.PI / 4;

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //set target position
        wheelFL.setTargetPosition(v1 > 0 ? (int) (tics * v1) : (int) (-1 * tics * v1));
        wheelFR.setTargetPosition(v2 > 0 ? (int) (tics * v2) : (int) (-1 * tics * v2));
        wheelBL.setTargetPosition(v3 > 0 ? (int) (tics * v3) : (int) (-1 * tics * v3));
        wheelBR.setTargetPosition(v4 > 0 ? (int) (tics * v4) : (int) (-1 * tics * v4));

        //Set wheel encoder mode
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //set the velocity for each wheel
        wheelFL.setVelocity(v1 * -1 * maxDriveTicsPerSec);
        wheelFR.setVelocity(v2 * -1 * maxDriveTicsPerSec);
        wheelBL.setVelocity(v3 * -1 * maxDriveTicsPerSec);
        wheelBR.setVelocity(v4 * -1 * maxDriveTicsPerSec);

        telemetry.addData("Task: ", routDescription);
        telemetry.update();

    }


}

