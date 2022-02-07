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


@Autonomous(name = "Auto Red Park", group = "Jackson")
//@Disabled
public class AutoRedPark extends OpMode {

    @Override
    public void start() {

        rotateLeft(5000,1000);
        //rotateRight(10,5);
//      travelUntilDistanceAway(-90, .5, sideRightDistanceSensor,50);
//      travelUntilDistance(0, .65, sideBackDistanceSensor, 15);
//      travelUntilDistance(90, .5, sideRightDistanceSensor, 30);
//      lazySusan(500, -2500);
    }


    private Rev2mDistanceSensor sideBackDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx susanWheel;
    private DcMotorEx armSlide;


    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POW = 1.0;     // Maximum rotational position
    static final double MIN_POW = 0.0;     // Minimum rotational position\
    double power = .5;
    double position = .5;// Start at halfway position


    @Override
    public void init() {
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


    }

    @Override
    public void loop() {

    }

    public void travel(int angle, int power, int target) {
        double r = Math.hypot(Math.cos(Math.toRadians(angle)), Math.sin(Math.toRadians(angle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))) - Math.PI / 4;
        double rightX = 0;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        wheelFL.setTargetPosition(-target);
        wheelFR.setTargetPosition(-target);
        wheelBL.setTargetPosition(target);
        wheelBR.setTargetPosition(target);

        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        wheelFL.setVelocity(v1 * power);
        wheelFR.setVelocity(v2 * power);
        wheelBL.setVelocity(v3 * power);
        wheelBR.setVelocity(v4 * power);


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

    public void travelUntilDistanceAway(int angle, double power,  Rev2mDistanceSensor sensor, int distanceInCm) {
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
    public void rotateLeft(int power, int target){
        wheelFL.setTargetPosition(target);
        wheelFR.setTargetPosition(-target);
        wheelBL.setTargetPosition(-target);
        wheelBR.setTargetPosition(target);
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFL.setVelocity(power);
        wheelFR.setVelocity(power);
        wheelBL.setVelocity(power);
        wheelBR.setVelocity(power);
    }

    public void rotateRight(int power, int target){
        wheelFL.setTargetPosition(-target);
        wheelFR.setTargetPosition(target);
        wheelBL.setTargetPosition(target);
        wheelBR.setTargetPosition(-target);
        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFL.setVelocity(power);
        wheelFR.setVelocity(power);
        wheelBL.setVelocity(power);
        wheelBR.setVelocity(power);
    }
    public void lazySusan(int power, int target) {
        susanWheel.setTargetPosition(target);
        susanWheel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        susanWheel.setVelocity(power);
    }




    @Override
    public void stop() {
        telemetry.addData("STOPPED!!!", "HEHEHEHA!");
        telemetry.update();
    }

}




