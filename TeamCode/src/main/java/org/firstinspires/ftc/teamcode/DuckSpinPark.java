package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Spin & Park", group="Jackson")

public class DuckSpinPark extends OpMode{
    private Rev2mDistanceSensor backDistanceSensor;
    private Rev2mDistanceSensor sideDistanceSensor;
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx susanWheel;
    private DcMotorEx armSlide;
    private Servo gripper;





    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POW     =  1.0;     // Maximum rotational position
    static final double MIN_POW   =  0.0;     // Minimum rotational position\
    double power = .5;
    double  position = .5;// Start at halfway position


    @Override
    public void init(){
        wheelFL  = hardwareMap.get(DcMotorEx.class, "left_fwd_drive");
        wheelFR  = hardwareMap.get(DcMotorEx.class, "right_fwd_drive");
        wheelBL  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        wheelBR  = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        susanWheel = hardwareMap.get(DcMotorEx.class, "susanWheel");
        armSlide  = hardwareMap.get(DcMotorEx.class, "armSlide");
        gripper = hardwareMap.get(Servo.class,"gripperServo");

        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBack");
        sideDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSide");

        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        susanWheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelFL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

    }


    //0 goes backward
    // 90 goes right
    //180 goes foward
    // 270 goest left

    public void travel(int angle, int power, int target){
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

    //0 goes backward
    // 90 goes right
    //180 goes foward
    // 270 goest left

    public void travelUntilDistance(int angle, double power, Rev2mDistanceSensor sensor, int distanceInCm){
        double r = Math.hypot(Math.cos(Math.toRadians(angle)), Math.sin(Math.toRadians(angle)));
        double robotAngle = Math.atan2(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle))) - Math.PI / 4;
        double rightX = 0;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        wheelFL.setPower(-v1 * power);
        wheelFR.setPower(-v2 * power);
        wheelBL.setPower(v3 * power);
        wheelBR.setPower(v4 * power);

        while(true){
            if(sensor.getDistance(DistanceUnit.CM) <= distanceInCm){
                wheelFL.setPower(0);
                wheelFR.setPower(0);
                wheelBL.setPower(0);
                wheelBR.setPower(0);
                break;
            }
        }


    }

    public void rotate(double rotatePower, int target){

        final double v1 = rotatePower;
        final double v2 = rotatePower;
        final double v3 = rotatePower;
        final double v4 = rotatePower;

        wheelFL.setTargetPosition(target);
        wheelFR.setTargetPosition(target);
        wheelBL.setTargetPosition(target);
        wheelBR.setTargetPosition(target);

        wheelFL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        wheelFL.setVelocity(v1);
        wheelFR.setVelocity(v2);
        wheelBL.setVelocity(v3);
        wheelBR.setVelocity(v4);

    }

    public void lazySusan(int power, int target){
        susanWheel.setTargetPosition(target);
        susanWheel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        susanWheel.setVelocity(power);
    }


    public void slideArm()  {

//        armSlide.setPosition(1);
//        z

//        if(gamepad1.dpad_up){
//            armSlide.setPosition(1);
//            SystemClock.sleep(7000);
//            armSlide.setPosition(.5) ;
//        }else if (gamepad1.dpad_down){
//            armSlide.setPosition(0);
//            SystemClock.sleep(7000);
//            armSlide.setPosition(.5) ;
//        }
        //armSlide.setP
    }


    @Override
    public void start() {
        armSlide.setVelocity(650);
        armSlide.setTargetPosition(400);

        rotate(1000, 1000);
        //travel(180,700,2100);
        //lazySusan(1000, 1000);
//        travelUntilDistance(-90, .5, sideDistanceSensor, 55);
        //slideArm();








        telemetry.addData("deviceName","backSensor" );
        telemetry.addData("range", String.format("%.3f cm", backDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range edited", backDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("deviceName", "sideSensor" );
        telemetry.addData("range", String.format("%.3f cm", sideDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range edited", sideDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();




    }

    @Override
    public void stop() {
        armSlide.setTargetPosition(0);
    }
}
