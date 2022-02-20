package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objectstuffforwebcam.Gobiidae;
@Autonomous(name = "redscorepark", group = "Concept")
public class RED_Score_Park extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx susanWheel;
    private DcMotorEx armSlide;
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;
    private Rev2mDistanceSensor backDistanceSensor;
    private Rev2mDistanceSensor rightDistanceSensor;
    private Rev2mDistanceSensor leftDistanceSensor;
    private double maxDriveTicsPerSec = 2000;

    private ElapsedTime runtime = new ElapsedTime();
    private int[] armLevelPosition = {0, 260, 650, 995};
    private int armLevel = 0;
    private int results;


    @Override
    public void runOpMode() {

        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);


        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        susanWheel = hardwareMap.get(DcMotorEx.class, "susanWheel");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
//        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBack");
//        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
//        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlide.setTargetPosition(0);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set wheel tolerance
        //JMAC Copy
        wheelFL.setTargetPositionTolerance(25);
        wheelFR.setTargetPositionTolerance(25);
        wheelBL.setTargetPositionTolerance(25);
        wheelBR.setTargetPositionTolerance(25);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                wheelFL.getCurrentPosition(),
                wheelFR.getCurrentPosition(),
                wheelBL.getCurrentPosition(),
                wheelBR.getCurrentPosition());
        telemetry.update();

        armSlide.setTargetPosition(armLevelPosition[armLevel]);
        armSlide.setTargetPositionTolerance(10);


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


        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("results", results);
            telemetry.update();
            results = pipeline.getAnalysis();

        }
        waitForStart();
        switch (results) {
            case 1:
                //do everything for 1
                armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlide.setTargetPosition(armLevelPosition[1]);
                armSlide.setPower(.5);

                break;
            case 2:
                //do everything for 2
                armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlide.setTargetPosition(armLevelPosition[2]);
                armSlide.setPower(.5);

                break;
            case 3:
                armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlide.setTargetPosition(armLevelPosition[3]);
                armSlide.setPower(.5);
                break;
            //do everything for 3
        }
        route1();
        sleep(500);
    }

    public void drive(double leftX, double leftY, double rightX, int tics, String routDescription) {

        //switch x and y since the robot is sideways
        double temp = leftX;
        leftX = leftY;
        leftY = -temp;

        //reset all wheels to 0
        wheelFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Mimic controller input
        double r = Math.hypot(leftY, leftX);
        double robotAngle = Math.atan2(leftX, leftY) - Math.PI / 4;

        //make calculations based upon the input
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //set target position
        wheelFL.setTargetPosition(v1 > 0 ? tics : -tics);
        wheelFR.setTargetPosition(v2 > 0 ? tics : -tics);
        wheelBL.setTargetPosition(v3 > 0 ? tics : -tics);
        wheelBR.setTargetPosition(v4 > 0 ? tics : -tics);

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

        while (wheelFL.isBusy() || wheelFR.isBusy()) {
        }

    }

    private void route1() {

        drive(0, 1, 0, 700, "Drive Forward");

        drive(-1, 0, 0, 970, "Drive Left");


        intakeLeft.setPower(-1);
        intakeRight.setPower(1);
        sleep(2000);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        armLevel = 1;

    }
}

