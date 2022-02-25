package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.objectstuffforwebcam.Gobiidae;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "testurmom", group = "Trajectory", preselectTeleOp = "DriverControl_Pressme;")
public class yotest extends LinearOpMode {

    private DcMotor Lift;
    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;


    public static double intaking = 1;
    public static int liftVelo = 2000;
    public static int TopL = 995;
    public static int MidL = 650;
    public static int LowL = 260;
    public static int DownL = 0;
    public static int Wall1 = -65;
    public static int Wall2 = -66;
    private int results;
    public int level;
    public SampleMecanumDrive drive;

//    Gobiidae pipeline;
//   pipeline = new Gobiidae();
//        pipeline.setup(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Lift = hardwareMap.get(DcMotor.class, "armSlide");
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setTargetPosition(LowL);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        IntakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        IntakeRight = hardwareMap.get(DcMotor.class, "intakeRight");

        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);

        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(8, -64, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("results", results);
            telemetry.update();
            results = pipeline.getAnalysis();
        }

        waitForStart();

        detection();
        scoreFreight();
        warehousePark();

//        while (opModeIsActive()) {
//            // telemetry.addData("placement]", Pipeline.Last);
//            // telemetry.update();
//            //sleep(50);
//        }
    }

    private void detection(){
        switch (results) {
            case 1:
                level = 1;
                break;
            case 2:
                level = 2;
                break;
            case 3:
                level = 3;
                break;
        }
    }

    private void scoreFreight(){
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-10, -40, Math.toRadians(0))) //move to storage tower thing
                .build());
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(level == 1 ? LowL : level == 2 ? MidL : TopL);
        sleep(1500);
        autoReleaseFreight();
    }

    private void warehousePark(){
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(LowL);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(72)
                .build());
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(DownL);
        sleep(2000);
    }

    private void autoReleaseFreight(){
        IntakeRight.setPower(1);
        IntakeLeft.setPower(-1);
        sleep(2000);
        IntakeRight.setPower(0);
        IntakeLeft.setPower(0);
    }


}