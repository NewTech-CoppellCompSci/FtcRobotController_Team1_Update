package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.objectstuffforwebcam.Gobiidae;

@Disabled
@Autonomous(name = "redLEFTautoNEWwithducks", group = "Trajectory", preselectTeleOp = "DriverControl_Pressme;")
public class redLEFTautowithducks extends LinearOpMode {

    private DcMotor Lift;
    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;


    public static double intaking = 1;
    public static int liftVelo = 2000;
    public int TopL;// = 995;
    public int MidL;// = 650;
    public int LowL;// = 260;
    public int DownL;// = 0;
    public int ZERO_POSITION;
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
        Lift.setTargetPosition(Lift.getCurrentPosition());
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ZERO_POSITION = Lift.getCurrentPosition();
        TopL = ZERO_POSITION + 995;
        MidL = ZERO_POSITION + 650;
        LowL = ZERO_POSITION + 260;
        DownL = ZERO_POSITION;



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

        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(LowL);

        detection();
        duckscore();
        scoreFreight();
        warehousePark();


    }

    private void detection() {
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

    private void scoreFreight() {
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(36, -42, Math.toRadians(0))) //move to storage tower thing
                .build());
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(level == 1 ? LowL : level == 2 ? MidL : TopL);
        sleep(2000);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(8)
                .build());
        sleep(3000);
        autoReleaseFreight();
    }

    private void warehousePark() {
        ((DcMotorEx) Lift).setVelocity(1000);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build());
        Lift.setTargetPosition(LowL);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(65)
                .build());
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(DownL);
        sleep(30000);
    }

    private void autoReleaseFreight() {
        IntakeRight.setPower(1);
        IntakeLeft.setPower(-1);
        sleep(2000);
        IntakeRight.setPower(0);
        IntakeLeft.setPower(0);
    }


    public void duckscore () {

        ((DcMotorEx) Lift).setVelocity(1000);
        Lift.setTargetPosition(LowL);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
        .strafeLeft(8)
                .build());
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(45)
                .build());

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(65)
                .build());
        ((DcMotorEx) Lift).setVelocity(liftVelo);
        Lift.setTargetPosition(DownL);
        sleep(30000);





    }


}