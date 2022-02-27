package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.objectstuffforwebcam.Gobiidae;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "blue_score_parkRR", group = "Trajectory", preselectTeleOp = "DriverControl_Pressme;")
public class blueWarehouseTrajectorySaiansh extends LinearOpMode {

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

//    Gobiidae pipeline;
//   pipeline = new Gobiidae();
//        pipeline.setup(hardwareMap);


    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift = hardwareMap.get(DcMotor.class, "armSlide");
        IntakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        IntakeRight = hardwareMap.get(DcMotor.class, "intakeRight");

        Gobiidae pipeline;
        pipeline = new Gobiidae();
        pipeline.setup(hardwareMap);

        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(8, 64, Math.toRadians(180));

        drive.setPoseEstimate(startPose);





        //trajectory0
        TrajectorySequence Trajectory0 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-10, 42, Math.toRadians(180))) //move to storage tower thing
                //.splineTo(new Vector2d(16, -64), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Lift.setTargetPosition(LowL);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) Lift).setVelocity(liftVelo);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }) //Raise lift to top position
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeRight.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeLeft.setPower(-1))//Lower lift
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(LowL)) //Lower lift
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeRight.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeLeft.setPower(0))//Lower lift
                .lineToLinearHeading(new Pose2d(10, 66, Math.toRadians(180))) //First move and turn to warehouse
                .splineTo(new Vector2d(40, 66), Math.toRadians(180)) //Second move moving straight in warehouse
                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(DownL)) //Lower lift
                .build();

        //Trajectory1
        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-10, 42, Math.toRadians(180))) //move to storage tower thing
                //.splineTo(new Vector2d(16, -64), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Lift.setTargetPosition(LowL);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) Lift).setVelocity(liftVelo);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }) //Raise lift to top position
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> Lift.setTargetPosition(MidL)) //Lower lift
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeRight.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeLeft.setPower(-1))//Lower lift
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(LowL)) //Lower lift
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeRight.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> IntakeLeft.setPower(0))//Lower lift
                .lineToLinearHeading(new Pose2d(10, 66, Math.toRadians(180))) //First move and turn to warehouse
                .splineTo(new Vector2d(40, 66), Math.toRadians(180)) //Second move moving straight in warehouse
                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(DownL)) //Lower lift
                .build();

        //Trajectory2
        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-10, 42, Math.toRadians(180))) //move to storage tower thing
                //.splineTo(new Vector2d(16, -64), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Lift.setTargetPosition(LowL);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) Lift).setVelocity(liftVelo);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }) //Raise lift to top position
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> Lift.setTargetPosition(TopL)) //Lower lift
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> IntakeRight.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> IntakeLeft.setPower(-1))//Lower lift
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> Lift.setTargetPosition(LowL)) //Lower lift
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> IntakeRight.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> IntakeLeft.setPower(0))//Lower lift
                .strafeLeft(8)//tune with field testing
                .forward(48)
                //.lineToLinearHeading(new Pose2d(10, 66, Math.toRadians(180))) //First move and turn to warehouse
                //.splineTo(new Vector2d(40, 66), Math.toRadians(180)) //Second move moving straight in warehouse
                //.lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(DownL)) //Lower lift
                .build();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("results", results);
            telemetry.update();
            results = pipeline.getAnalysis();
        }
        waitForStart();
        switch (results) {
            case 1:
                drive.followTrajectorySequence(Trajectory0);
                break;
            case 2:
                drive.followTrajectorySequence(Trajectory1);
                break;
            case 3:
                drive.followTrajectorySequence(Trajectory2);
                break;
        }
        while (opModeIsActive()) {
            // telemetry.addData("placement]", Pipeline.Last);
            // telemetry.update();
            //sleep(50);
        }
    }


}