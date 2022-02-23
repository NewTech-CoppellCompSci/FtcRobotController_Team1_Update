package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "red_park", group = "Trajectory", preselectTeleOp = "DriverControl_Pressme;")
public class RedWarehouseTrajectorySaianshOneMove extends LinearOpMode {

    private DcMotor Lift;


    public static int liftVelo = 1500;
    public static int LowL = 260;
    public static int DownL = 0;



    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift = hardwareMap.get(DcMotor.class, "armSlide");


        //start bot at pose x = -30, y = -64, heading 90 degrees
        Pose2d startPose = new Pose2d(8, -64, Math.toRadians(0));

        drive.setPoseEstimate(startPose);





        //trajectory0
        TrajectorySequence Trajectory0 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(40, -66, Math.toRadians(0))) //move to storage tower thing
                //.splineTo(new Vector2d(16, -64), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    Lift.setTargetPosition(LowL);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) Lift).setVelocity(liftVelo);
                    Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }) //Raise lift to top position
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> Lift.setTargetPosition(DownL)) //Lower lift
                .build();


        waitForStart();
        if(!isStopRequested())
            drive.followTrajectorySequence(Trajectory0);
    }
    }
