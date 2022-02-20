package org.firstinspires.ftc.teamcode.objectstuffforwebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.objectstuffforwebcam.Gobiidae;

@Autonomous(name="Calibrate Camera Threshold", group="Special")
public class Calibrationthing extends LinearOpMode {

    Gobiidae cameraSymbiote = new Gobiidae();

    @Override
    public void runOpMode(){
        cameraSymbiote.setup(hardwareMap);

        waitForStart();

        while(!isStopRequested()){
            cameraSymbiote.calibrateRoutine();
            telemetry.addData("Thres Calc ", Storage.STORED_THRESHOLD);
            telemetry.update();
            sleep(10);
        }
    }
}