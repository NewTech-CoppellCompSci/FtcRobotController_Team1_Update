package org.firstinspires.ftc.teamcode;





        import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import java.util.List;
        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
//@Disabled
public class AUTO_Barcode extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
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
    private int armLevel;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference // change
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUlS8ez/////AAABmUD5bSVHq09euUA9G90rLAuFcxoeq0o0q4nvbHVinlVTs1fJrhxYGtR79DO28ebtRUhApSZAbLMEZT9nnY5W/4Yu45HIZrVDirUdyle8uldP/VDFlB/V0vvt5ynDhdCPwvzFHaaGJXP6mSLxPcBcUARm/+yd8xf/+Q2zEeYLwdP8102j4QoSiRnSp/SDj6nCmeas9GWU6RhP46G4pkRKBKqGL+UiSIMKAeVpYohGOoNi4SykQp4apse+X/VvGYIdvHVwgjX4fvqraYpXR0CwjY4K3zdR6S4ps/2L2l7XWw4nuAjsRUwf+JMnJ8D1P+lF65Won+1MigCqVtDo1MELiuWA0LEjCs0aPEVih/dctWre";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public static VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public static TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");
        susanWheel = hardwareMap.get(DcMotorEx.class, "susanWheel");
        armSlide = hardwareMap.get(DcMotorEx.class, "armSlide");
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBack");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set wheel tolerance
        //JMAC Copy
        wheelFL.setTargetPositionTolerance(25);
        wheelFR.setTargetPositionTolerance(25);
        wheelBL.setTargetPositionTolerance(25);
        wheelBR.setTargetPositionTolerance(25);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                wheelFL.getCurrentPosition(),
                wheelFR.getCurrentPosition(),
                 wheelBL.getCurrentPosition(),
                wheelBR.getCurrentPosition());
        telemetry.update();




        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0 / 9.0);
        }

        armLevel = 0;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        detectDuck();

        rout1();


    }

    public void detectDuck(){

        int duckPos = 0;

        if (opModeIsActive()) {
            if (getPosition() == "Left") { //if duck is on the left barcode
                armLevel = 1;
                duckPos = 1;

            } else if (getPosition() == "Right") { //if duck is on the right barcode
                armLevel = 2;
                duckPos = 2;
            } else { // if the duck is on the barcode that the camera doesn't see
                armLevel = 3;
                duckPos = 3;

            }
            telemetry.addData("Duck Level: ", duckPos);
            armSlide.setTargetPosition(armLevelPosition[armLevel]);
            armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlide.setPower(0.7);
            while (Math.abs(armLevelPosition[armLevel] - armSlide.getCurrentPosition()) > 20 && opModeIsActive()) {
//                armSlide.setVelocity(1500); //removed because
            }

        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public static String getPosition(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel().equals("Duck")){
                        boolean pose = checkPose(recognition.getLeft());
                        if(pose) return "Right";
                        else return "Left";
                    }

                    i++;
                }
            }
        }
        return "None";
    }
    public static boolean checkPose(double pixel){
        if(pixel >= 180) return true; // on the right
        else return false; //on the left
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = wheelFL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = wheelFR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            wheelFL.setTargetPosition(newLeftTarget);
            wheelFR.setTargetPosition(newRightTarget);
            wheelBL.setTargetPosition(newLeftTarget);
            wheelBR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            wheelFL.setPower(Math.abs(speed));
            wheelFR.setPower(Math.abs(speed));
            wheelBL.setPower(Math.abs(speed));
            wheelBR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelFL.isBusy() && wheelFR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelFL.getCurrentPosition(),
                        wheelFR.getCurrentPosition(),
                        wheelBL.getCurrentPosition(),
                        wheelBR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            wheelFL.setPower(0);
            wheelFR.setPower(0);
            wheelBL.setPower(0);
            wheelBR.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            armSlide.setTargetPosition(armLevelPosition[armLevel]);
            armSlide.setTargetPositionTolerance(armLevelPosition[armLevel]);


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















            //  sleep(250);   // optional pause after each move
        }
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
        wheelFL.setTargetPosition(v1 > 0 ? tics : - tics);
        wheelFR.setTargetPosition(v2 > 0 ? tics : - tics);
        wheelBL.setTargetPosition(v3 > 0 ? tics : - tics);
        wheelBR.setTargetPosition(v4 > 0 ? tics : - tics);

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

        while(wheelFL.isBusy() || wheelFR.isBusy()){}

    }


    /*
        you're mimicking the joystick controls
        leftX = left stick x position -1 <= x <= 1
        leftY = left stick y position -1 <= y <= 1
        rightX = right stick x position -1 <= x <= 1
        tics = max number of tics
     */
    public void driveDistnce(double leftX, double leftY, double rightX, int sensorNum, int distance, String routDescription){
        Rev2mDistanceSensor sensor;
        switch (sensorNum)
        {
            case 1:
                sensor = rightDistanceSensor;
                break;
            case 2:
                sensor = leftDistanceSensor;
                break;
            case 3:
                sensor = backDistanceSensor;
                break;
            default:
                sensor = backDistanceSensor;

        }
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

        //Set wheel encoder mode
        wheelFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //set the velocity for each wheel
        wheelFL.setVelocity(v1 * -1 * maxDriveTicsPerSec);
        wheelFR.setVelocity(v2 * -1 * maxDriveTicsPerSec);
        wheelBL.setVelocity(v3 * -1 * maxDriveTicsPerSec);
        wheelBR.setVelocity(v4 * -1 * maxDriveTicsPerSec);

        //add loop sensor timer
        while (Math.abs(sensor.getDistance(DistanceUnit.CM) - distance) < 0.5){
            telemetry.addData("Task: ", routDescription);
            telemetry.addData("Distance: ", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }


        wheelFL.setVelocity(0);
        wheelFR.setVelocity(0);
        wheelBL.setVelocity(0);
        wheelBR.setVelocity(0);


    }

//    public void encoderDrive(){
//        //encoderdrive stuff goes here
//        encoderDrive(DRIVE_SPEED, 3, 3, 50.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
//        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
//        encoderDrive(DRIVE_SPEED, 3, 3, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
//        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
//        encoderDrive(DRIVE_SPEED, 3, 3, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        intakeLeft.setPower(-1);
//        intakeRight.setPower(1);
//        sleep(2000);
//        intakeLeft.setPower(0);
//        intakeRight.setPower(0);
//
//        //removed for drivestcik encodinbg
//
//        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//
//        intakeLeft.setPower(-1);
//        intakeRight.setPower(-1);
//        sleep(2000);
//        intakeLeft.setPower(0);
//        intakeRight.setPower(0);
//
//        encoderDrive(DRIVE_SPEED, 48, 48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
//        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//
//
//        //paste outtake
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//
//        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        // robot.rightClaw.setPosition(0.0);//not needed
//        //sleep(1000);     // pause for servos to move
//    }



    private void rout1() {

        drive(0, 1, 0, 700, "Drive Forward");

        drive(-1, 0, 0, 970, "Drive Left");

        driveDistnce(0,1,0,3, 50, "Move to Hub");

        intakeLeft.setPower(-1);
        intakeRight.setPower(1);
        sleep(2000);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        //drive(.5, .5, 0, 1000, "Forward Left");

        //drive(0, 0, 1, 1000, "Spin Right");

        // drive(.5, 0, .05, 1000, "curve");



    }

}
//afdter all this set arm to 0 and then