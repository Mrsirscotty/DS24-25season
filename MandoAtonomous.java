package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name="JanRedRight", group="Linear Opmode")

public class JanRedRight extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor lift = null;
    private DcMotor slide = null;
    private Servo claw = null;

    private BNO055IMU       imu         = null; // Control/Expansion Hub IMU
    
    /*// huskyLens 
    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    // huskyLens end*/

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    
        // NeveRest 20 Ticks Per Revolution: 28 * 20 = 560
        // NeveRest 40 Ticks Per Revolution: 28 * 40 = 1120 https://www.andymark.com/products/neverest-classic-40-gearmotor
        // NeveRest 60 Ticks Per Revolution: 28 * 60 = 1680 https://www.andymark.com/products/neverest-classic-60-gearmotor

        // Gobuilda - assume kit https://www.gobilda.com/strafer-chassis-kit-v5/
        // Encoder Resolution    537.7 PPR at the Output Shaft https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.3;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;
    
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
 
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    DistanceSensor distance;
    DistanceSensor distanceside;
    
    private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    /*//declare LED lights
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;*/

    
    @Override
    public void runOpMode() {
                // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        
        /*// huskyLens 
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) 
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        // huskyLens end*/
        
        /*//AprilTag
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        */
        //other
        //double distanceToAprilTag = 0;

        // Initialize the Apriltag Detection process
        //initAprilTag();
        //AprilTag end
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //blinkinLedDriver = hardwareMap.get (RevBlinkinLedDriver.class, "blinkin");
        
        backleft  = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.dcMotor.get("slide");
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("claw");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        distanceside = hardwareMap.get(DistanceSensor.class, "distanceside");


        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.REVERSE);

        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Starting at",  "%7d :%7d", backleft.getCurrentPosition(), backright.getCurrentPosition());
        telemetry.update();
        
        int iLocation = 0;
        
        
        
        
        /*pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                blinkinLedDriver.setPattern(pattern);*/
                
        //camera
        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        //telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        claw.setPosition(0.545);
        //claw.setPosition(0.97);
        //DcMotorEncoder(25, 0);
        //waitForStart();
                
        while (!isStarted()) {
            /*iLocation = 0;
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                String str = "";
                telemetry.addData("Block", blocks[i].toString());
                str = blocks[i].toString();
                String delims = "[ :,]+";
                String[] tokens = str.split(delims);
                iLocation = Integer.parseInt(tokens[4]);
            }*/
            telemetry.addData(" ", "May The Force Be With You" );
            telemetry.addData(" ", " " );
            telemetry.addData("Current Angle", "%5.0f", getRawHeading());
            telemetry.addData(" ", " " );
            telemetry.addData("Distance", "%5.0f", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData(" ", " " );
            telemetry.addData("Starting at",  " FL%7d :FR%7d :BL%7d :BR%7d", frontleft.getCurrentPosition(), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
            telemetry.addData(" ", " " );
            telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.addData(" ", " " );
            telemetry.addData("range2", String.format("%.01f in", distanceside.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
        

        telemetry.addData("Location Identified", " %7d", iLocation );
        telemetry.update();
        
        /*distanceToAprilTag = driveToAprilTag();
        while (distanceToAprilTag>DESIRED_DISTANCE){
            distanceToAprilTag = driveToAprilTag();
        }*/
        
        //wall pickup
        //DcMotorEncoder(25, 300);
        //low chamber
        //DcMotorEncoder(25, 650);
        //high chamber
        
        //DriveY(-0.5,23,20);
        
        //START OF NEW CODE
        
        //Deliver specimen
        claw.setPosition(0.545);
        //encoderStraffe(1, -50, 10);
        DcMotorLiftEncoder(25, 2275);
        encoderY(1,-23,-23,20);
        encoderY(0.3,-10,-10,20);
        //could decrease time
        sleep(100);
        DcMotorLiftEncoder(25, 1500);
        encoderY(DRIVE_SPEED,-2,-2,20);
        claw.setPosition(0.5);
        //Push 1st sample
        encoderY(1,8,8,20);
        DcMotorLiftEncoder(25, 600);
        encoderStraffe(1, -1400, 10);
        encoderY(1,-25,-25,20);
        
        //push in 2 samples
        turnToHeading(1, 0.0);
        //turnToHeading(1, -90.0);
        DriveX(-0.5,22,20);
        //turnToHeading(1, -90.0);
        //StopRobot();
        //encoderStraffe(1, -2400, 10);
        encoderY(1,42,42,20);
        
        encoderStraffe(1, 300, 10);
        
        //turnToHeading(1, 0.0);
        
        //Push 2nd sample
        //encoderStraffe(DRIVE_SPEED, 2400, 10);
        //turnToHeading(DRIVE_SPEED, -90.0);
        //encoderY(DRIVE_SPEED,-10,-10,20);
        //encoderStraffe(DRIVE_SPEED, -2400, 10);
        
        //Deliver 2nd specimen
        //turnToHeading(1, -90.0);
        encoderY(1,-44,-44,20);
        turnToHeading(1, 0.0);
        //turnToHeading(1, -90.0);
        DriveX(-0.5,10.5,20);
        turnToHeading(1, 0.0);
        encoderY(1,42,42,20);
        
        encoderY(1,-15,-15,20);
        
        //score 2nd specimen
        turnToHeading(1, -180.0);
        sleep(200);
        
        
        
        //encoderY(DRIVE_SPEED,-14,-14,20);
        DcMotorLiftEncoder(25, 510);
        DriveY(-0.6,5,20);
        //encoderY(0.4,-4,-4,20);
        claw.setPosition(0.545);
        sleep(300);
        DcMotorLiftEncoder(25, 700);
        encoderY(1,10,10,20);
        encoderStraffe(1, -2800, 10);
        turnToHeading(1, 0.0);
        DcMotorLiftEncoder(25, 2275);
        sleep(200);
        //deliver 2nd specimen
        encoderY(0.9,-23.5,-23.5,20);
        //encoderY(0.3,-8,-8,20);
        sleep(100);
        DcMotorLiftEncoder(25, 1500);
        encoderY(DRIVE_SPEED,-1,-1,20);
        claw.setPosition(0.5);
        encoderY(1,8,8,20);
        DcMotorLiftEncoder(25, 700);
        //encoderStraffe(0.8, -2200, 10);
        
        DriveX(-0.8,27.5,20);
        
        //score 3rd specimen
        turnToHeading(1, -180.0);
        DcMotorLiftEncoder(25, 520);
        DriveY(-0.6,4,20);
        //encoderY(0.4,-4,-4,20);
        claw.setPosition(0.545);
        sleep(300);
        DcMotorLiftEncoder(25, 700);
        encoderY(1,10,10,20);
        encoderStraffe(1, -2200, 10);
        turnToHeading(1, 0.0);
        DcMotorLiftEncoder(25, 2275);
        sleep(200);
        //deliver 2nd specimen
        encoderY(0.9,-23.5,-23.5,20);
        //encoderY(0.3,-8,-8,20);
        sleep(100);
        DcMotorLiftEncoder(25, 1500);
        encoderY(DRIVE_SPEED,-1,-1,20);
        claw.setPosition(0.5);
        encoderY(1,10,10,20);
        
        turnToHeading(1, -180.0);
        
        StopRobot();
        
        //END OF NEW CODE
        
        /*encoderY(1,-26,-26,20);
        turnToHeading(1, -90.0);
        DriveY(-0.5,12.5,20);
        turnToHeading(1, -90.0);
        //encoderY(1,-12,-12,20);
        encoderStraffe(1, -2400, 10);
        DcMotorLiftEncoder(25, 500);
        turnToHeading(1, -180.0);
        //encoderStraffe(1, 200, 10);
        //turnToHeading(1, -180.0);
        StopRobot();*/
        
        
        /*encoderY(1,10,10,20);
        turnToHeading(DRIVE_SPEED, -180.0);
        DcMotorLiftEncoder(25, 400);
        encoderY(1,-18,-18,20);
        encoderStraffe(DRIVE_SPEED, 2500, 10);
        StopRobot();*/
        
        
        
        //Pick up and deliver 1st sample
        /*encoderY(DRIVE_SPEED,17,17,20);
        DcMotorLiftEncoder(25, 0);
        turnToHeading(DRIVE_SPEED, -90.0);
        encoderY(DRIVE_SPEED,-20.5,-20.5,20);
        turnToHeading(DRIVE_SPEED, 45.0);
        DcMotorSlideEncoder(25, 2070);
        
        intake.setPower(1);
        sleep(3000);
        intake.setPower(0);
        turnToHeading(DRIVE_SPEED, 135.0);
        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        turnToHeading(DRIVE_SPEED, 90.0);
        encoderY(DRIVE_SPEED,3,3,20);
        turnToHeading(DRIVE_SPEED, 45.0);
        
        //Pick up and deliver 2nd sample
        turnToHeading(DRIVE_SPEED, 90.0);
        turnToHeading(DRIVE_SPEED, 45.0);
        encoderY(DRIVE_SPEED,11,11,20);
        intake.setPower(1);
        sleep(3000);
        intake.setPower(0);
        turnToHeading(DRIVE_SPEED, 45.0);
        encoderY(DRIVE_SPEED,4,4,20);
        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        //1st Level Ascent
        DcMotorSlideEncoder(25, 0);
        turnToHeading(DRIVE_SPEED, 90.0);
        encoderStraffe(DRIVE_SPEED, 400, 10);
        encoderY(DRIVE_SPEED,10,10,20);
        tap.setPosition(0.975);
        StopRobot();*/
        
        /*//Pick up and deliver   specimen
        DcMotorSlideEncoder(25, 0);
        sleep(1000);
        turnToHeading(DRIVE_SPEED, -180.0);
        DcMotorLiftEncoder(25, 300);
        encoderStraffe(DRIVE_SPEED, 200, 10);
        encoderY(DRIVE_SPEED,-15,-15,20);
        claw.setPosition(0.97);
        DcMotorLiftEncoder(25, 1400);
        turnToHeading(DRIVE_SPEED, 0.0);
        encoderStraffe(DRIVE_SPEED, -400, 10);
        DcMotorLiftEncoder(25, 2175);
        encoderY(0.3,-20,-20,20);
        encoderY(0.3,-4,-4,20);
        sleep(500);
        DcMotorLiftEncoder(25, 1400);
        sleep(500);
        DcMotorLiftEncoder(25, 1000);
        encoderY(DRIVE_SPEED,-3,-3,20);
        claw.setPosition(1);
        StopRobot();*/
        
        
        /*encoderY(DRIVE_SPEED,-15,-15,20);
        encoderStraffe(DRIVE_SPEED, -100, 10);
        DcMotorSlideEncoder(25, 1000);*/
        
        
        /*encoderStraffe(DRIVE_SPEED, 100, 10);
        StopRobot();
        encoderY(DRIVE_SPEED,-29,-29,20);
        StopRobot();
        lift.setPosition(25);
        encoderY(DRIVE_SPEED,-5,-5,20);
        //lower slide
        //open claw
        encoderY(DRIVE_SPEED,15,15,20);
        turnToHeading(DRIVE_SPEED,-90.0);
        StopRobot();
        encoderY(DRIVE_SPEED,25,25,20);
        StopRobot();
        //intake in
        StopRobot();
        encoderStraffe(DRIVE_SPEED, 50, 10);
        StopRobot();
        //intake out
        encoderStraffe(DRIVE_SPEED, -50, 10);
        StopRobot();
        encoderY(DRIVE_SPEED,10,10,20);
        //intake in
        encoderStraffe(DRIVE_SPEED, -50, 10);
        StopRobot();
        //intake out
        turnToHeading(DRIVE_SPEED,-90.0);
        //pick up specimen
        encoderY(DRIVE_SPEED,10,10,20);
        turnToHeading(DRIVE_SPEED,180.0);
        encoderStraffe(DRIVE_SPEED, 200, 10);
        //lift slide
        encoderY(DRIVE_SPEED,-5,-5,20);
        //lower slide
        //open claw
        encoderY(DRIVE_SPEED,15,15,20);
        turnToHeading(DRIVE_SPEED,-90.0);
        StopRobot();
        encoderY(DRIVE_SPEED,35,35,20);
        StopRobot();
        //intake in
        encoderStraffe(DRIVE_SPEED, -50, 10);
        StopRobot();
        //intake out
        turnToHeading(DRIVE_SPEED,-90.0);
        //pick up specimen
        encoderY(DRIVE_SPEED,10,10,20);
        turnToHeading(DRIVE_SPEED,180.0);
        encoderStraffe(DRIVE_SPEED, 200, 10);
        //lift slide
        encoderY(DRIVE_SPEED,-5,-5,20);
        //lower slide
        //open claw
        encoderY(DRIVE_SPEED,15,15,20);
        StopRobot();
        encoderStraffe(DRIVE_SPEED,-200, 10);
        StopRobot();
        encoderY(DRIVE_SPEED,15,15,20);
        //pick up specimen
        encoderY(DRIVE_SPEED,10,10,20);
        turnToHeading(DRIVE_SPEED,180.0);
        encoderStraffe(DRIVE_SPEED,200, 10);
        //lift slide
        encoderY(DRIVE_SPEED,-5,-5,20);
        //lower slide
        //open claw
        encoderY(DRIVE_SPEED,15,15,20);
        StopRobot();
        encoderStraffe(DRIVE_SPEED,50, 10);
        StopRobot();
        encoderY(DRIVE_SPEED,-15,-15,20);
        StopRobot();*/
        

        
        
        //telemetry.addData("Image", "%s", sFoundItem);
        telemetry.update();
            
        sleep(20000);  // pause to display final telemetry message.
        

    }// ***************** End Autonomous 

    public void LeftStraffe(double power, long time){
        frontright.setPower(power);
        frontleft.setPower(-power);
        backright.setPower(-power);
        backleft.setPower(power);
        sleep(time);
    }
 
    public void TestMotor(double power, long time){
        frontright.setPower(power);
        sleep(time);
        frontright.setPower(0);
        frontleft.setPower(power);
        sleep(time);
        frontleft.setPower(0);
        backright.setPower(power);
        sleep(time);
        backright.setPower(0);
        backleft.setPower(power);
        sleep(time);        
        backleft.setPower(0);

        
    }
  
    public void RightStraffe(double power, long time){
        frontright.setPower(-power);
        frontleft.setPower(power);
        backright.setPower(power);
        backleft.setPower(-power);
        sleep(time);
    }
    
    public void DriveY(double power, double driveDistance, double timeoutS){
        runtime.reset();  // This resets timer so you can use the runtime.seconds()
        while (opModeIsActive() && (distance.getDistance(DistanceUnit.INCH) > driveDistance) && (runtime.seconds() < timeoutS)){ // the && means all conditions need to be true for the while loop to run
            frontright.setPower(power);
            frontleft.setPower(power);
            backright.setPower(power);
            backleft.setPower(power);
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));//added show end value it may help see the last value
        telemetry.update();
    }
    
    public void DriveX(double power, double driveDistance, double timeoutS){
        runtime.reset();  // This resets timer so you can use the runtime.seconds()
        while (opModeIsActive() && (distanceside.getDistance(DistanceUnit.INCH) > driveDistance) && (runtime.seconds() < timeoutS)){ // the && means all conditions need to be true for the while loop to run
            frontright.setPower(-power);
            frontleft.setPower(power);
            backright.setPower(power);
            backleft.setPower(-power);
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        telemetry.addData("range2", String.format("%.01f in", distanceside.getDistance(DistanceUnit.INCH)));//added show end value it may help see the last value
        telemetry.update();
    }

    public void Turn(double Lpower, double Rpower, long time){
        frontright.setPower(Rpower);
        frontleft.setPower(Lpower);
        backright.setPower(Rpower);
        backleft.setPower(Lpower);
        sleep(time);
    }
    

    public void StopRobot()
    {
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        sleep(20);
    }
    
    public void DcMotorLiftEncoder(double speed, int target) {
        lift.setTargetPosition(target);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(Math.abs(speed));
    }
    
    public void DcMotorSlideEncoder(double speed, int target) {
        slide.setTargetPosition(target);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(speed));
    }

    public void encoderY(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontleftTarget = frontleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontrightTarget = frontright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackleftTarget = backleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackrightTarget = backright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            // Turn On RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
        /*    if (0> leftInches) // if destinstion inches is negative set speed to-
            frontleft.setPower(speed);
            else
            frontright.setPower(speed);
            
            if (0 > rightInches)
            frontright.setPower(-speed);
            else
            frontright.setPower(speed);*/
            
            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));
            

            
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()  )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newfrontleftTarget,  newfrontrightTarget, newbackleftTarget,  newbackrightTarget);
                telemetry.addData("Currently at",  " %7d :%7d :%7d :%7d", frontleft.getCurrentPosition(), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(20);   // optional pause after each move.
        }
                             
                             }                     
        public void encoderStraffe(double speed, int iTicks, double timeoutS) {

        // Ensure that the opmode is still active Right
        if (opModeIsActive()) {
        
        int newfrontleftTarget;
        int newfrontrightTarget;
        int newbackleftTarget;
        int newbackrightTarget;
            
            newfrontleftTarget = frontleft.getCurrentPosition() + iTicks;
            newfrontrightTarget = frontright.getCurrentPosition() - iTicks;
            newbackleftTarget = backleft.getCurrentPosition() - iTicks;
            newbackrightTarget = backright.getCurrentPosition() + iTicks;

            frontleft.setTargetPosition(newfrontleftTarget);
            frontright.setTargetPosition(newfrontrightTarget);
            backleft.setTargetPosition(newbackleftTarget);
            backright.setTargetPosition(newbackrightTarget);

            // Turn On RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            frontleft.setPower(Math.abs(speed));
            frontright.setPower(Math.abs(speed));
            backleft.setPower(Math.abs(speed));
            backright.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()  )) {


                // Display it for the driver.
                telemetry.addData("Target", " :%7d", iTicks);
                telemetry.addData("Currently at",  " %7d :%7d :%7d :%7d", frontleft.getCurrentPosition(), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            StopRobot();

            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(20);   // optional pause after each move.
        }
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = backleft.getCurrentPosition() + moveCounts;
            leftTarget = frontleft.getCurrentPosition() + moveCounts;
            rightTarget = backright.getCurrentPosition() + moveCounts;
            rightTarget = frontright.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontleft.setTargetPosition(leftTarget);
            backleft.setTargetPosition(leftTarget);
            frontright.setTargetPosition(rightTarget);
            backright.setTargetPosition(rightTarget);

            // Turn On RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (frontleft.isBusy() && frontright.isBusy()))
                   {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);            
            
            // Turn off RUN_TO_POSITION
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    
    

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        //moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontleft.setPower(leftSpeed);
        frontright.setPower(rightSpeed);
        backleft.setPower(leftSpeed);
        backright.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontleft.getCurrentPosition(),
                   frontright.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

       /*public void ArmGotoGroundPosition(double speed, int position, int height, double timeout) {
        runtime.reset();
        arm.setTargetPosition(position);
        arm.setPower(speed);
        while ((distance.getDistance(DistanceUnit.CM) > height)  && arm.isBusy() && opModeIsActive() &&  (runtime.seconds() < timeout) ){
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("Sensor currently at",  " %7d ", (double) distance.getDistance(DistanceUnit.CM));
        //telemetry.addData("Arm currently at",  " %7d ", (double) arm.getCurrentPosition());
        //telemetry.update();
    } */
    
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    
    // Step through the list of detected tags and look for a matching tag
        public double driveToAprilTag() {
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            desiredTag  = null;
             double  rangeError      = (desiredTag.ftcPose.range);
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                    targetFound = false;
                }
                if(targetFound == true){
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;
                double  drive           = 0;        // Desired forward power/speed (-1 to +1)
                double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
                double  turn            = 0;        // Desired turning power/speed (-1 to +1)


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                
                moveRobot(drive, strafe, turn);
                sleep(10);
                }
                
            }
            return rangeError;
        }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    
    /*private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessor(aprilTag)
        .build();
        
    }*/
    
    /*private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }*/

        // Set camera controls unless we are stopping.
        /*if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }*/

    
}
                                                
