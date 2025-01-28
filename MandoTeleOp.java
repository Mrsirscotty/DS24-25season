package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.dfrobot.HuskyLens;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="MandoTeleOp", group="Linear Opmode")
public class MandoTeleOp extends LinearOpMode {
    static final double BACK_POS     =  0.52;
    static final double MID_POS     =  0.3;     
    static final double FRONT_POS     =  0.42;
    double  position = FRONT_POS;
   //DistanceSensor distance;
    
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    
    private double BClawPosition = 0.57;
    
    //RevBlinkinLedDriver blinkinLedDriver;
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    
    @Override
    public void runOpMode() throws InterruptedException {
       /* blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
        */
        DcMotor motorfrontleft = hardwareMap.dcMotor.get("frontleft");
        DcMotor motorbackleft = hardwareMap.dcMotor.get("backleft");
        DcMotor motorfrontright = hardwareMap.dcMotor.get("frontright");
        DcMotor motorbackright = hardwareMap.dcMotor.get("backright");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor hinge = hardwareMap.dcMotor.get("hinge");
        //CRServo intake = hardwareMap.crservo.get("intake");
        Servo claw = hardwareMap.servo.get("claw");
        //DcMotor hang = hardwareMap.dcMotor.get("hang");
        //Servo tap = hardwareMap.servo.get("tap");
        //distance = hardwareMap.get(DistanceSensor.class, "distance");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Servo BClaw = hardwareMap.servo.get("BClaw");
        Servo BSpin = hardwareMap.servo.get("BSpin");
        
        
        
    
    


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorfrontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorbackleft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorbackleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfrontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbackright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfrontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hinge.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        

        //pointer.setPosition(0.11);
       
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        double slowdown = 1;
        
        BSpin.setPosition(0.57);
        
        //BClaw.setPosition(0.1);


        waitForStart();

        if (isStopRequested()) return;
        double rx = 0.0;
        double dFlipPosition = 0.0;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            //rx = gamepad1.right_trigger;

            if (gamepad1.right_trigger > 0)
                rx = gamepad1.right_trigger;
            else
                rx = -gamepad1.left_trigger;

            if (gamepad1.back)
                imu.initialize(parameters);

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorfrontleft.setPower(frontLeftPower/slowdown); //was 1.35
            motorbackleft.setPower(backLeftPower/slowdown);
            motorfrontright.setPower(frontRightPower/slowdown);
            motorbackright.setPower(backRightPower/slowdown);

           //slide
           if (gamepad2.dpad_right){
               slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               slide.setPower(0.5);
            /*if (slide.getCurrentPosition() > 2100){
                    slide.setPower(0);
               }*/
            }
           else if (gamepad2.dpad_left){
               slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               slide.setPower(-0.5);
           }
           else{
                slide.setPower(0);
           }

            /*if (slide.getCurrentPosition() > 2600){
                slide.setPower(0.0);
           }*/
           
           //intake
           /*if (gamepad2.dpad_left){
               intake.setPower(1);
           }
           else if (gamepad2.dpad_right){
               intake.setPower(-1);
           }
           else{
               intake.setPower(0);
           }*/
           
           //lift 2.0
           /*if (gamepad2.dpad_up){
               lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               lift.setPower(-0.5);
               if (lift.getCurrentPosition() < 2400){
                    lift.setPower(-0.5);
               }
            }
           else if (gamepad2.dpad_down){
               lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               lift.setPower(0.5);
           }
           else{
                lift.setPower(0);
           }*/
           
           //lift
           if (gamepad2.dpad_up){
               lift.setPower(-.5);
           }
           else if (gamepad2.dpad_down){
               lift.setPower(.5);
           }
           else{
               lift.setPower(0);
           }
         
           //claw
           if (gamepad1.x){
               claw.setPosition(0.5);
           }
           else if (gamepad1.b){
               claw.setPosition(0.545);
           }
           
           /*//hang
           if (gamepad2.y){
               hang.setPower(1);
           }
           else if (gamepad2.a){
               hang.setPower(-1);
           }
           else{
               hang.setPower(0);
           }*/
           
          
           
           /*if (gamepad2.y){
               BClaw.setPosition(0.08);
           }
           else if(gamepad2.a){
               BClaw.setPosition(0);
           }
           
           if(gamepad2.left_stick_button){
               BClawPosition += 0.002;
               BSpin.setPosition(BClawPosition);
           }
           
           if(gamepad2.right_stick_button){
               BClawPosition -= 0.002;
               BSpin.setPosition(BClawPosition);
           }*/
           
           
            
            
            
           /*if (gamepad2.y){
           
                if ((blocks[i].width > 100) && (blocks[i].height < 101) && (blocks[i].id == 1)){
                    //rotate the servo positivily
                    BClawPosition += 0.002;
                    //claw.setPosition(0.013);
                    
                }
                else if ((blocks[i].width < 100) && (blocks[i].height > 101) && (blocks[i].id == 1)){
                    //set servo power = 0
                    BClaw.setPosition(0.08);
                    //claw.setPosition(0.055);
                }
                BSpin.setPosition(BClawPosition);
            }
           }*/
           
           if (gamepad2.b) {
               /*hinge.setTargetPosition(664);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));*/
               
               BSpin.setPosition(0.5);
               
                    /*hinge.setTargetPosition(664);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    BSpin.setPosition(0.525);
                    sleep(500);
                    hinge.setTargetPosition(800);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    sleep(500);
                    BClaw.setPosition(0.07);
                    sleep(500);
                    hinge.setTargetPosition(700);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    hinge.setTargetPosition(600);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    hinge.setTargetPosition(450);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));*/
                    
                    
                /*sleep(1000);
                motorbackright.setPower(-0.3);
                    //motorfrontleft.setPower(-0.3);
                    //motorfrontright.setPower(-0.3);
                    motorbackleft.setPower(-0.4);
                    sleep(500);
                        //motorfrontleft.setPower(0);
                        //motorfrontright.setPower(0);
                        motorbackleft.setPower(0);
                        motorbackright.setPower(0);*/
                
               
           }
           
           
           if (gamepad2.start){
               //BClaw.setPosition(0.08);
               HuskyLens.Block[] blocks = huskyLens.blocks();
               telemetry.addData("Block count", blocks.length);
               for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                BClaw.setPosition(0.09);
                //horizontal sample
                if (((blocks[i].width > 90) && (blocks[i].height < 101)) && (blocks[i].id == 1) && (BClawPosition != 0.252)){
                    //move hinge down
                    hinge.setTargetPosition(664);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    /*slide.setTargetPosition(slide.getCurrentPosition()+200);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(Math.abs(0.5));*/
                    
                    //move robot
                    if((blocks[i].x < 170) && (blocks[i].id == 1)){
                        motorfrontright.setPower(0.3);
                        motorbackright.setPower(0.3);
                        motorbackleft.setPower(0.3);
                        motorfrontleft.setPower(0.3);
                        sleep(50);
                        motorfrontright.setPower(0);
                        motorbackright.setPower(0);
                        motorbackleft.setPower(0);
                        motorfrontleft.setPower(0);
                        sleep(500);
                    }
                    
                    if((blocks[i].x > 178) && (blocks[i].id == 1)){
                        motorfrontright.setPower(-0.3);
                        motorbackright.setPower(-0.3);
                        motorbackleft.setPower(-0.3);
                        motorfrontleft.setPower(-0.3);
                        sleep(50);
                        motorfrontright.setPower(0);
                        motorbackright.setPower(0);
                        motorbackleft.setPower(0);
                        motorfrontleft.setPower(0);
                        sleep(500);
                    }
                    
                    //move slide out
                    if((blocks[i].y < 170) && (blocks[i].id == 1)){
                    slide.setPower(0.3);
                    }
                    else if(((blocks[i].y > 170) || (blocks.length == 0)) && (blocks[i].id == 1)){
                    slide.setPower(0);
                    slide.setPower(0.3);
                    sleep(300);
                    slide.setPower(0);
                    //move hinge down
                    //sleep(500);
                    hinge.setTargetPosition(830);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    sleep(500);
                    BClaw.setPosition(0.07);
                    sleep(500);
                    BSpin.setPosition(0.62);
                    sleep(500);
                    hinge.setTargetPosition(820);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(810);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(800);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(790);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(780);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(770);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(760);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(750);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(330);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(1000);
                    }
                }
                //vertical sample
                else if ((((blocks[i].width < 100) && (blocks[i].height > 101)) || ((blocks[i].width < 80) && (blocks[i].height < 100))) && (blocks[i].id == 1) && (BClawPosition != 0.525)){
                    hinge.setTargetPosition(664);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    
                    //middle
                    if ((blocks[i].x < 230) && (blocks[i].x > 135) && (blocks[i].id == 1)){
                        motorfrontright.setPower(0.3);
                    motorbackright.setPower(0.3);
                    motorbackleft.setPower(0.3);
                    motorfrontleft.setPower(0.3);
                    sleep(75);
                    motorfrontright.setPower(0);
                    motorbackright.setPower(0);
                    motorbackleft.setPower(0);
                    motorfrontleft.setPower(0);
                    sleep(500);
                    }
                    
                    //location of sample for BSpin
                    //left
                    if ((blocks[i].x >= 230) && (blocks[i].id == 1)){
                    if((blocks[i].y < 190) && (blocks[i].id == 1)){
                    slide.setPower(0.3);
                    }
                    else if((blocks[i].y > 190) && (blocks[i].id == 1)){
                    slide.setPower(0);
                    BSpin.setPosition(0.505);
                    slide.setPower(0.3);
                    sleep(360);
                    slide.setPower(0);
                    
                    if ((blocks[i].height > 13) && (blocks[i].id == 1)){
                    motorfrontright.setPower(-0.3);
                    motorbackright.setPower(-0.3);
                    motorbackleft.setPower(-0.3);
                    motorfrontleft.setPower(-0.3);
                    sleep(100);
                    motorfrontright.setPower(0);
                    motorbackright.setPower(0);
                    motorbackleft.setPower(0);
                    motorfrontleft.setPower(0);
                    }
                    /*if ((blocks[i].height > 13) && (blocks[i].id == 1)){
                    //motorfrontright.setPower(-0.3);
                    motorbackright.setPower(-0.3);
                    motorbackleft.setPower(-0.3);
                    //motorfrontleft.setPower(-0.3);
                        if((blocks[i].height < 13) && (blocks[i].id == 1))
                        //motorfrontright.setPower(0);
                        motorbackleft.setPower(0);
                        motorbackright.setPower(0);
                        //motorfrontleft.setPower(0);
                    }*/
                    sleep(1500);
                    hinge.setTargetPosition(830);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    sleep(500);
                    BSpin.setPosition(0.53);
                    sleep(500);
                    /*hinge.setTargetPosition(800);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    sleep(500);*/
                    BClaw.setPosition(0.07);
                    sleep(500);
                    hinge.setTargetPosition(820);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(810);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(800);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(790);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(780);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(770);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(760);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(750);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(330);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(1000);
                    }
                    }
                    //right
                    else if ((blocks[i].x <= 135) && (blocks[i].id == 1)){
                    if((blocks[i].y < 190) && (blocks[i].id == 1)){
                    slide.setPower(0.3);
                    }
                    else if((blocks[i].y > 190) && (blocks[i].id == 1)){
                    slide.setPower(0);
                    BSpin.setPosition(0.62);
                    slide.setPower(0.3);
                    sleep(260);
                    slide.setPower(0);
                    
                    /*do{
                        motorfrontleft.setPower(0.3);
                        motorfrontright.setPower(0.3);
                        motorbackleft.setPower(0.3);
                        motorbackright.setPower(0.3);
                        sleep(10);
                        blocks = huskyLens.blocks();
                    } while(blocks.length>0);*/
                    
                    if ((blocks[i].height > 13) && (blocks[i].id == 1)){
                    motorfrontright.setPower(0.3);
                    motorbackright.setPower(0.3);
                    motorbackleft.setPower(0.3);
                    motorfrontleft.setPower(0.3);
                    sleep(120);
                    motorfrontright.setPower(0);
                    motorbackright.setPower(0);
                    motorbackleft.setPower(0);
                    motorfrontleft.setPower(0);
                    }
                    sleep(1500);
                    hinge.setTargetPosition(830);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    sleep(500);
                    BSpin.setPosition(0.58);
                    sleep(500);
                    BClaw.setPosition(0.07);
                    sleep(500);
                    hinge.setTargetPosition(820);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(810);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(800);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(790);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(780);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(770);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(760);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(750);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(200);
                    hinge.setTargetPosition(330);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(-0.5));
                    sleep(1000);
                    }
                    }
                }
                
                //diagonal sample
                else if (((blocks[i].width > 100) && (blocks[i].height > 100)) && (blocks[i].id == 1)){
                    
                    //move chasis so middle position of block is correct
                    //turn left angled
                    //if vertical, turn and go out
                    //if horizontal, go out
                    //pick up
                    
                    hinge.setTargetPosition(664);
                    hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hinge.setPower(Math.abs(0.5));
                    if((blocks[i].x < 180) && (blocks[i].id == 1)){
                        motorfrontright.setPower(0.3);
                        motorbackright.setPower(0.3);
                        motorbackleft.setPower(0.3);
                        motorfrontleft.setPower(0.3);
                        sleep(80);
                        motorfrontright.setPower(0);
                        motorbackright.setPower(0);
                        motorbackleft.setPower(0);
                        motorfrontleft.setPower(0);
                        //sleep(500);
                    }
                    if((blocks[i].x >= 180) && (blocks[i].id == 1)){
                    BSpin.setPosition(0.525);
                    BClawPosition = 0.525;
                    }
                    
                    
                    
                    //if (((blocks[i].width > 90) && (blocks[i].height < 80)) && (blocks[i].id == 1)){
                        
                    
                }
           }
           }   
           
           
           
           
           
           
           

           
           /*if (gamepad2.start){
            BClaw.setPosition(0);
            
            //hinge.setTargetPosition(806);
            //hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //hinge.setPower(Math.abs(0.5));
            
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (((blocks[i].width < 100) && (blocks[i].height > 101)) && (blocks[i].id == 1)){
                    //rotate the servo positivily
                    BClaw.setPosition(0.115);
                    //|| ((blocks[i].width > 100) && (blocks[i].height > 101))
                    //claw.setPosition(0.013);
                    
                }
                else if (((blocks[i].width > 100) && (blocks[i].height < 101)) || ((blocks[i].width > 100) && (blocks[i].height > 101)) && (blocks[i].id == 1)){
                    //set servo power = 0
                    BSpin.setPosition(0.5);
                    sleep(500);
                    //int slideposition = (slide.getCurrentPosition()+=50);
                    slide.setTargetPosition(slide.getCurrentPosition()+300);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(Math.abs(0.5));
                    sleep(500);
                    BClaw.setPosition(0.115);
                    sleep(1000);
                    BSpin.setPosition(0.55);
                    
                    //claw.setPosition(0.055);
                }
            }
           }*/
           
           
           //Blue Claw open
           if (gamepad2.back){
               BClaw.setPosition(0.08);
           }
           else if (gamepad2.y){
               BClaw.setPosition(0.09);
               //x
               //0.12
           }
           else if (gamepad2.a){
               BClaw.setPosition(0.07);
               //b
           }
             
           //new
           if (gamepad2.left_bumper && gamepad2.right_bumper){
               hinge.setPower(0);
           }
           
           else if (gamepad2.left_bumper){
               //bHingeMode=false;
               hinge.setTargetPosition(hinge.getCurrentPosition()-60);
                hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hinge.setPower(Math.abs(-0.5));
           }
           /*else if (hinge.getCurrentPosition()<500 && bHingeMode==false){
               hinge.setPower(0);
               }*/
           
           
           else if (gamepad2.right_bumper){
                //bHingeMode=true;
                hinge.setTargetPosition(hinge.getCurrentPosition()+50);
                hinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hinge.setPower(Math.abs(0.5));
           }
           
           //hinge Annika
           /*if (gamepad2.left_bumper){
               hinge.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               hinge.setPower(-0.5);
           }
           else if (gamepad2.right_bumper){
               hinge.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               hinge.setPower(0.5);
           }
           else{
               hinge.setPower(0);
           }*/
           
           
           
           //Blue Claw Spin
           if (gamepad2.right_stick_button){
               BClawPosition -= 0.002;
               BSpin.setPosition(BClawPosition);
           }
           else if (gamepad2.left_stick_button){
               BClawPosition += 0.002;
               BSpin.setPosition(BClawPosition);
           }
           
           if (gamepad2.x) {
               BSpin.setPosition(0.57);
               BClawPosition = 0.57;
           }
           
        

            telemetry.addData(" ", "May The Force Be With You" );
            telemetry.addData(" ", " " );


            telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d", 
            motorfrontleft.getCurrentPosition(), 
            motorbackleft.getCurrentPosition(), 
            motorfrontright.getCurrentPosition(), 
            motorbackright.getCurrentPosition());
            

            telemetry.addData(" ", " " );
         
            telemetry.addData("slowdown", "%7f", slowdown);
            
            telemetry.addData("Slide at", "%7d",
            slide.getCurrentPosition());
            
            telemetry.addData("Hinge at", "%7d",
            hinge.getCurrentPosition());
            
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            
            /*telemetry.addData("Blue Claw Spin at", "%7f",
            BClawSpin.getPosition());
            
            telemetry.addData("Blue Claw at", "%7f",
            BClaw.getPosition());*/
            
            //telemetry.addData("distance", "%7f", distance.getDistance(DistanceUnit.CM));
            
            //double flipPosition = flip.getPosition();
            //telemetry.addData("flip - get", "%7f", flip.getPosition());
            //telemetry.addData("flipPosition", "%7d", flipPosition);
        
         
            //telemetry.addData("Lift", "%7d", lift.getCurrentPosition());
            telemetry.update();
        }
    }
    

}
