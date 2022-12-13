package org.firstinspires.ftc.teamcode.V4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
    Motor Config
    0 - Lmotor white
    1 - RMotor black
    2 - FMotor red
    3 - BMotor blue


    Expansion Hub Config
    0 - LiftMotor
    1 - lEncoder
    2 - bEncoder
    3 - rEncoder

    Servo Config
    0 - Claw

    Digital Config
    1 - Home
*/
public class EaglebotConfig_v4 {
    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.


    DcMotor FLMotor = null;
    DcMotor BRMotor = null;
    DcMotor BLMotor = null;
    DcMotor FRMotor = null;

    DcMotor liftMotor = null;
    Servo claw = null;
    TouchSensor home = null;

    ColorSensor ColorHue;
    DistanceSensor Distance;
    DistanceSensor leftDist;
    DistanceSensor backDist;
    DistanceSensor rightDist;

    ElapsedTime runtime = new ElapsedTime();

    //Define a constructor that allows the OpMode to pass a reference to itself.
    public EaglebotConfig_v4(LinearOpMode opmode) {
        myOpMode = opmode;
    }//end EagleConfig

    public void init() {
        ColorHue = myOpMode.hardwareMap.get(ColorSensor.class, "V3color");
        Distance = myOpMode.hardwareMap.get(DistanceSensor.class, "V3color");

        leftDist = myOpMode.hardwareMap.get(DistanceSensor.class, "leftDist");
        backDist = myOpMode.hardwareMap.get(DistanceSensor.class, "backDist");
        rightDist = myOpMode.hardwareMap.get(DistanceSensor.class, "rightDist");

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLMotor = myOpMode.hardwareMap.get(DcMotor.class, "LMotor");
        BLMotor = myOpMode.hardwareMap.get(DcMotor.class, "BMotor");
        BRMotor = myOpMode.hardwareMap.get(DcMotor.class, "RMotor");
        FRMotor = myOpMode.hardwareMap.get(DcMotor.class, "FMotor");

        // Resets encoder
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor direction to move forward
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Sets the mode of the motors
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Lift motor config
        liftMotor = myOpMode.hardwareMap.get(DcMotor.class, "LiftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// add a little power to hold claw up on lift
        home = myOpMode.hardwareMap.get(TouchSensor.class, "Home");// Home switch at the bottom of the lift

        //Servo claw config
        claw = myOpMode.hardwareMap.get(Servo.class, "Claw");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }//end init function


    //sends motor positions to datapad
    public void checkData() {
        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("R>", FLMotor.getCurrentPosition());
        myOpMode.telemetry.addData("B>", BLMotor.getCurrentPosition());
        myOpMode.telemetry.addData("L>", BRMotor.getCurrentPosition());
        myOpMode.telemetry.addData("F>", FRMotor.getCurrentPosition());
        myOpMode.telemetry.addData("Lift", liftMotor.getCurrentPosition());

        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("Red", ColorHue.red());
        myOpMode.telemetry.addData("Green", ColorHue.green());
        myOpMode.telemetry.addData("Blue", ColorHue.blue());

        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("Color Distance", Distance.getDistance(DistanceUnit.CM));

        myOpMode.telemetry.addData("leftDist", leftDist.getDistance(DistanceUnit.INCH));
        myOpMode.telemetry.addData("backDist", backDist.getDistance(DistanceUnit.INCH));
        myOpMode.telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.INCH));
        myOpMode.telemetry.update();
    }//end checkEncoder function


    public void move(double drive, double strafe, double turn, boolean boost){
        double max;
        double leftAdj = 0;
        double rightAdj = .05;

        // when boost is true speeds to full speed otherwise reduced
        if (boost) {
            max = 0.75;
        } else {
            max = 0.35;
        }

        //send calculated power to wheels
        FLMotor.setPower(((rightAdj + drive - strafe) - turn) * max);
        BLMotor.setPower(((rightAdj + drive + strafe) - turn) * max);
        BRMotor.setPower(((leftAdj + drive - strafe) + turn) * max);
        FRMotor.setPower(((leftAdj + drive + strafe) + turn) * max);
    }// end move


    //stops all motors if necessary
    public void stopDrive() {
        FLMotor.setPower(0.0);
        BLMotor.setPower(0.0);
        BRMotor.setPower(0.0);
        FRMotor.setPower(0.0);
    }// end stopDrive


    public void lift(double liftCtrl, double clawCtrl, boolean slow) {
        double max;// Defines speed

        //slows lift if A is pressed on gamepad2
        if (slow) {
            max = 0.5;
        } else {
            max = 1;
        }

        liftMotor.setPower((liftCtrl * max) + 0.002);

        claw.setPosition(clawCtrl / 2);
    }// end lift


    public void liftHome() {
        //goes down until lift hits home
        while (!home.isPressed()) {
            liftMotor.setPower(-0.2);
        }

        liftMotor.setPower(0);

        //sets liftMotor encoder to 0 and allows it to go to encoder position
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }// End liftHome



    public void stopLift() {liftMotor.setPower(0);}// stops lift motor if necessary


    public int colorSense() {
        runtime.reset();

        move(-0.8,0.0,-0.05,false);//a little turn is added to correct for drift
        while((myOpMode.opModeIsActive() && Distance.getDistance(DistanceUnit.CM) > 2) && (runtime.seconds() < 4.0)){
            myOpMode.idle();
        }// end move to cone color sense

        //color sensor sees red
        if (ColorHue.red() * 1.25 > ColorHue.green() && ColorHue.red() > ColorHue.blue()) {
            stopDrive();
            return (1);
        }

        //color sensor sees green
        if ((ColorHue.green() > ColorHue.red() && ColorHue.green() > ColorHue.blue())) {
            stopDrive();
            return (2);
        }

        //color sensor sees blue
        if (ColorHue.blue() > ColorHue.red() && ColorHue.blue() > ColorHue.green()) {
            stopDrive();
            return (3);
        }
        return (0);
    }// end colorSense


    public void colorMoveDistLeft(){
        double getColor = colorSense();

        while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 25){
            move(-1, 0, 0, false);
        }
        stopDrive();

        //if color sensor sees red
        if(getColor == 1){
            //move to correct zone
            while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) < 2){
                move(0, -1, 0, false);
            }
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees red

        //if color sensor sees green
        if(getColor == 2){
            //move more fully into the zone
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30) {
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees green

        //if color sensor sees blue
        if(getColor == 3){
            //move to correct zone
            while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) < 45){
                move(0, 1, 0, false);
            }
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees blue
    }//end colorMoveDistLeft

    public void colorMoveDistRight(){
        double getColor = colorSense();

        while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 25){
            move(-1, 0, 0, false);
        }
        stopDrive();

        //if color sensor sees red
        if(getColor == 1){
            //move to correct zone
            while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) < 50){
                move(0, -1, 0, false);
            }
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees red

        //if color sensor sees green
        if(getColor == 2){
            //move more fully into the zone
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30) {
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees green

        //if color sensor sees blue
        if(getColor == 3){
            //move to correct zone
            while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) > 6){
                move(0, 1, 0, false);
            }
            while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                move(-1, 0, 0, false);
            }
            stopDrive();
        }//end if color sensor sees blue
    }//end colorMoveDistRight

}//end class Eagle config