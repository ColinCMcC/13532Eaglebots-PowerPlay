package org.firstinspires.ftc.teamcode.Olaf;

import static com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/*
    Motor Config
    0 - Lmotor white
    1 - RMotor black
    2 - FMotor red
    3 - BMotor blue
    4 - LiftMotor

    Servo Config
    0 - Claw

    Digital Config
    0 - Home

    I2C Config
    0 - 0- IMU
    0 - 1 - leftDist
    1 - 0 - backDist
    2 - 0 - rightDist
    3 - 0 - V3color
*/
public class EaglebotConfig_v5 {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    //Creates variables for all the devices
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

    //IMU setups
    BNO055IMU imu;

    ElapsedTime runtime = new ElapsedTime();

    //Define a constructor that allows the OpMode to pass a reference to itself.
    public EaglebotConfig_v5(LinearOpMode opmode) {myOpMode = opmode;}//end EagleConfig


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
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Lift motor config
        liftMotor = myOpMode.hardwareMap.get(DcMotor.class, "LiftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// add a little power to hold claw up on lift
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        home = myOpMode.hardwareMap.get(TouchSensor.class, "Home");// Home switch at the bottom of the lift

        //Servo claw config
        claw = myOpMode.hardwareMap.get(Servo.class, "Claw");

        //sets the parameters of the IMU in the control hub
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = IMU;
        parameters.angleUnit            = DEGREES;
        parameters.accelUnit            = METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }//end init function


    //sends info to datapad for debug and testing
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

        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("IMU:", getHeading());
        myOpMode.telemetry.update();
    }//end checkData function


    public void move(double drive, double strafe, double turn, boolean boost){
        double max;
        double leftAdj = 1;
        double rightAdj = 0.95;

        // when boost is true speeds to full speed otherwise reduced
        if (boost) {
            max = 0.75;
        } else {
            max = 0.35;
        }

        //send calculated power to wheels
        FLMotor.setPower(((drive - strafe - turn) * max) * leftAdj);
        BLMotor.setPower(((drive + strafe - turn) * max) * leftAdj);
        BRMotor.setPower(((drive - strafe + turn) * max) * rightAdj);
        FRMotor.setPower(((drive + strafe + turn) * max) * rightAdj);
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

        if (home.isPressed() && myOpMode.gamepad2.left_stick_y > 0) {
            max =0;
        }

        liftMotor.setPower((liftCtrl * max) + 0.002);

        claw.setPosition(clawCtrl / 2);
    }// end lift


    public void liftHome() {
        runtime.reset();
        //goes down until lift hits home
        while (!home.isPressed() && runtime.seconds() < 2) {
            liftMotor.setPower(-0.1);
        }

        liftMotor.setPower(0);

        //sets liftMotor encoder to 0 and allows it to go to encoder position
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }// End liftHome


    public void stopLift() {liftMotor.setPower(0);}// stops lift motor if necessary


    public int colorSense() {
        int result = 1;

        //color sensor sees red
        if (ColorHue.red() * 1.25 > ColorHue.green() && ColorHue.red() * 1.25 > ColorHue.blue()) {
            stopDrive();
        }

        //color sensor sees green
        else if (ColorHue.green() > ColorHue.blue()) {
            stopDrive();
            result = 2;
        }

        //color sensor sees blue
        else {result = 3;}

        return (result);
    }// end colorSense


    // Side 1 is left, Side 2 is right
    public void colorMove(int side){
        double getColor = colorSense();// Runs color sense to get the color of the cone

        if (side == 1){
            // If color sensor sees red
            if (getColor == 1){
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 28){// Runs until robot is far enough away from back wall
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) > 2){// Moves to the left zone
                    move(0, -1, 0, false);
                    myOpMode.sleep(500);
                }stopDrive();
            }// End if color sensor sees red

            // If color sensor sees green
            if(getColor == 2){
                //move more fully into the zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 28) {
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                stopDrive();
            }// End if color sensor sees green

            // If color sensor sees blue
            if(getColor == 3){
                //move to correct zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) < 45){
                    move(0, 1, 0, false);
                    myOpMode.sleep(500);
                }
                stopDrive();
            }// End if color sensor sees blue
        }// End if side == 1

        else if (side == 2){
            // If color sensor sees red
            if(getColor == 1){
                //move to red zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) < 50){
                    move(0, -1, 0, false);
                    myOpMode.sleep(500);
                }

                stopDrive();
            }// End if color sensor sees red

            // If color sensor sees green
            if(getColor == 2){
                //move more fully into the green zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30) {
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                stopDrive();
            }// End if color sensor sees green

            // If color sensor sees blue
            if(getColor == 3){
                //move to blue zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30){
                    move(-1, 0, 0, false);
                    myOpMode.sleep(500);
                }
                while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) > 6){
                    move(0, 1, 0, false);
                    myOpMode.sleep(500);
                }
                stopDrive();
            }// End if color sensor sees blue
        }// End else if side == 2
    }// End colorMove


    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        return heading;
    }//end getHeading

}//end class Eagle config