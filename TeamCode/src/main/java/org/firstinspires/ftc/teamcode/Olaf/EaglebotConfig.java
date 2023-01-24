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

public class EaglebotConfig {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Creates variables for all the devices
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
    public EaglebotConfig(LinearOpMode opmode) {myOpMode = opmode;}


    public void init() {


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

        //Sensors config
        ColorHue = myOpMode.hardwareMap.get(ColorSensor.class, "V3color");
        Distance = myOpMode.hardwareMap.get(DistanceSensor.class, "V3color");
        leftDist = myOpMode.hardwareMap.get(DistanceSensor.class, "leftDist");
        backDist = myOpMode.hardwareMap.get(DistanceSensor.class, "backDist");
        rightDist = myOpMode.hardwareMap.get(DistanceSensor.class, "rightDist");

        // Sets the parameters of the IMU in the control hub
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = IMU;
        parameters.angleUnit            = DEGREES;
        parameters.accelUnit            = METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }// End init function


    // Sends info to datapad for debug and testing
    public void checkData() {
        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("R>", FLMotor.getCurrentPosition());
        myOpMode.telemetry.addData("B>", BLMotor.getCurrentPosition());
        myOpMode.telemetry.addData("L>", BRMotor.getCurrentPosition());
        myOpMode.telemetry.addData("F>", FRMotor.getCurrentPosition());

        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addData("Lift", liftMotor.getCurrentPosition());
        myOpMode.telemetry.addData("Claw", claw.getPosition());

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
    }// End checkData function


    public void move(double drive, double strafe, double turn, boolean boost){
        double max;
        double leftAdj = 1;
        double rightAdj = 0.95;

        // when boost is true speeds to full speed otherwise reduced
        if (boost) {
            max = 0.75;
        } else {
            max = 0.33;
        }

        //send calculated power to wheels
        FLMotor.setPower(((drive - strafe - turn) * max) * leftAdj);
        BLMotor.setPower(((drive + strafe - turn) * max) * leftAdj);
        BRMotor.setPower(((drive - strafe + turn) * max) * rightAdj);
        FRMotor.setPower(((drive + strafe + turn) * max) * rightAdj);
    }// End move


    public void fieldMove(double x, double y, double turn, boolean boost){
        double heading = Math.atan2(x, y);
        double power = Math.sqrt(Math.pow(x, x) + Math.pow(y, y));
        double qPi = Math.PI / 4;
        double max = 0.33;

        if (boost){
            max = 1;
        }else{
            max = 0.33;
        }

        double FL = -Math.sin(heading + qPi) * power - turn;
        double BL = -Math.cos(heading + qPi) * power - turn;
        double BR = Math.sin(heading + qPi) * power - turn;
        double FR = Math.cos(heading + qPi) * power - turn;

        if (Math.abs(FL) > 1)

        FLMotor.setPower(FL);
        BLMotor.setPower(BL);
        BRMotor.setPower(BR);
        FRMotor.setPower(FR);
    }


    // Stops all motors if necessary
    public void stopDrive() {
        FLMotor.setPower(0.0);
        BLMotor.setPower(0.0);
        BRMotor.setPower(0.0);
        FRMotor.setPower(0.0);
    }// End stopDrive


    public void lift(double liftCtrl, double clawCtrl, boolean slow) {
        double max;// Defines speed

        //slows lift if A is pressed on gamepad2
        if (slow) {
            max = 0.5;
        } else {
            max = 1;
        }

        if (home.isPressed() && myOpMode.gamepad2.left_stick_y > 0) {
            max = 0;
        }

        liftMotor.setPower(liftCtrl * max);

        claw.setPosition(clawCtrl / 2);
    }// End lift


    public void liftHome() {
        runtime.reset();

        // Goes down until lift hits home
        while (!home.isPressed() && runtime.seconds() < 2) {
            liftMotor.setPower(-0.1);
        }

        liftMotor.setPower(0);

        // Sets liftMotor encoder to 0 and allows it to go to encoder position
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }// End liftHome


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
    }// End colorSense


    public void colorMove(double side, double color){
        if (side == 1){
            if (color == 1){
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 26){// Runs until robot is far enough away from back wall
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);// Sets how often it runs this loop
                }
                while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) > 2){// Moves to the left zone
                    double turnPower = getHeading();// Keeps robot straight

                    move(0, -0.3, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
                stopDrive();
            }// End if color sensor sees red

            if(color == 2){// If color sensor sees green
                //move more fully into the zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30) {
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
                stopDrive();
            }// End if color sensor sees green

            if(color == 3){// If color sensor sees blue
                //move to correct zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 26){
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
                while (myOpMode.opModeIsActive() && leftDist.getDistance(DistanceUnit.INCH) < 26){
                    double turnPower = getHeading();// Keeps robot straight

                    move(0, 0.5, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
            }// End if color sensor sees blue
        }

        else if (side == 2){
            if(color == 1){// If color sensor sees red
                //move to red zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 26){
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
                while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) < 50){
                    double turnPower = getHeading();// Keeps robot straight

                    move(0, -0.5, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
            }// End if color sensor sees red

            if(color == 2){// If color sensor sees green
                //move more fully into the green zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 30) {
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
            }// End if color sensor sees green

            if(color == 3){// If color sensor sees blue
                //move to blue zone
                while (myOpMode.opModeIsActive() && backDist.getDistance(DistanceUnit.INCH) < 26){
                    double turnPower = getHeading();// Keeps robot straight

                    move(-0.5, 0, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
                while (myOpMode.opModeIsActive() && rightDist.getDistance(DistanceUnit.INCH) > 2){
                    double turnPower = getHeading();// Keeps robot straight

                    move(0, 0.5, turnPower / 20, false);
                    checkData();
                    myOpMode.sleep(100);
                }
            }// End if color sensor sees blue
        }// End if side == 2
        stopDrive();
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
    }// End getHeading

}// End class Eagle config