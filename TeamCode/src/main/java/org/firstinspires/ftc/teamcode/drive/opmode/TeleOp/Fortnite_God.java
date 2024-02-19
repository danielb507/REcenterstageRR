package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Fortnite God")

public class Fortnite_God extends LinearOpMode {


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor Intake = null;




    private CRServo wheel_bucket;

    private Servo left_servo_lift;

    private Servo right_servo_lift;

    private Servo flipper_bucket;

    private DcMotor slide = null;
    private CRServo drone = null;

    private DcMotor left_lift = null;

    private DcMotor right_lift = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront"); //backright, port 2
        Intake = hardwareMap.get(DcMotor.class, "Intake");  //Intake
        slide = hardwareMap.get(DcMotor.class, "slide");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift =  hardwareMap.get(DcMotor.class, "right_lift");


        wheel_bucket = hardwareMap.get(CRServo.class, "wheel_bucket"); // Port 5 Expansion Hub
        flipper_bucket = hardwareMap.get(Servo.class, "flipper_bucket"); // port 4 Expansion Hub
        drone = hardwareMap.get(CRServo.class, "drone");

        left_servo_lift = hardwareMap.get(Servo.class, "left_servo_lift");
        right_servo_lift = hardwareMap.get(Servo.class, "right_servo_lift");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);



        drone.setDirection(DcMotorSimple.Direction.FORWARD);

        double left;
        double right;
        double drive;
        double turn;
        double max;


        waitForStart();
        //bucket.setPosition(0.85);

        while (opModeIsActive()) {


            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            double G1rightStickX = gamepad1.right_stick_x;
            double G1leftStickX = gamepad1.left_stick_x;

            boolean G1rightBumper = gamepad1.right_bumper;
            boolean G1leftBumper = gamepad1.left_bumper;
            boolean G1UD = gamepad1.dpad_up;   // up dpad
            boolean G1DD = gamepad1.dpad_down; //Down dpad
            boolean G1RD = gamepad1.dpad_right;// right dpad
            boolean G1LD = gamepad1.dpad_left; //left dpad
            boolean G1Y = gamepad1.y;
            boolean G1B = gamepad1.b;
            boolean G1X = gamepad1.x;
            boolean G1A = gamepad1.a;
            double G1RT = gamepad1.right_trigger;
            double G1LT = gamepad1.left_trigger;
            //Second controller (Intake/linear slide)
            double G2leftStickY = gamepad2.left_stick_y;
            boolean G2B = gamepad2.b;
            boolean G2Y = gamepad2.y;
            boolean G2A = gamepad2.a;
            boolean G2X = gamepad2.x;
            boolean G2UD = gamepad2.dpad_up; // up dpad
            boolean G2DD = gamepad2.dpad_down; // down dpad
            boolean G2RD = gamepad2.dpad_right;// right dpad
            boolean G2LD = gamepad2.dpad_left; //left dpad
            double G2LT = gamepad2.left_trigger;
            double G2RT = gamepad2.right_trigger;
            boolean G2rightBumper = gamepad2.right_bumper;
            boolean G2leftBumper = gamepad2.left_bumper;
            boolean G2back = gamepad2.back;


            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            leftFront.setPower(left / 1.3);
            rightFront.setPower(right / 1.3);
            leftRear.setPower(left / 1.3);
            rightRear.setPower(right / 1.3);


            //Driving movements
            double power = .8;

            if (G1rightStickX > 0) {  // Clockwise
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
            if (G1rightStickX < 0) { // Counterclockwise
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightFront.setPower(power);
                rightRear.setPower(power);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);


            }


            if (gamepad1.dpad_right) { //strafe right
                leftFront.setPower(1);
                rightFront.setPower(-1);
                leftRear.setPower(-1);
                rightRear.setPower(1);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }
            if (gamepad1.dpad_left) { //strafe left
                leftFront.setPower(-1);
                rightFront.setPower(1);
                leftRear.setPower(1);
                rightRear.setPower(-1);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }
            if (gamepad1.dpad_up) { // forwards
                leftFront.setPower(1);
                rightFront.setPower(1);
                leftRear.setPower(1);
                rightRear.setPower(1);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }
            if (gamepad1.dpad_down) { // backwards
                leftFront.setPower(-1);
                rightFront.setPower(-1);
                leftRear.setPower(-1);
                rightRear.setPower(-1);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
            }


            if (G1B) { // Intake Forward
                Intake.setPower(-0.7);
                //wheel_bucket.setPower(-1);
                //flipper_bucket.setPosition(1);

            } else if (G1X) {
                Intake.setPower(.7);

            } else {
                Intake.setPower(0);
                //wheel_bucket.setPower(0);
            }

            if (G1Y) {
                /*
                Intake.setPower(-1);
                wheel_bucket.setPower(-1);

                 */
            }

            /*
            if (G2UD) { // Upward
                slide.setPower(0.85);
            } else if (G2DD) {
                slide.setPower(-.85);
            } else {
                slide.setPower(.05);
            }

             */
            if (G1A) {
                right_servo_lift.setPosition(.3);
                sleep(400);
                drone.setPower(1);

            } else {

                drone.setPower(0);
            }
            /*
            if (G2X) {
                flipper_bucket.setPosition(.5);
            } else if (G2B) {
                flipper_bucket.setPosition(1);
            }

             */

            if (G1RT > 0) {
                left_lift.setPower(1);
                right_lift.setPower(1);
            } else if (G1LT > 0) {
                left_lift.setPower(-1);
                right_lift.setPower(-1);
            } else {
                left_lift.setPower(0);
                right_lift.setPower(0);
            }
            if (G1rightBumper) {
                left_servo_lift.setPosition(1);
                right_servo_lift.setPosition(0);
            }else if (G1Y){
                right_servo_lift.setPosition(0.4);
            }else if (G1leftBumper) {
                left_servo_lift.setPosition(0);
                right_servo_lift.setPosition(.6);
            }



            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

}

