package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


@TeleOp(group = "drive")
@Config
public class teleop extends OpMode {

    //gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    //intake control
    boolean intakeForward = false;
    boolean intakeBack = false;

    //lift var
    public static int top = 1000;
    public static int bottom = 400;
    public static int var  = 50;
    //winch var
    public static int extend = 2000;
    public static int retract = 1900;

    public static int start = -100;

    boolean winchToggle = false;



    //lift power
    public static double lPower = 1;
    //winch power
    public static double wPower = 1;

    public static double raVar = 100;
    public static double laVar = 100;
    public static double rbVar = 100;
    public static double lbVar = 100;

//motor
    private DcMotorEx lift;
    private DcMotorEx winch;
    private DcMotorEx intake;

    private DcMotorEx RF;
    private DcMotorEx RB;
    private DcMotorEx LF;
    private DcMotorEx LB;

    // servo
    ServoImplEx rBucket;
    ServoImplEx lBucket;
    ServoImplEx rArm;
    ServoImplEx lArm;

    //axon servo encoder
    AnalogInput rbEncoder;
    AnalogInput lbEncoder;
    AnalogInput raEncoder;
    AnalogInput laEncoder;





    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //drive
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB= hardwareMap.get(DcMotorEx.class, "RB");
        LF= hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);


        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       //lift
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        //winch
        winch = hardwareMap.get(DcMotorEx.class, "winch");
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // servo
        rBucket = (ServoImplEx) hardwareMap.get(Servo.class, "rBucket");
        lBucket = (ServoImplEx) hardwareMap.get(Servo.class, "lBucket");
        rArm = (ServoImplEx) hardwareMap.get(Servo.class, "rArm");
        lArm = (ServoImplEx) hardwareMap.get(Servo.class, "lArm");

        //axon servo encoders

        rBucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        lBucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        rArm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        lArm.setPwmRange(new PwmControl.PwmRange(505, 2495));

        rbEncoder = hardwareMap.get(AnalogInput.class, "rbEncoder");
        lbEncoder = hardwareMap.get(AnalogInput.class, "lbEncoder");
        raEncoder = hardwareMap.get(AnalogInput.class, "raEncoder");
        laEncoder = hardwareMap.get(AnalogInput.class, "laEncoder");

    }

    @Override
    public void loop() {

        //store gamepad input
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        /*///////
        VARIABLE
        *////////


        double drive = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;
        //gamepad
        double d_power = .5;
        //strafe
        double s_power = 1 * gamepad1.right_trigger * gamepad1.left_trigger;

        //axon servo encoder
        double rbPos = rbEncoder.getVoltage() / 3.3 * 360;
        double lbPos = lbEncoder.getVoltage() / 3.3 * 360;
        double raPos = raEncoder.getVoltage() / 3.3 * 360;
        double laPos = laEncoder.getVoltage() / 3.3 * 360;

        /*////////////////
        DRIVER 1 CONTROLS
        *////////////////

        //drive

        RF.setPower(drive + rotate);
        RB.setPower(drive + rotate);
        LF.setPower(drive + rotate);
        LB.setPower(drive + rotate);


        //trigger strafe control
        if (gamepad1.right_trigger<.1) {
            RF.setPower(-s_power);
            RB.setPower(s_power);
            LF.setPower(s_power);
            LB.setPower(-s_power);
        }
        if (gamepad1.left_trigger<.1) {
            RF.setPower(-s_power);
            RB.setPower(s_power);
            LF.setPower(s_power);
            LB.setPower(-s_power);
        }
// d_pad drive control
        if (gamepad1.dpad_up) {
            RF.setPower(d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_down) {
            RF.setPower(-d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(-d_power);
        }
        if (gamepad1.dpad_left) {
            RF.setPower(d_power);
            RB.setPower(-d_power);
            LF.setPower(-d_power);
            LB.setPower(d_power);
        }
        if (gamepad1.dpad_right) {
            RF.setPower(-d_power);
            RB.setPower(d_power);
            LF.setPower(d_power);
            LB.setPower(-d_power);
        }

        //winch

        if (gamepad1.y) {
            winch.setTargetPosition(start);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winch.setPower(wPower);
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            winchToggle = !winchToggle;
        }
        if (winchToggle) {
            winch.setTargetPosition(extend);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winch.setPower(wPower);
        }
        else {
        winch.setTargetPosition(retract);
        winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winch.setPower(wPower);
        }

            /*////////////////
            DRIVER 2 CONTROLS
            *////////////////

        //lift
        if (gamepad2.y) {
            top = top + var;
        }
        if (gamepad2.a) {
            top = top - var;
        }

        if (gamepad2.b) {
            lift.setTargetPosition(top);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(lPower);
        }
        if (gamepad2.x) {
            lift.setTargetPosition(bottom);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(lPower);
        }

        //intake
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            intakeForward = !intakeForward;
        }
        if (intakeForward) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            intakeBack = !intakeBack;
        }
        if (intakeBack) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        raVar =  Range.clip(raVar, 5, 355);
        laVar = Range.clip(laVar, 5, 355);

        if (gamepad2.right_stick_y < .2 || gamepad2.right_stick_y > -.2) {
            laVar = laVar + gamepad2.right_stick_y *10;
            raVar = raVar + gamepad2.right_stick_y *10;
        }

        if (gamepad2.left_stick_y < .2 || gamepad2.left_stick_y > -.2) {
            lbVar = lbVar + gamepad2.left_stick_y;
            rbVar = rbVar + gamepad2.left_stick_y;
        }

        rBucket.setPosition(rbVar * .0028);
        lBucket.setPosition(lbVar * .0028);
        lArm.setPosition(laVar * .0028);
        rArm.setPosition(raVar * .0028);

        //telemetry

        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("lift power", lift.getPower());
        telemetry.addData("winch pos", winch.getCurrentPosition());
        telemetry.addData("winch power", winch.getPower());
        telemetry.addData("right bucket pos", rbPos);
        telemetry.addData("left bucket pos", lbPos);
        telemetry.addData("right arm pos", raPos);
        telemetry.addData("left arm pos", laPos);
        telemetry.addData("rbVar", rbVar);
        telemetry.addData("lbVar", lbVar);
        telemetry.addData("raVar", raVar);
        telemetry.addData("laVar", laVar);



        telemetry.update();
    }


}