//package org.firstinspires.ftc.teamcode.ControlFreaks;
//
///**
// * Created by adevries on 11/6/2015.
// */
//
//import android.util.Log;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlock;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlockList;
//import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyCamera;
//
//import java.util.ArrayList;
//import java.util.Calendar;
//import java.util.List;
//
////import com.qualcomm.ftcrobotcontroller.opmodes.ColorSensorDriver;
//
//public class CFPushBotMotor {
//
//    private String config_motor_name = "left_drive";
//
//    public boolean isDriveAndyMark20 = false;
//
//    /*
//        Motor Encoder Vars
//     */
//    /*
//        AndyMark 20 560 Pulses per 360
//        AndyMark 40 1120 Pulses per 360
//        AndyMark 60 1680 Pulses per 360
//     */
//
//
//    private float v_drive_turn_ticks_per_degree;
//    private double v_drive_inches_ticksPerInch;
//    private double v_drive_inches_strife_ticksPerInch;
//
//    //This is the value used to stop before reaching target to account for delay in stop command being processed
//    //private static final int v_drive_inches_ticksStop = 150;
//
//    private float v_drive_power;
//    private float v_drive_power_reverse;
//    private float v_drive_power_slowdown;
//
//
//    private  float v_turn_motorspeed;
//    private  float v_turn_motorspeed_slow;
//    private static final DcMotor.Direction v_drive_leftDirection = DcMotor.Direction.REVERSE;
//    private static final DcMotor.Direction v_drive_rightDirection = DcMotor.Direction.FORWARD;
//
//    private static final DcMotor.Direction v_drive_leftfrontDirection = DcMotor.Direction.REVERSE;
//    private static final DcMotor.Direction v_drive_rightfrontDirection = DcMotor.Direction.FORWARD;
//
//    private boolean v_zeromessage_set = false;
//    //old treads
//    //private static final double driveInches_ticksPerInch = 182.35;
//
//    //new Treads
//    //ticks before target to slow to slowdown 2 speed
//    //private static final double v_drive_inches_ticksSlowDown2 = 1000;
//
//    //private static final float v_drive_power_slowdown2 = .30f;
//
//
//    private boolean v_debug = false;  //set this to false to prevent writing to log makes loop lots shorter
//
//    //We Increment the v_loop_ticks each time through our loop
//    private long v_loop_ticks = 0;
//    // each time through the loop we check to see if v_loop_ticks % v_loop_ticks_slow_count == 0 is so then slow loop = true
//    private int v_loop_ticks_slow_count = 20;  //20
//    private boolean v_loop_ticks_slow = false;
//    private boolean v_drive_use_slowdown;
//    private int v_drive_inches_slowdown;  //slowdown inches before target
//    private int v_drive_inches_slowdown_ticks; //do not set this the inches above is used to calc this on init
//
//    //Global Vars to the class
//    private static final double ServoErrorResultPosition = -0.0000000001;
//
//
//    private DcMotor v_motor_lifter;
//    private static final double v_motor_lifter_power = 1.0;
//    private static final DcMotor.Direction v_motor_lifter_direction = DcMotor.Direction.FORWARD;
//    private int v_motor_lifter_encoder_min = 0;
//    private static final int v_motor_lifter_encoder_max = 2250;
//    private static final int v_motor_lifter_ExtendSlowdownTicks = 300;
//    private int v_motor_lifter_Position = 0;
//
//    //Extender is a AndyMark 20
//    private DcMotor v_motor_extender;
//    private static final double v_motor_extender_power = 1.0f;
//    private static final DcMotor.Direction v_motor_extender_direction = DcMotor.Direction.FORWARD;
//    private static final double v_motor_extender_SpeedSlowDown = 0.5f;
//    private static final int v_motor_extender_encoder_min = 30;
//    private static final int v_motor_extender_encoder_min2 = 815;
//    private static final int v_motor_extender_encoder_max =  3600; //8150; //8750;
//    private static final int v_motor_extender_ExtendSlowdownTicks = 300;
//    private int v_motor_extender_Position = 0;
//
//    LinearOpMode opMode;
//
//
//    private PixyCamera v_pixy;
//
//    private DcMotor v_motor_left_drive;
//    private DcMotor v_motor_right_drive;
//
//    private DcMotor v_motor_leftfront_drive;
//    private DcMotor v_motor_rightfront_drive;
//
//
//    /**
//     * Indicate whether a message is a available to the class user.
//     */
//    private boolean v_warning_generated = false;
//
//    /**
//     * Store a message to the user if one has been generated.
//     */
//    private String v_warning_message = "No Errors";
//
//
//    public CFPushBotMotor()
//    {
//    }
//
//
//
//    //--------------------------------------------------------------------------
//    //
//    // init
//    //
//    /**
//     * Perform any actions that are necessary when the OpMode is enabled.
//     *
//     * The system calls this member once when the OpMode is enabled.
//     */
//    public void init (LinearOpMode ahOpmode)
//
//    {
//        //
//        // Use the hardwareMap to associate class members to hardware ports.
//        //
//        // Note that the names of the devices (i.e. arguments to the get method)
//        // must match the names specified in the configuration file created by
//        // the FTC Robot Controller (Settings-->Configure Robot).
//
//        opMode = ahOpmode;
//        v_warning_generated = false;
//        v_warning_message = "Can't map; ";
//
//        //
//        try
//        {
//            v_motor_left_drive = opMode.hardwareMap.dcMotor.get (config_motor_leftdrive);
//
//            v_motor_left_drive.setDirection (v_drive_leftDirection);
//        }
//        catch (Exception p_exeception)
//        {
//            debugLogException(config_motor_leftdrive,"missing",p_exeception);
//            v_motor_left_drive = null;
//        }
//
//        try
//        {
//            v_motor_right_drive = opMode.hardwareMap.dcMotor.get (config_motor_rightdrive);
//
//            v_motor_right_drive.setDirection (v_drive_rightDirection);
//        }
//        catch (Exception p_exeception)
//        {
//            debugLogException(config_motor_rightdrive, "missing", p_exeception);
//            v_motor_right_drive = null;
//        }
//
//
//        if (isMechDrive){
//            try
//            {
//                v_motor_leftfront_drive = opMode.hardwareMap.dcMotor.get (config_motor_leftfrontdrive);
//
//                v_motor_leftfront_drive.setDirection (v_drive_leftfrontDirection);
//            }
//            catch (Exception p_exeception)
//            {
//                debugLogException(config_motor_leftfrontdrive,"missing",p_exeception);
//                v_motor_leftfront_drive = null;
//            }
//
//            try
//            {
//                v_motor_rightfront_drive = opMode.hardwareMap.dcMotor.get (config_motor_rightfrontdrive);
//
//                v_motor_rightfront_drive.setDirection (v_drive_rightfrontDirection);
//            }
//            catch (Exception p_exeception)
//            {
//                debugLogException(config_motor_rightfrontdrive, "missing", p_exeception);
//                v_motor_rightfront_drive = null;
//            }
//        }
//
//        try {
//            int counter = 0;
//            reset_drive_encoders();
//            while (counter < 10 && have_drive_encoders_reset() == false){
//                counter++;
//                sleep(10);
//                //debugLogException("init", "waiting on  rest_drive_encoders() complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }
//            run_using_encoders();
//            //debugLogException("init", "run_using_encoders() and rest_drive_encoders() complete", null);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("run_using encoders", "error", p_exeception);
//            v_motor_right_drive = null;
//        }
//
//
//
//
//
//        try
//        {
//            v_motor_extender = opMode.hardwareMap.dcMotor.get (config_motor_extender);
//            v_motor_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            v_motor_extender.setDirection(v_motor_extender_direction);
//            v_motor_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            int counter = 0;
//            while (counter < 5 && v_motor_extender.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER){
//                counter++;
//                sleep(10);
//                //debugLogException("init", "waiting on slider motor Stop_and_rest complete",null);
//            }
//            v_motor_extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            counter = 0;
//            while (counter < 5 && v_motor_extender.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
//                counter++;
//                sleep(10);
//                //debugLogException("init", "waiting on slider motor RUN_TO_POSITION complete",null);
//            }
//
//        }
//        catch (Exception p_exeception)
//        {
//            debugLogException(config_motor_extender,"missing",p_exeception);
//            v_motor_extender = null;
//        }
//
//
//
//        try
//        {
//            v_motor_lifter = opMode.hardwareMap.dcMotor.get (config_motor_lifter);
//            v_motor_lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            v_motor_lifter.setDirection(v_motor_lifter_direction);
//            v_motor_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            int counter = 0;
//            while (counter < 5 && v_motor_lifter.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER){
//                counter++;
//                sleep(10);
//                //debugLogException("init", "waiting on lifter motor Stop_and_rest complete",null);
//            }
//            v_motor_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            counter = 0;
//            while (counter < 5 && v_motor_lifter.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
//                counter++;
//                sleep(10);
//                //debugLogException("init", "waiting on lifter motor RUN_TO_Position complete",null);
//            }
//        }
//        catch (Exception p_exeception)
//        {
//            debugLogException(config_motor_lifter,"missing",p_exeception);
//            v_motor_lifter = null;
//        }
//
//
///*
//        try{
//
//            v_ledseg = new AdafruitLEDBackpack7Seg(opMode.hardwareMap, config_i2c_led7seg);
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_led7seg, "missing", p_exeception);
//            v_ledseg = null;
//
//        }
//*/
//
//
//
//        if(isDriveAndyMark20) {
//            drive_setup_am20();
//        }else{
//            drive_setup_am40();
//        }
//        //update our telmentry after init so we know if we are missing anything
//
//        update_telemetry();
//        opMode.telemetry.update();
//
//    } // init
//
//    //--------------------------------------------------------------------------
//    //
//    // a_warning_generated
//    //
//    /**
//     * Access whether a warning has been generated.
//     */
//    boolean a_warning_generated ()
//
//    {
//        return v_warning_generated;
//
//    } // a_warning_generated
//
//
//
//
//
//
//    /**
//     * Used to retrive the total loop count
//     * @return The number of time loop has been executed
//     */
//    public long loopCounter(){
//        return v_loop_ticks;
//    }
//
//    void debugLogException(String type, String msg, Exception ex){
//        if (ex != null){
//            m_warning_message(type);
//        }
//        String debugMessage = type + ":" + msg;
//        if (ex != null) {
//            String errMsg = ex.getMessage();
//            if (errMsg != null) {
//                debugMessage = debugMessage + errMsg;
//            }
//
//            String stackTrace = Log.getStackTraceString(ex);
//            if (stackTrace != null) {
//                debugMessage = debugMessage + "\n" + stackTrace;
//            }
//        }
//        //if (v_debug) {
//        Log.e("CFPushBotHardware", debugMessage);
//        //}
//    }
//
//    void debugPrint(String msg){
//        String debugMessage = msg;
//        Log.d("CFPushBotHardware", debugMessage);
//    }
//
//    void warningPrint(String msg){
//        Log.w("CFPushBotHardware", msg);
//    }
//
//    public boolean sensor_pixy_init(){
//        boolean retval = true;
//        try{
//            v_pixy = new PixyCamera(opMode.hardwareMap, config_i2c_pixy);
//            if(v_pixy == null){
//                warningPrint("v_pixy is null");
//            }
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "missing", p_exeception);
//            v_pixy = null;
//            retval = false;
//        }
////        try{
////            v_pixy_led = opMode.hardwareMap.get(LED.class, config_pixy_led);
////        }catch (Exception p_exeception)
////        {
////            debugLogException(config_pixy_led, "missing", p_exeception);
////            v_pixy_led = null;
////            retval = false;
////        }
//        return retval;
//    }
//
//    public boolean sensor_pixy_set_servos(int s0, int s1){
//        try{
//            if(v_pixy != null) {
//                v_pixy.set_servos(s0, s1);
//            }
//            return true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_set_servos", p_exeception);
//            return false;
//        }
//    }
//
//    public boolean sensor_pixy_enable(boolean enable){
//        try{
//            if(v_pixy != null) {
//                v_pixy.enabled(enable);
//            }
//            return true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_enable", p_exeception);
//            return false;
//        }
//    }
//    public boolean sensor_pixy_signature_enable(int signature, boolean enable){
//        try{
//            if(v_pixy != null) {
//                v_pixy.signature_enable(signature, enable);
//            }else{
//                warningPrint("sensor_pixy_signature_enable: v_pixy is null");
//            }
//            return true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_signature_enable", p_exeception);
//            return false;
//        }
//    }
//
//    public boolean sensor_pixy_maxsignature_enable(int signature, boolean enable){
//        try{
//            if(v_pixy != null) {
//                v_pixy.maxSignature_enable(signature, enable);
//            }else{
//                warningPrint("sensor_pixy_maxsignature_enable: v_pixy is null");
//            }
//            return true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_maxsignature_enable", p_exeception);
//            return false;
//        }
//    }
//
////    public boolean sensor_pixy_led_external(boolean enable){
////        try{
////            if(v_pixy_led != null) {
////
////                v_pixy_led.enable(enable);
////            }
////            return true;
////        }catch (Exception p_exeception)
////        {
////            debugLogException(config_pixy_led, "sensor_pixy_led_external", p_exeception);
////            return false;
////        }
////    }
//
//    public boolean sensor_pixy_set_leds(byte red, byte green, byte blue){
//        try{
//            if(v_pixy != null) {
//                v_pixy.set_led(red,green,blue);
//            }
//            return true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_set_leds", p_exeception);
//            return false;
//        }
//    }
//
//    public PixyBlock sensor_pixy_signatureBlock(int signature){
//        try{
//            if(v_pixy != null) {
//                return v_pixy.largestSignatureBlock(signature);
//            }else {
//                return null;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_signatureBlock", p_exeception);
//            return null;
//        }
//    }
//    public PixyBlockList sensor_pixy_maxSignatureBlocks(int signature){
//        try{
//            if(v_pixy != null) {
//                return v_pixy.maxSignatureBlocks(signature);
//            }else {
//                return null;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_maxSignatureBlocks", p_exeception);
//            return null;
//        }
//    }
//    private void sensor_pixy_loop(){
//        //this is already in hardwareloop this is
//        if(v_pixy != null){
//            v_pixy.loop();
//        }
//    }
//    public void sensor_pixy_getjewelcolor_init(){
//        sensor_pixy_init();
//        sensor_pixy_maxsignature_enable(0,true);
//        //Enable Pixy witch will start the i2c queries
//        //sensor_pixy_signature_enable(2,true);
//    }
//    public int sensor_pixy_getjewelcolor(boolean debug){
//        int blockcolor = -1;
//        try{
//            sensor_pixy_enable(true);
//            String dbg = "Blocks:";
//            /*
//            PixyBlock myblueblock = sensor_pixy_signatureBlock(2);
//            if(debug){
//                dbg = dbg + myblueblock.print();
//            }
//            if(myblueblock.numBlocks > 0){
//                if(myblueblock.x < 100){
//                    return 2;
//                }else{
//                    return 0;
//                }
//            }
//            */
//            PixyBlockList sigMaxBlockList_All;
//            int Sig1X = -1;//Sig 1 is red
//            int Sig2X = -1; //Sig 2 is Blue
//            sigMaxBlockList_All = sensor_pixy_maxSignatureBlocks(0);
//            dbg = "Blocks:" + sigMaxBlockList_All.BlockCount;
//            if(sigMaxBlockList_All.BlockCount > 0){
//
//                for(int i = 0; i < sigMaxBlockList_All.Blocks.length && i < sigMaxBlockList_All.BlockCount; i++ ){
//                    //we assume the largest blue and red are the balls so will come first
//                    if(sigMaxBlockList_All.Blocks[i].signature == 1 && Sig1X == -1){
//                        //avoid left 50 x so not to read red in vuforia pictures with red in them
//                        if(sigMaxBlockList_All.Blocks[i].x > 5) {
//                            Sig1X = sigMaxBlockList_All.Blocks[i].x;
//                        }
//                    }else if(sigMaxBlockList_All.Blocks[i].signature == 2 && Sig2X == -1){
//                        Sig2X =  sigMaxBlockList_All.Blocks[i].x;
//                    }
//                    if(Sig1X > 0 && Sig2X > 0){
//                        break; //exit the 4 loop we already found the largest Red and Blue
//                    }
//
//                }
//                //The Block we care abount is always on the Left so lowest X value
//                if(Sig1X > 0 && Sig2X > 0) {
//                    if (Sig1X < Sig2X) {
//                        blockcolor = 0;  //Ball on Left is Signature 1 Red
//                    } else {
//                        blockcolor = 2;  //Ball on Left is Signature 2 Blue
//                    }
//                }
//                if(debug){
//                    dbg = dbg + sigMaxBlockList_All.print();
//                }
//            }
//            set_second_message(dbg);
//        }catch(Exception ex){
//            debugLogException("robot","sensor_pixy_getjewelcolor",ex );
//            set_error_message("sensor_pixy_getjewelcolor: " + ex.getMessage());
//        }
//        return blockcolor;
//    }
//    public void sensor_pixy_signature_colorcode_set(int colorcode){
//        try{
//            if(v_pixy != null) {
//                v_pixy.color_code = colorcode;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_signature_colorcode_set", p_exeception);
//        }
//    }
//
//    public int sensor_pixy_signature_colorcode(){
//        try{
//            if(v_pixy != null) {
//                return v_pixy.color_code;
//            }else {
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException(config_i2c_pixy, "sensor_pixy_signature_colorcode", p_exeception);
//            return 0;
//        }
//    }
//
//    private ElapsedTime v_timewait2_elapsedtime;
//    private boolean v_is_timewaiting2_complete;
//    private int v_timewait2_milliseconds;
//
//    private ElapsedTime v_timewait_elapsedtime;
//    private boolean v_is_timewaiting_complete;
//    private float v_timewait_seconds;
//
//    public boolean timewait(float seconds){
//        v_timewait_elapsedtime = new ElapsedTime();
//        v_timewait_seconds = seconds;
//        v_is_timewaiting_complete = false;
//        return true;
//    }
//
//    public boolean timewait_Complete(){
//        if ( v_timewait_elapsedtime.seconds() > v_timewait_seconds ){
//            v_is_timewaiting_complete = true;
//        }
//        return v_is_timewaiting_complete;
//    }
//
//    private boolean timewait2Milliseconds(int milliseconds){
//        v_timewait2_elapsedtime = new ElapsedTime();
//        v_timewait2_milliseconds = milliseconds;
//        v_is_timewaiting2_complete = false;
//        return true;
//    }
//
//    private boolean timewait2Milliseconds_Complete(){
//        if ( v_timewait2_elapsedtime.milliseconds() >= v_timewait2_milliseconds ){
//            v_is_timewaiting2_complete = true;
//        }
//        return v_is_timewaiting2_complete;
//    }
//
//    //--------------------------------------------------------------------------
//    //
//    // a_warning_message
//    //
//    /**
//     * Access the warning message.
//     */
//    String a_warning_message ()
//
//    {
//        return v_warning_message;
//
//    } // a_warning_message
//
//    //--------------------------------------------------------------------------
//    //
//    // m_warning_message
//    //
//    /**
//     * Mutate the warning message by ADDING the specified message to the current
//     * message; set the warning indicator to true.
//     *
//     * A comma will be added before the specified message if the message isn't
//     * empty.
//     */
//    void m_warning_message (String p_exception_message)
//
//    {
//        if (v_warning_generated)
//        {
//            v_warning_message += ", ";
//        }
//        v_warning_generated = true;
//        v_warning_message += p_exception_message;
//
//    } // m_warning_message
//
//    //--------------------------------------------------------------------------
//    //
//    // start
//    //
//    /**
//     * Perform any actions that are necessary when the OpMode is enabled.
//     *
//     * The system calls this member once when the OpMode is enabled.
//     */
//
//
//
//    //--------------------------------------------------------------------------
//    //
//    // stop
//    //
//    /**
//     * Perform any actions that are necessary when the OpMode is disabled.
//     *
//     * The system calls this member once when the OpMode is disabled.
//     */
//    public void stop ()
//    {
//        //
//        // Nothing needs to be done for this method.
//        //
//        hardware_stop();
//    } // stop
//
//
//    //We use this to throttle methods that get called each hardware loop but only need to service rutine every so many loops.
//    //We can keep our loop performance up and still use high latency calls like get color etc.
//    public boolean is_slow_tick(){
//        return  v_loop_ticks_slow;
//    }
//
//
//    /** called each time through the loop needed to sync hardware and look for status changes
//     *
//     */
//    private Calendar v_loop_previous_timestamp;
//    private long v_slow_loop_milliseconds = 0;
//    public void hardware_loop() throws InterruptedException{
//
//        v_loop_ticks++;
//        if(v_loop_ticks % v_loop_ticks_slow_count == 0 ){
//            v_loop_ticks_slow = true;
//            if (v_loop_previous_timestamp != null){
//                v_slow_loop_milliseconds = Calendar.getInstance().getTimeInMillis() - v_loop_previous_timestamp.getTimeInMillis();
//            }
//            v_loop_previous_timestamp = Calendar.getInstance();
//        }else{
//            v_loop_ticks_slow = false;
//        }
//        if(v_sensor_rangeSensor != null && v_sensor_rangeSensor_enabled == true){
//            v_sensor_rangeSensor_distance = v_sensor_rangeSensor.getDistance(DistanceUnit.INCH);
//        }
//
//
//        if(v_pixy != null){
//            v_pixy.loop();
//        }
//
//
//
//        if(v_loop_ticks_slow){
//
//            //heartbeat_tick();
//            if(v_ledseg != null){
//                v_ledseg.loop();
//            }
//
//            // get the heading info.
//            // the Modern Robotics' gyro sensor keeps
//            // track of the current heading for the Z axis only.
////            if(v_sensor_gyro != null) {
////                v_sensor_gyro_heading = v_sensor_gyro.getHeading();
////                v_sensor_gyro_x = v_sensor_gyro.rawX();
////                v_sensor_gyro_y = v_sensor_gyro.rawY();
////                v_sensor_gyro_z = v_sensor_gyro.rawZ();
////            }
//            //the i2c color sensor uses a memory lock that is taxing so we only do this if we are using the color sensor and ever slow loop count
//            if(v_sensor_color_i2c != null && v_sensor_color_i2c_enabled == true){
//                v_sensor_color_i2c_rgbaValues[0] = v_sensor_color_i2c.red();
//                v_sensor_color_i2c_rgbaValues[1] = v_sensor_color_i2c.green();
//                v_sensor_color_i2c_rgbaValues[2] = v_sensor_color_i2c.blue();
//                v_sensor_color_i2c_rgbaValues[3] = v_sensor_color_i2c.alpha();
//                set_first_message("color:" + v_sensor_color_i2c_rgbaValues[0] + ":" + v_sensor_color_i2c_rgbaValues[1] + ":" + v_sensor_color_i2c_rgbaValues[2] + ":" + v_sensor_color_i2c_rgbaValues[3]);
//            }
//
//
//
//            //jewel ease so not to hard hit ground
//            if(v_servo_jewel_is_retracting){
//                if( v_servo_jewel_position > v_servo_jewel_MinPosition){
//                    v_servo_jewel_position = v_servo_jewel_position - .05;
//                    v_servo_jewel.setPosition(v_servo_jewel_position);
//                }else{
//                    v_servo_jewel.setPosition(v_servo_jewel_MinPosition);
//                    v_servo_jewel_is_retracting = false;
//                }
//            }
//
//
//            if(v_debug || v_zeromessage_set || v_errormessage_set) {
//                update_telemetry();
//                opMode.updateTelemetry(opMode.telemetry);
//            }
//        }
//
//        opMode.idle();
//        //waitForTick(5);
//    }
//
//    public long hardware_loop_slowtime_milliseconds(){
//        return v_slow_loop_milliseconds;
//    }
//
//    public void hardware_stop(){
//        /*if(v_led_heartbeat !=null){
//            v_led_heartbeat.enable(false);
//        }*/
//        if(v_ledseg != null){
//            v_ledseg.stop();
//        }
//
//        if(v_motor_left_drive != null){
//            v_motor_left_drive.setPower(0);
//        }
//        if(v_motor_right_drive != null){
//            v_motor_right_drive.setPower(0);
//        }
//        if(v_motor_lifter != null){
//            v_motor_lifter.setPower(0);
//        }
//        if(v_motor_slider != null){
//            v_motor_slider.setPower(0);
//        }
//    }
//
////    //--------------------------------------------------------------------------
////    //
////    // scale_motor_power
////    //
////    /**
////     * Scale the joystick input using a nonlinear algorithm.
////     */
////    float scale_motor_power (float p_power)
////    {
////        //
////        // Assume no scaling.
////        //
////        float l_scale = 0.0f;
////
////        //
////        // Ensure the values are legal.
////        //
////        float l_power = Range.clip (p_power, -1, 1);
////
////        float[] l_array =
////                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
////                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
////                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
////                        , 1.00f, 1.00f
////                };
////
////        //
////        // Get the corresponding index for the specified argument/parameter.
////        //
////        int l_index = (int)(l_power * 16.0);
////        if (l_index < 0)
////        {
////            l_index = -l_index;
////        }
////        else if (l_index > 16)
////        {
////            l_index = 16;
////        }
////
////        if (l_power < 0)
////        {
////            l_scale = -l_array[l_index];
////        }
////        else
////        {
////            l_scale = l_array[l_index];
////        }
////
////        return l_scale;
////
////    } // scale_motor_power
//
//    //--------------------------------------------------------------------------
//    //
//    // a_left_drive_power
//    //
//    /**
//     * Access the left drive motor's power level.
//     */
//    double a_left_drive_power ()
//    {
//        double l_return = 0.0;
//
//        if (v_motor_left_drive != null)
//        {
//            l_return = v_motor_left_drive.getPower ();
//        }
//
//        return l_return;
//
//    } // a_left_drive_power
//
//    //--------------------------------------------------------------------------
//    //
//    // a_right_drive_power
//    //
//    /**
//     * Access the right drive motor's power level.
//     */
//    double a_right_drive_power ()
//    {
//        double l_return = 0.0;
//
//        if (v_motor_right_drive != null)
//        {
//            l_return = v_motor_right_drive.getPower ();
//        }
//
//        return l_return;
//
//    } // a_right_drive_power
//
//    //--------------------------------------------------------------------------
//    //
//    // set_drive_power
//    //
//    /**
//     * Scale the joystick input using a nonlinear algorithm.
//     */
//    // float l_left_drive_power = 0.0f;
//    // float l_right_drive_power = 0.0f;
//    public void drive_set_power (float p_left_power, float p_right_power)
//    {
//        try {
//            float l_left_drive_power = Range.clip(p_left_power, -1, 1);
//            float l_right_drive_power = Range.clip(p_right_power, -1, 1);
//
//            if (v_motor_left_drive != null) {
//                v_motor_left_drive.setPower(l_left_drive_power);
//            }
//            if (v_motor_right_drive != null) {
//                v_motor_right_drive.setPower(l_right_drive_power);
//            }
//            if(isMechDrive){
//                if (v_motor_leftfront_drive != null) {
//                    v_motor_leftfront_drive.setPower(l_left_drive_power);
//                }
//                if (v_motor_rightfront_drive != null) {
//                    v_motor_rightfront_drive.setPower(l_right_drive_power);
//                }
//            }
//        }catch(Exception ex) {
//            debugLogException("robot", "set_drive_power", ex );
//        }
//        //set_second_message("set_drive_power l" + p_left_power + ":r" + p_right_power + " cliped:l:" + l_left_drive_power +":r" + l_right_drive_power);
//    } // set_drive_power
//
//    //--------------------------------------------------------------------------
//    //
//    // set_drive_strife_power
//    //
//    /**
//     * Scale the joystick input using a nonlinear algorithm.
//     */
//    public void drive_set_strife_power (float power)
//    {
//        try {
//
//            if(isMechDrive){
//                float drive_power = Range.clip(power, -1, 1);
//
//                if (v_motor_left_drive != null) {
//                    v_motor_left_drive.setPower(0-drive_power);
//                }
//                if (v_motor_right_drive != null) {
//                    v_motor_right_drive.setPower(drive_power);
//                }
//                if (v_motor_leftfront_drive != null) {
//                    v_motor_leftfront_drive.setPower(drive_power);
//                }
//                if (v_motor_rightfront_drive != null) {
//                    v_motor_rightfront_drive.setPower(0-drive_power);
//                }
//            }
//        }catch(Exception ex) {
//            debugLogException("robot", "set_drive_power", ex );
//        }
//        //set_second_message("set_drive_power l" + p_left_power + ":r" + p_right_power + " cliped:l:" + l_left_drive_power +":r" + l_right_drive_power);
//    } // set_drive_power
//
////    //--------------------------------------------------------------------------
////    //
////    // set_drive_power
////    //
////    /**
////     * Scale the joystick input using a nonlinear algorithm.
////     */
////    // float l_left_drive_power = 0.0f;
////    // float l_right_drive_power = 0.0f;
////    public void drive_set_power_scaled (float p_left_power, float p_right_power)
////    {
////        try{
////        float l_left_drive_power = scale_motor_power(p_left_power);
////        float l_right_drive_power = scale_motor_power(p_right_power);
////
////        if (v_motor_left_drive != null)
////        {
////            v_motor_left_drive.setPower (l_left_drive_power);
////        }
////        if (v_motor_right_drive != null)
////        {
////            v_motor_right_drive.setPower(l_right_drive_power);
////        }
////        //set_second_message("set_drive_power " + p_left_power + ":" + p_right_power + " " + l_left_drive_power +":" + l_right_drive_power);
////        }catch(Exception ex) {
////            debugLogException("robot", "set_drive_power_scaled", ex );
////        }
////    } // set_drive_power
//
//
//
//
//
//    public void extender_extend () throws InterruptedException
//    {
//        if (v_motor_extender != null)
//        {
//
//            v_extender_state = 0;
//            v_motor_extender_Position = v_motor_extender_Position + v_motor_extender_encoder_max - v_motor_extender_ExtendSlowdownTicks;
//
//            v_motor_extender.setTargetPosition(v_motor_extender_Position);
//            set_second_message("extendinging extender");
//            v_motor_extender.setPower(v_motor_extender_power);
//
//        }
//    }
//
//    private int v_extender_state = 0;
//    public boolean extender_extend_complete () {
//        if (v_motor_extender != null) {
//            switch (v_extender_state) {
//                case 0:
//                    if (v_motor_extender.isBusy() == false) {
//                        v_motor_extender_Position = v_motor_extender_Position + v_motor_extender_ExtendSlowdownTicks;
//                        v_motor_extender.setPower(0.0F);
//                        v_motor_extender.setTargetPosition(v_motor_extender_Position);
//                        set_second_message("slider almost extended");
//                        v_motor_extender.setPower(0.5F);
//
//                        v_extender_state++;
//                    }
//                    break;
//                case 1:
//                    if (v_motor_extender.isBusy() == false) {
//                        v_motor_extender.setPower(0.0F);
//                        v_slider_isExtended = true;
//                        set_second_message("slider loaded");
//                        return true;
//                    }
//                    break;
//            }
//            return false;
//
//        }else{
//            return true;
//        }
//    }
//
//
//    //Retract the slider
//
//    //--------------------------------------------------------------------------
//    //
//    // Retract_slider
//    //
//    /**
//     * Retract the slider
//     */
//    private boolean v_slider_isExtended = false;
//    public void slider_step (int stepAmount, boolean min2Override)
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                int cPosition = v_motor_slider.getCurrentPosition();
//                v_motor_slider_Position =  cPosition + stepAmount;
//                if(min2Override ==false && v_motor_slider_Position < v_motor_slider_encoder_min2 && cPosition > v_motor_slider_encoder_min2  ){
//                    v_motor_slider_Position = v_motor_slider_encoder_min2;
//
//                }else if(v_motor_slider_Position < v_motor_slider_encoder_min && min2Override ==true){
//                    v_motor_slider_Position = v_motor_slider_encoder_min;
//
//                }else if(v_motor_slider_Position > v_motor_slider_encoder_max) {
//                    v_motor_slider_Position = v_motor_slider_encoder_max;
//                }
//
//                v_motor_slider.setTargetPosition(v_motor_slider_Position);
//                v_motor_slider.setPower(v_motor_slider_power);
//            }
//            set_second_message("Slider Step " + v_motor_slider_Position + " minOverride:" + min2Override);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_step", "error", p_exeception);
//        }
//    }
//    public void slider_stop (boolean min2Override)
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                v_motor_slider.setPower(0);
//                v_motor_slider_Position = v_motor_slider.getCurrentPosition();
//                if(min2Override ==false && v_motor_slider_Position < v_motor_slider_encoder_min2 && v_motor_slider_Position > v_motor_slider_encoder_min2  ){
//                    v_motor_slider_Position = v_motor_slider_encoder_min2;
//
//                }else if(v_motor_slider_Position < v_motor_slider_encoder_min && min2Override ==true){
//                    v_motor_slider_Position = v_motor_slider_encoder_min;
//
//                }else if(v_motor_slider_Position > v_motor_slider_encoder_max) {
//                    v_motor_slider_Position = v_motor_slider_encoder_max;
//                }
//            }
//            set_second_message("Slider Stop " + v_motor_slider_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_stop", "error", p_exeception);
//        }
//    }
//
//    public void slider_hold ()
//    {
//        try{
//            if (v_motor_slider != null )
//            {
//                v_motor_slider.setTargetPosition(v_motor_slider_Position);
//                v_motor_slider.setPower(.2);
//            }
//            set_second_message("Slider Stop " + v_motor_slider_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("slider_stop", "error", p_exeception);
//        }
//    }
//
//    public void slider_retract ()
//    {
//
//        if (v_motor_slider != null )
//        {
//
//            v_motor_slider.setTargetPosition(v_motor_slider_encoder_min);
//            set_second_message("Retracting slider");
//            v_motor_slider.setPower(v_motor_slider_power);
//        }
//
//    }
//    public boolean slider_retract_complete ()
//    {
//        if (v_motor_slider != null )
//        {
//
//            if (v_motor_slider.isBusy()== false) {
//                v_motor_slider.setPower(0.0F);
//                v_slider_isExtended = false;
//                set_second_message("Retracted slider");
//                return true;
//            }
//        }
//        return false;
//    }
//
//
//    public void lifter_extend () throws InterruptedException
//    {
//        if (v_motor_lifter != null)
//        {
////            if (v_lifter_isExtended == true){
////                set_second_message("lifter already extended");
////                return;
////            }
//            v_lifter_state = 0;
//            v_motor_lifter_Position = v_motor_lifter_Position + v_motor_lifter_encoder_max - v_motor_lifter_ExtendSlowdownTicks;
//
//            v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//            set_second_message("extendinging lifter");
//            v_motor_lifter.setPower(v_motor_lifter_power);
//
//        }
//    }
//
//    private int v_lifter_state = 0;
//    public boolean lifter_extend_complete () {
//        if (v_motor_lifter != null) {
//            switch (v_lifter_state) {
//                case 0:
//                    if (v_motor_lifter.isBusy() == false) {
//                        v_motor_lifter_Position = v_motor_lifter_Position + v_motor_lifter_ExtendSlowdownTicks;
//                        v_motor_lifter.setPower(0.0F);
//                        v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//                        set_second_message("lifter almost extended");
//                        v_motor_lifter.setPower(0.5F);
//
//                        v_lifter_state++;
//                    }
//                    break;
//                case 1:
//                    if (v_motor_lifter.isBusy() == false) {
//                        v_motor_lifter.setPower(0.0F);
//                        v_lifter_isExtended = true;
//                        set_second_message("lifter loaded");
//                        return true;
//                    }
//                    break;
//            }
//            return false;
//
//        }else{
//            return true;
//        }
//    }
//
//
//
//    /**
//     * Retract the slider
//     */
//    private boolean v_lifter_isExtended = false;
//    public void lifter_step (int stepAmount)
//    {
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter_Position = v_motor_lifter.getCurrentPosition() + stepAmount;
//                if(v_motor_lifter_Position <= v_motor_lifter_encoder_min  ) {
//                    v_motor_lifter_Position = v_motor_lifter_encoder_min;
//                }
//                if(v_motor_lifter_Position >= v_motor_lifter_encoder_max  ) {
//                    v_motor_lifter_Position = v_motor_lifter_encoder_max;
//                }
//                if(v_motor_lifter_Position >= v_motor_lifter_encoder_min && v_motor_lifter_Position <= v_motor_lifter_encoder_max ) {
//                    v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//                    v_motor_lifter.setPower(v_motor_lifter_power);
//                }
//            }
//            set_second_message("lifter Step " + v_motor_lifter_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_step", "error", p_exeception);
//        }
//    }
//
//    public void lifter_stop ()
//    {
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter.setPower(0);
//                v_motor_lifter_Position = v_motor_lifter.getCurrentPosition();
//                v_motor_lifter.setTargetPosition(v_motor_lifter_Position);
//            }
//            set_second_message("lifter Stop " + v_motor_lifter_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_stop", "error", p_exeception);
//        }
//    }
//
//    public void lifter_testbot_reverse(){
//        try{
//            if (v_motor_lifter != null )
//            {
//                v_motor_lifter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            }
//            set_second_message("lifter testbot Reverse ");
//        }catch (Exception p_exeception)
//        {
//            debugLogException("lifter_testbot_reverse", "error", p_exeception);
//        }
//    }
//
//    public void lifter_stepmin(int steps){
//        v_motor_lifter_encoder_min = v_motor_lifter_encoder_min + steps;
//    }
//
//    public void lifter_retract ()
//    {
//
//        if (v_motor_lifter != null )
//        {
////            if (v_lifter_isExtended == false){
////                set_second_message("lifter not loaded");
////                return;
////            }
//            v_motor_lifter.setTargetPosition(v_motor_lifter_encoder_min);
//            set_second_message("Retracting lifter");
//            v_motor_lifter.setPower(v_motor_lifter_power);
//        }
//
//    }
//    public boolean lifter_retract_complete ()
//    {
//        if (v_motor_lifter != null )
//        {
//
//            if (v_motor_lifter.isBusy()== false) {
//                v_motor_lifter.setPower(0.0F);
//                v_lifter_isExtended = false;
//                set_second_message("Retracted lifter");
//                return true;
//            }
//        }
//        return false;
//    }
//
//
//    //BGTilt Begin
//
//    public void bgtilt_extend () throws InterruptedException
//    {
//        if (v_motor_bgtilt != null)
//        {
//
//            v_bgtilt_state = 0;
//            v_motor_bgtilt_Position = v_motor_bgtilt_Position + v_motor_bgtilt_encoder_max - v_motor_bgtilt_ExtendSlowdownTicks;
//
//            v_motor_bgtilt.setTargetPosition(v_motor_bgtilt_Position);
//            set_second_message("extendinging bgtilt");
//            v_motor_bgtilt.setPower(v_motor_bgtilt_power);
//
//        }
//    }
//
//    private int v_bgtilt_state = 0;
//    public boolean bgtilt_extend_complete () {
//        if (v_motor_bgtilt != null) {
//            switch (v_bgtilt_state) {
//                case 0:
//                    if (v_motor_bgtilt.isBusy() == false) {
//                        v_motor_bgtilt_Position = v_motor_bgtilt_Position + v_motor_bgtilt_ExtendSlowdownTicks;
//                        v_motor_bgtilt.setPower(0.0F);
//                        v_motor_bgtilt.setTargetPosition(v_motor_bgtilt_Position);
//                        set_second_message("bgtilt almost extended");
//                        v_motor_bgtilt.setPower(0.5F);
//
//                        v_lifter_state++;
//                    }
//                    break;
//                case 1:
//                    if (v_motor_bgtilt.isBusy() == false) {
//                        v_motor_bgtilt.setPower(0.0F);
//                        v_bgtilt_isExtended = true;
//                        set_second_message("bgtilt loaded");
//                        return true;
//                    }
//                    break;
//            }
//            return false;
//
//        }else{
//            return true;
//        }
//    }
//
//
//
//    private boolean v_bgtilt_isExtended = false;
//    public void bgtilt_step (int stepAmount)
//    {
//        try{
//            if (v_motor_bgtilt != null )
//            {
//                v_motor_bgtilt_Position = v_motor_bgtilt.getCurrentPosition() + stepAmount;
//                if(v_motor_bgtilt_Position <= v_motor_bgtilt_encoder_min  ) {
//                    v_motor_bgtilt_Position = v_motor_bgtilt_encoder_min;
//                }
//                if(v_motor_bgtilt_Position >= v_motor_bgtilt_encoder_max  ) {
//                    v_motor_bgtilt_Position = v_motor_bgtilt_encoder_max;
//                }
//                if(v_motor_bgtilt_Position >= v_motor_bgtilt_encoder_min && v_motor_bgtilt_Position <= v_motor_bgtilt_encoder_max ) {
//                    v_motor_bgtilt.setTargetPosition(v_motor_bgtilt_Position);
//                    v_motor_bgtilt.setPower(v_motor_bgtilt_power);
//                }
//            }
//            set_second_message("bgtilt Step " + v_motor_bgtilt_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("bgtilt_step", "error", p_exeception);
//        }
//    }
//
//    public void bgtilt_stop ()
//    {
//        try{
//            if (v_motor_bgtilt != null )
//            {
//                v_motor_bgtilt.setPower(0);
//                v_motor_bgtilt_Position = v_motor_bgtilt.getCurrentPosition();
//                v_motor_bgtilt.setTargetPosition(v_motor_bgtilt_Position);
//            }
//            set_second_message("bgtilt Stop " + v_motor_bgtilt_Position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("bgtilt_stop", "error", p_exeception);
//        }
//    }
//
//
//
//    public void bgtilt_stepmin(int steps){
//        v_motor_bgtilt_encoder_min = v_motor_bgtilt_encoder_min + steps;
//    }
//
//    public void bgtilt_retract ()
//    {
//
//        if (v_motor_bgtilt != null )
//        {
//            v_motor_bgtilt.setTargetPosition(v_motor_bgtilt_encoder_min);
//            set_second_message("Retracting bgtilt");
//            v_motor_bgtilt.setPower(v_motor_bgtilt_power);
//        }
//
//    }
//    public boolean bgtilt_retract_complete ()
//    {
//        if (v_motor_bgtilt != null )
//        {
//
//            if (v_motor_bgtilt.isBusy()== false) {
//                v_motor_bgtilt.setPower(0.0F);
//                v_bgtilt_isExtended = false;
//                set_second_message("Retracted bgtilt");
//                return true;
//            }
//        }
//        return false;
//    }
//
//    //BGTilt End
//
//    public void run_to_position(float power, float inches ) throws InterruptedException{
//        //setupDriveToPosition();
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opMode.opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = v_motor_left_drive.getCurrentPosition() + (int) (inches * v_drive_inches_ticksPerInch);
//            newRightTarget = v_motor_right_drive.getCurrentPosition() + (int) (inches * v_drive_inches_ticksPerInch);
//            v_motor_left_drive.setTargetPosition(newLeftTarget);
//            v_motor_right_drive.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            drive_set_power(power, power);
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            while (opMode.opModeIsActive() &&
//                    (v_motor_left_drive.isBusy() && v_motor_right_drive.isBusy())) {
//
//                // Display it for the driver.
//                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
//                        v_motor_left_drive.getCurrentPosition(),
//                        v_motor_right_drive.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//        }
//    }
//
//    public void run_without_encoders(){
//        v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    //--------------------------------------------------------------------------
//    //
//    // run_using_left_drive_encoder
//    //
//    /**
//     * Set the left drive wheel encoder to run, if the mode is appropriate.
//     */
//    public void run_using_left_drive_encoder ()
//
//    {
//        if (v_motor_left_drive != null)
//        {
//            v_motor_left_drive.setMode
//                    (DcMotor.RunMode.RUN_USING_ENCODER
//                    );
//        }
//
//    } // run_using_left_drive_encoder
//
//    //--------------------------------------------------------------------------
//    //
//    // run_using_right_drive_encoder
//    //
//    /**
//     * Set the right drive wheel encoder to run, if the mode is appropriate.
//     */
//    public void run_using_right_drive_encoder ()
//
//    {
//        if (v_motor_right_drive != null)
//        {
//            v_motor_right_drive.setMode
//                    (DcMotor.RunMode.RUN_USING_ENCODER
//                    );
//        }
//
//    } // run_using_right_drive_encoder
//
//    //--------------------------------------------------------------------------
//    //
//    // run_using_leftfront_drive_encoder
//    //
//    /**
//     * Set the left drive wheel encoder to run, if the mode is appropriate.
//     */
//    public void run_using_leftfront_drive_encoder ()
//
//    {
//        if (v_motor_leftfront_drive != null)
//        {
//            v_motor_leftfront_drive.setMode
//                    (DcMotor.RunMode.RUN_USING_ENCODER
//                    );
//        }
//
//    } // run_using_leftfront_drive_encoder
//
//    //--------------------------------------------------------------------------
//    //
//    // run_using_rightfront_drive_encoder
//    //
//    /**
//     * Set the right front drive wheel encoder to run, if the mode is appropriate.
//     */
//    public void run_using_rightfront_drive_encoder ()
//
//    {
//        if (v_motor_rightfront_drive != null)
//        {
//            v_motor_rightfront_drive.setMode
//                    (DcMotor.RunMode.RUN_USING_ENCODER
//                    );
//        }
//
//    } // run_using_rightfront_drive_encoder
//
//    //--------------------------------------------------------------------------
//    //
//    // run_using_encoders
//    //
//    /**
//     * Set both drive wheel encoders to run, if the mode is appropriate.
//     */
//    public void run_using_encoders ()
//
//    {
//        //
//        // Call other members to perform the action on both motors.
//        //
//        run_using_left_drive_encoder ();
//        run_using_right_drive_encoder ();
//        run_using_leftfront_drive_encoder ();
//        run_using_rightfront_drive_encoder ();
//
//    } // run_using_encoders
///*
//
//    //--------------------------------------------------------------------------
//    //
//    // run_without_left_drive_encoder
//    //
//    */
//    /**
//     * Set the left drive wheel encoder to run, if the mode is appropriate.
//     *//*
//
//    public void run_without_left_drive_encoder ()
//
//    {
//        if (v_motor_left_drive != null)
//        {
//            if (v_motor_left_drive.getMode() ==
//                    DcMotorController.RunMode.RESET_ENCODERS)
//            {
//                v_motor_left_drive.setMode
//                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
//                        );
//            }
//        }
//
//    } // run_without_left_drive_encoder
//*/
//
//    public boolean sound_play_dtmf(int tone, int duration){
//        if (v_tone_generator != null) {
//            v_tone_generator.startTone(tone, duration);
//            return true;
//        }else{
//            return false;
//        }
//    }
//
//    /*//--------------------------------------------------------------------------
//    //
//    // run_without_right_drive_encoder
//    //
//    *//**
//     * Set the right drive wheel encoder to run, if the mode is appropriate.
//     *//*
//    public void run_without_right_drive_encoder ()
//
//    {
//        if (v_motor_right_drive != null)
//        {
//            if (v_motor_right_drive.getMode() ==
//                    DcMotorController.RunMode.RESET_ENCODERS)
//            {
//                v_motor_right_drive.setMode
//                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
//                        );
//            }
//        }
//
//    } // run_without_right_drive_encoder
//
//    //--------------------------------------------------------------------------
//    //
//    // run_without_drive_encoders
//    //
//    *//**
//     * Set both drive wheel encoders to run, if the mode is appropriate.
//     *//*
//    public void run_without_drive_encoders ()
//
//    {
//        //
//        // Call other members to perform the action on both motors.
//        //
//        run_without_left_drive_encoder ();
//        run_without_right_drive_encoder ();
//
//    } // run_without_drive_encoders
//*/
//    //--------------------------------------------------------------------------
//    //
//    // reset_left_drive_encoder
//    //
//    /**
//     * Reset the left drive wheel encoder.
//     */
//    public void reset_left_drive_encoder ()
//
//    {
//        if (v_motor_left_drive != null)
//        {
//            //This may Cause a Stop Now versus just a rest andy 09/24/2016
//            v_motor_left_drive.setMode
//                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                    );
//        }
//
//    } // reset_left_drive_encoder
//
//    public boolean isInDriveMode(DcMotor.RunMode RunMode){
//        if (v_motor_left_drive != null && v_motor_right_drive != null){
//            if(v_motor_left_drive.getMode() == RunMode && v_motor_right_drive.getMode() == RunMode
//                && (
//                        isMechDrive == false
//                        || (isMechDrive==true && v_motor_leftfront_drive.getMode() == RunMode && v_motor_rightfront_drive.getMode() == RunMode )
//                    )
//                ){
//                return true;
//            }else{
//                return false;
//            }
//        }
//        return true;
//    }
//
//    //--------------------------------------------------------------------------
//    //
//    // reset_right_drive_encoder
//    //
//    /**
//     * Reset the right drive wheel encoder.
//     */
//    public void reset_right_drive_encoder ()
//
//    {
//        if (v_motor_right_drive != null)
//        {
//            v_motor_right_drive.setMode
//                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                    );
//        }
//
//    } // reset_right_drive_encoder
//
//    //
//    /**
//     * Reset the right front drive wheel encoder.
//     */
//    public void reset_rightfront_drive_encoder ()
//
//    {
//        if (v_motor_rightfront_drive != null)
//        {
//            v_motor_rightfront_drive.setMode
//                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                    );
//        }
//
//    } // reset_rightfront_drive_encoder
//
//    public void reset_leftfront_drive_encoder ()
//
//    {
//        if (v_motor_leftfront_drive != null)
//        {
//            v_motor_leftfront_drive.setMode
//                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                    );
//        }
//
//    } // reset_leftfront_drive_encoder
//
//    public void setupAutoDrive() {
//
//        if(isInDriveMode(DcMotor.RunMode.RUN_USING_ENCODER) == false){
//            drive_set_power(0.0f,0.0f);
//            run_using_encoders();
//            int counter = 0;
//
//            while (counter < 10 &&isInDriveMode(DcMotor.RunMode.RUN_USING_ENCODER)==false){
//                counter++;
//                sleep(100);
//                //debugLogException("init", "waiting on  DcMotor.RunMode.RUN_USING_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }
//            if(counter > 1){
//                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_USING_ENCODER) complete. Counter: " + counter, null);
//            }
//
//
//        }
//
//    }
//    public void setupManualDrive() {
//
//        if (isInDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) == false){
//            drive_set_power(0.0f,0.0f);
//            run_using_encoders();
//            int counter = 0;
//            while (counter < 5 && isInDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)==false){
//                counter++;
//                sleep(50);
//                //debugLogException("init", "waiting on  DcMotor.RunMode.RUN_WITHOUT_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }
//            if(counter > 1){
//                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_WITHOUT_ENCODER) complete. Counter: " + counter, null);
//            }
//        }
//
//
//    }
//
//    public void setupDriveToPosition(){
//
//        if (isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION) == false){
//            //reset_drive_encoders();
//            int counter = 0;
//            /*while (counter < 20 && isInDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)==false){
//                counter++;
//                sleep(100);
//                debugLogException("init", "waiting on  DcMotor.RunMode.STOP_AND_RESET_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }*/
//            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if(isMechDrive ){
//                v_motor_leftfront_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                v_motor_rightfront_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            while (counter < 10 && isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION)==false){
//                counter++;
//                sleep(10);
//            }
//            if(counter > 1){
//                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_TO_POSITION) complete. Counter: " + counter, null);
//            }
//        }
//
//    }
///*
//    public void setDriveToPosition(){
//
//        if (isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION) == false){
//            //reset_drive_encoders();
//            int counter = 0;
//            //while (counter < 10 && isInDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)==false){
//            //    counter++;
//            //    sleep(100);
//            //    debugLogException("init", "waiting on  DcMotor.RunMode.STOP_AND_RESET_ENCODER) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            //}
//            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (counter < 10 && isInDriveMode(DcMotor.RunMode.RUN_TO_POSITION)==false){
//                counter++;
//                sleep(100);
//                debugLogException("init", "waiting on  DcMotor.RunMode.RUN_TO_POSITION) complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }
//        }
//
//    }*/
//
//    //--------------------------------------------------------------------------
//    //
//    // reset_drive_encoders
//    //
//    /**
//     * Reset both drive wheel encoders.
//     */
//    public void reset_drive_encoders ()
//
//    {
//        //
//        // Reset the motor encoders on the drive wheels.
//        //
//        reset_left_drive_encoder();
//        reset_right_drive_encoder();
//
//    } // reset_drive_encoders
//
//    //--------------------------------------------------------------------------
//    //
//    // a_left_encoder_count
//    //
//    /**
//     * Access the left encoder's count.
//     */
//    int a_left_encoder_count ()
//    {
//        int l_return = 0;
//
//        if (v_motor_left_drive != null)
//        {
//            l_return = v_motor_left_drive.getCurrentPosition ();
//        }
//
//        return l_return;
//
//    } // a_left_encoder_count
//
//    int a_leftfront_encoder_count ()
//    {
//        int l_return = 0;
//
//        if (v_motor_leftfront_drive != null)
//        {
//            l_return = v_motor_leftfront_drive.getCurrentPosition ();
//        }
//
//        return l_return;
//
//    }
//
//    int a_rightfront_encoder_count ()
//    {
//        int r_return = 0;
//
//        if (v_motor_rightfront_drive != null)
//        {
//            r_return = v_motor_rightfront_drive.getCurrentPosition ();
//        }
//
//        return r_return;
//
//    }
//
//
//    /**
//     * Access the left drive mode.
//     */
//    DcMotor.RunMode a_left_drive_mode ()
//    {
//
//
//        if (v_motor_left_drive != null)
//        {
//            return v_motor_left_drive.getMode();
//        }
//
//        return DcMotor.RunMode.RUN_TO_POSITION;
//
//    } // a_left_drive_mode
//
//    /**
//     * Access the right drive mode.
//     */
//    DcMotor.RunMode a_right_drive_mode ()
//    {
//
//
//        if (v_motor_right_drive != null)
//        {
//            return v_motor_right_drive.getMode();
//        }
//
//        return DcMotor.RunMode.RUN_TO_POSITION;
//
//    } // a_right_drive_mode
//
//
//    /**
//     * Access the leftfront drive mode.
//     */
//    DcMotor.RunMode a_leftfront_drive_mode ()
//    {
//
//
//        if (v_motor_leftfront_drive != null)
//        {
//            return v_motor_leftfront_drive.getMode();
//        }
//
//        return DcMotor.RunMode.RUN_TO_POSITION;
//
//    } // a_leftfront_drive_mode
//
//    /**
//     * Access the rightfront drive mode.
//     */
//    DcMotor.RunMode a_rightfront_drive_mode ()
//    {
//
//
//        if (v_motor_rightfront_drive != null)
//        {
//            return v_motor_rightfront_drive.getMode();
//        }
//
//        return DcMotor.RunMode.RUN_TO_POSITION;
//
//    } // a_rightfront_drive_mode
//
///*
//    public boolean neopixels_set_rgb(byte red, byte green, byte blue){
//        //I2c didn't work seemed to bring down whole bus on modern Robotics controller will work on it later
//        // so we use digital io for now
//
//        if (v_dim != null){
//            if (v_neopixels_use_i2c && v_neopixels != null ) {
//                v_neopixels.set_rgb(red, green, blue);
//            }else {
//                if (red > 0) {
//                    v_dim.setDigitalChannelState(v_neopixel_red_pin, false);
//                } else {
//                    v_dim.setDigitalChannelState(v_neopixel_red_pin, true);
//                }
//                if (green > 0) {
//                    v_dim.setDigitalChannelState(v_neopixel_green_pin, false);
//                } else {
//                    v_dim.setDigitalChannelState(v_neopixel_green_pin, true);
//                }
//                if (blue > 0) {
//                    v_dim.setDigitalChannelState(v_neopixel_blue_pin, false);
//                } else {
//                    v_dim.setDigitalChannelState(v_neopixel_blue_pin, true);
//                }
//            }
//            return true;
//        }else{
//            return false;
//        }
//    }
//
//    public boolean neopixels_set_brightness(byte brightness){
//        if (v_neopixels != null){
//            v_neopixels.set_brightness(brightness);
//            return true;
//            }else{
//                return false;
//        }
//    }
//
//    boolean v_neopixel_modechange_pin_state = false;
//    public boolean neopixels_set_mode(byte mode){
//
//        if (v_dim != null){
//            if (v_neopixels_use_i2c && v_neopixels != null){
//                 v_neopixels.set_mode(mode);
//            }else{
//                v_neopixel_modechange_pin_state = !v_neopixel_modechange_pin_state;
//                v_dim.setDigitalChannelState(v_neopixel_modechange_pin, v_neopixel_modechange_pin_state);
//            }
//            return true;
//        }else{
//            return false;
//        }
//    }
//*/
//
//    //--------------------------------------------------------------------------
//    //
//    // a_right_encoder_count
//    //
//    /**
//     * Access the right encoder's count.
//     */
//    int a_right_encoder_count ()
//
//    {
//        int l_return = 0;
//
//        if (v_motor_right_drive != null)
//        {
//            l_return = v_motor_right_drive.getCurrentPosition();
//        }
//
//        return l_return;
//
//    } // a_right_encoder_count
//
//    //--------------------------------------------------------------------------
//    //
//    // has_left_drive_encoder_reached
//    //
//    /**
//     * Indicate whether the left drive motor's encoder has reached a value.
//     */
//    boolean has_left_drive_encoder_reached (double p_count)
//
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        if (v_motor_left_drive != null)
//        {
//            //
//            // Has the encoder reached the specified values?
//            //
//            // TODO Implement stall code using these variables.
//            //
//            if (Math.abs (v_motor_left_drive.getCurrentPosition ()) > p_count)
//            {
//                //
//                // Set the status to a positive indication.
//                //
//                l_return = true;
//            }
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_left_drive_encoder_reached
//
//    //--------------------------------------------------------------------------
//    //
//    // has_right_drive_encoder_reached
//    //
//    /**
//     * Indicate whether the right drive motor's encoder has reached a value.
//     */
//    boolean has_right_drive_encoder_reached (double p_count)
//
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        if (v_motor_right_drive != null)
//        {
//            //
//            // Have the encoders reached the specified values?
//            //
//            // TODO Implement stall code using these variables.
//            //
//            if (Math.abs (v_motor_right_drive.getCurrentPosition ()) > p_count)
//            {
//                //
//                // Set the status to a positive indication.
//                //
//                l_return = true;
//            }
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_right_drive_encoder_reached
//
//    //--------------------------------------------------------------------------
//    //
//    // have_drive_encoders_reached
//    //
//    /**
//     * Indicate whether the drive motors' encoders have reached a value.
//     */
//    boolean have_drive_encoders_reached
//    ( double p_left_count
//            , double p_right_count
//    )
//
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Have the encoders reached the specified values?
//        //
//        if (has_left_drive_encoder_reached (p_left_count) &&
//                has_right_drive_encoder_reached (p_right_count))
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // have_encoders_reached
//
//    //--------------------------------------------------------------------------
//    //
//    // drive_using_encoders
//    //
//
//    /**
//     * *
//     * Indicate whether the drive motors' encoders have reached a value.
//     * @param p_left_power  Power 0.0-1.0 for left motor
//     * @param p_right_power Power 0.0-1.0 for left motor
//     * @param p_left_count  Encoder ticks to travel before stopping
//     * @param p_right_count Encoder ticks to travel before stopping
//     * @param useGyro   If true then the gyro will try to maintain a heading by slightly decreasing a tracks power
//     * @param desiredHeading the desired track call sensor_gyro_heading to get current heading
//     * @return true if we have reached the desired distance
//     */
//
//    public boolean drive_using_encoders
//    ( float p_left_power
//            , float p_right_power
//            , double p_left_count
//            , double p_right_count
//            , boolean useGyro
//            , int desiredHeading
//    )
//
//    {
//        //
//        // Assume the encoders have not reached the limit.
//        //
//        boolean l_return = false;
//
//        //
//        // Tell the system that motor encoders will be used.
//        //
//        run_using_encoders ();
//
//        //
//        // Start the drive wheel motors at full power.
//        //
//        drive_set_power (p_left_power, p_right_power);
//
//        //
//        // Have the motor shafts turned the required amount?
//        //
//        // If they haven't, then the op-mode remains in this state (i.e this
//        // block will be executed the next time this method is called).
//        //
//        if (have_drive_encoders_reached (p_left_count, p_right_count))
//        {
//
//            //
//            // Stop the motors.
//            //
//            drive_set_power (0.0f, 0.0f);
//
//            //
//            // Transition to the next state when this method is called
//            // again.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // drive_using_encoders
//
//    //--------------------------------------------------------------------------
//    //
//    // has_left_drive_encoder_reset
//    //
//    /**
//     * Indicate whether the left drive encoder has been completely reset.
//     */
//    boolean has_left_drive_encoder_reset ()
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Has the left encoder reached zero?
//        //
//        if (a_left_encoder_count() == 0)
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_left_drive_encoder_reset
//
//    //--------------------------------------------------------------------------
//    //
//    // has_right_drive_encoder_reset
//    //
//    /**
//     * Indicate whether the left drive encoder has been completely reset.
//     */
//    boolean has_right_drive_encoder_reset ()
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Has the right encoder reached zero?
//        //
//        if (a_right_encoder_count() == 0)
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_right_drive_encoder_reset
//
//
//    //--------------------------------------------------------------------------
//    //
//    // has_left_drive_encoder_reset
//    //
//    /**
//     * Indicate whether the leftfront drive encoder has been completely reset.
//     */
//    boolean has_leftfront_drive_encoder_reset ()
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Has the leftfront encoder reached zero?
//        //
//        if (a_leftfront_encoder_count() == 0)
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_leftfront_drive_encoder_reset
//
//    //--------------------------------------------------------------------------
//    //
//    // has_rightfront_drive_encoder_reset
//    //
//    /**
//     * Indicate whether the right front drive encoder has been completely reset.
//     */
//    boolean has_rightfront_drive_encoder_reset ()
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Has the right encoder reached zero?
//        //
//        if (a_rightfront_encoder_count() == 0)
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // has_rightfront_drive_encoder_reset
//
//
//    //--------------------------------------------------------------------------
//    //
//    // have_drive_encoders_reset
//    //
//    /**
//     * Indicate whether the encoders have been completely reset.
//     */
//    boolean have_drive_encoders_reset ()
//    {
//        //
//        // Assume failure.
//        //
//        boolean l_return = false;
//
//        //
//        // Have the encoders reached zero?
//        //
//        if (has_left_drive_encoder_reset() && has_right_drive_encoder_reset ()
//                && (
//                    isMechDrive == false
//                    ||
//                    (isMechDrive == true && has_leftfront_drive_encoder_reset() && has_rightfront_drive_encoder_reset () )
//                )
//            )
//        {
//            //
//            // Set the status to a positive indication.
//            //
//            l_return = true;
//        }
//
//        //
//        // Return the status.
//        //
//        return l_return;
//
//    } // have_drive_encoders_reset
//
//    private long v_drive_inches_ticks_target_right_slowdown;
//    private long v_drive_inches_ticks_target_left_slowdown;
//    private long v_drive_inches_ticks_target_right_stop;
//    private long v_drive_inches_ticks_target_left_stop;
//    private int v_drive_inches_ticks_target_right;
//    private int v_drive_inches_ticks_target_left;
//    private int v_drive_inches_ticks_target_rightfront;
//    private int v_drive_inches_ticks_target_leftfront;
//    private float v_drive_inches_power;
//    private boolean v_drive_inches_useGyro;
//    private int v_drive_inches_state;
//    private int v_drive_inches_heading;
//    private boolean v_drive_slowdown1_already_set = false;
//
//
//    // inches is positive
//    public void drive_power_override(float drive_power, float drive_power_reverse, float drive_power_slowdown){
//        v_drive_power = drive_power;
//        v_drive_power_reverse=drive_power_reverse;
//        v_drive_power_slowdown=drive_power_slowdown;
//    }
//
//    // inches is positive
//    public void drive_inches(float inches, boolean useGyro){
//        if(inches < 0){
//            //added a reverse power as we are rear heavey need to run slower backward so not to tip over
//            drive_inches(v_drive_power_reverse, inches, useGyro);
//        }else {
//            drive_inches(v_drive_power, inches, useGyro);
//        }
//    }
//
//    public void drive_inches_strife(float inches, boolean useGyro){
//        if(inches < 0){
//            //added a reverse power as we are rear heavy need to run slower backward so not to tip over
//            drive_inches_strife(v_drive_power_reverse, inches, useGyro);
//        }else {
//            drive_inches_strife(v_drive_power, inches, useGyro);
//        }
//    }
//    //left is positive inches, //right is negitive inches
//    private void drive_inches_strife(float power,float inches, boolean useGyro){
//        try {
//
//            if (v_motor_left_drive == null || v_motor_right_drive == null){
//                return;
//            }
//            if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//                return;
//            }
//            if (power < 0 ){
//                set_second_message("Power should be positive");
//                return;
//            }
//            if(isMechDrive == false) {
//                set_second_message("IsMechDrive = false");
//                return;
//            }
//            setupDriveToPosition();
//            v_drive_inches_useGyro = useGyro;
//            v_drive_inches_state = 0;
//            v_drive_inches_power = power;
//            String msg = "drive_inches_strife: p: " + v_drive_inches_power;
//
//            int v_left_position =  a_left_encoder_count();
//            int v_right_position =  a_right_encoder_count();
//
//            v_drive_inches_ticks_target_right = v_right_position -  (int)Math.round(inches * v_drive_inches_strife_ticksPerInch);
//            v_drive_inches_ticks_target_left = v_left_position +  (int)Math.round(inches * v_drive_inches_strife_ticksPerInch);
//
//            v_motor_left_drive.setTargetPosition(v_drive_inches_ticks_target_left);
//            v_motor_right_drive.setTargetPosition(v_drive_inches_ticks_target_right);
//
//            msg = msg + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
//                    + ",l:" + v_left_position + ",r:" + v_right_position;
//
//
//            int v_leftfront_position =  a_leftfront_encoder_count();
//            int v_rightfront_position =  a_rightfront_encoder_count();
//
//            v_drive_inches_ticks_target_rightfront = v_rightfront_position +  (int)Math.round(inches * v_drive_inches_strife_ticksPerInch);
//            v_drive_inches_ticks_target_leftfront = v_leftfront_position -  (int)Math.round(inches * v_drive_inches_strife_ticksPerInch);
//            v_motor_leftfront_drive.setTargetPosition(v_drive_inches_ticks_target_leftfront);
//            v_motor_rightfront_drive.setTargetPosition(v_drive_inches_ticks_target_rightfront);
//            msg = msg + ",tlf:" + v_drive_inches_ticks_target_leftfront + ",trf:" + v_drive_inches_ticks_target_rightfront
//                    + ",lf:" + v_leftfront_position + ",rf:" + v_rightfront_position;
//
//
//            set_second_message(msg);
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException("drive inches", "drive_inches_strife", p_exeception);
//
//
//        }
//    }
//
//    private void drive_inches(float power,float inches, boolean useGyro){
//        try {
//
//            if (v_motor_left_drive == null || v_motor_right_drive == null){
//                return;
//            }
//            if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//                return;
//            }
//            if (power < 0 ){
//                set_second_message("Power should be positive");
//                return;
//            }
//            setupDriveToPosition();
//            v_drive_inches_useGyro = useGyro;
//            v_drive_inches_state = 0;
//            v_drive_inches_power = power;
//            String msg = "drive_inches: p: " + v_drive_inches_power;
//
//            int v_left_position =  a_left_encoder_count();
//            int v_right_position =  a_right_encoder_count();
//
//            v_drive_inches_ticks_target_right = v_right_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);
//            v_drive_inches_ticks_target_left = v_left_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);
//            v_motor_left_drive.setTargetPosition(v_drive_inches_ticks_target_left);
//            v_motor_right_drive.setTargetPosition(v_drive_inches_ticks_target_right);
//
//            msg = msg + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
//                    + ",l:" + v_left_position + ",r:" + v_right_position;
//
//            if(isMechDrive){
//                int v_leftfront_position =  a_leftfront_encoder_count();
//                int v_rightfront_position =  a_rightfront_encoder_count();
//
//                v_drive_inches_ticks_target_rightfront = v_rightfront_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);
//                v_drive_inches_ticks_target_leftfront = v_leftfront_position +  (int)Math.round(inches * v_drive_inches_ticksPerInch);
//                v_motor_leftfront_drive.setTargetPosition(v_drive_inches_ticks_target_leftfront);
//                v_motor_rightfront_drive.setTargetPosition(v_drive_inches_ticks_target_rightfront);
//                msg = msg + ",tlf:" + v_drive_inches_ticks_target_leftfront + ",trf:" + v_drive_inches_ticks_target_rightfront
//                        + ",lf:" + v_leftfront_position + ",rf:" + v_rightfront_position;
//            }
//
//            set_second_message(msg);
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException("drive inches", "drive_inches", p_exeception);
//
//
//        }
//    }
//
//
//    public boolean drive_inches_complete(){
//        if (v_motor_left_drive == null || v_motor_right_drive == null){
//            return true;
//        }
//        if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//            return true;
//        }
//        int v_drive_inches_ticks_left = v_motor_left_drive.getCurrentPosition();
//        int v_drive_inches_ticks_right = v_motor_right_drive.getCurrentPosition();
//
//        int v_drive_inches_ticks_leftfront = 0;
//        int v_drive_inches_ticks_rightfront = 0;
//        if(isMechDrive) {
//            v_drive_inches_ticks_leftfront = v_motor_leftfront_drive.getCurrentPosition();
//            v_drive_inches_ticks_rightfront = v_motor_rightfront_drive.getCurrentPosition();
//        }
//
//        switch(v_drive_inches_state){
//
//            case 0:
//                if (v_drive_inches_useGyro) {
//                    v_drive_inches_heading = sensor_gyro_mr_get_heading();
//                }
//
//                drive_set_power(v_drive_inches_power, v_drive_inches_power);
//                String msg = "drive_inches_complete: set the drive power "
//                        + "gyro th:" + v_drive_inches_heading
//                        + " p: " + v_drive_inches_power
//                        + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
//                        + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right;
//                if(isMechDrive){
//                    msg = msg + ",tlf:" + v_drive_inches_ticks_target_leftfront + ",trf:" + v_drive_inches_ticks_target_rightfront
//                            + ",lf:" + v_drive_inches_ticks_leftfront + ",rf:" + v_drive_inches_ticks_rightfront;
//                }
//                set_second_message(msg);
//                v_drive_inches_state++;
//                break;
//            case 1:
//                if(v_motor_left_drive.isBusy() == false && v_motor_right_drive.isBusy() == false
//                        && (isMechDrive == false
//                            || (isMechDrive ==true && v_motor_leftfront_drive.isBusy() == false && v_motor_rightfront_drive.isBusy() == false)
//                            )
//                        ) {
//                    return true;
//                }else if(v_drive_use_slowdown == true ){
//                    int v_drive_left_difference =  v_drive_inches_ticks_target_left - v_drive_inches_ticks_left;
//                    if(v_drive_left_difference < 0){
//                        v_drive_left_difference = 0 - v_drive_left_difference;
//                    }
//                    int v_drive_right_difference =  v_drive_inches_ticks_target_right - v_drive_inches_ticks_right;
//                    if(v_drive_right_difference < 0){
//                        v_drive_right_difference = 0 - v_drive_right_difference;
//                    }
//
//                    if(v_drive_left_difference <= v_drive_inches_slowdown_ticks || v_drive_right_difference <= v_drive_inches_slowdown_ticks){
//                        drive_set_power(v_drive_power_slowdown, v_drive_power_slowdown);
//                    }
//                }
//                /*else if(v_drive_inches_useGyro){
//                    //the logic here is to try to hold a gyro heading by slowing a track down a touch v_drive_inches_power_gyro_correction
//                    //the issue is our gyro is slow to refresh so may need to only do this a couple of loops then turn off
//                    //not sure yet still testing 11/19/2015 Two days to first competition wow this is tight deadline
//                    int headingDifference;
//                    int currentHeading = sensor_gyro_get_heading();
//                    if ((v_drive_inches_heading - currentHeading) > 180 ) {
//                        currentHeading = 360 + currentHeading;
//                    }else if ((v_drive_inches_heading - currentHeading) < -180 ) {
//                        currentHeading = currentHeading - 360;
//                    }
//                    headingDifference = Math.abs(currentHeading-v_drive_inches_heading);
//                    //hard limit of no more then 3 times the correction
//                    if (headingDifference > v_drive_inches_power_gyro_correction_max_times){
//                        headingDifference = v_drive_inches_power_gyro_correction_max_times;
//                    }
//                    float powerCorrectAmount = v_drive_inches_power_gyro_correction * headingDifference;
//                    if (v_drive_inches_power < 0.0d){
//                        powerCorrectAmount = 0 - powerCorrectAmount;
//                    }
//                    float v_left_power_adjust = v_drive_inches_power;
//                    float v_right_power_adjust = v_drive_inches_power;
//                    if ((v_drive_inches_heading > currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading < currentHeading && v_drive_inches_power < 0) ){
//                        v_right_power_adjust = v_drive_inches_power - powerCorrectAmount;
//                    }else if ((v_drive_inches_heading < currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading > currentHeading && v_drive_inches_power < 0)){
//                        v_left_power_adjust = v_drive_inches_power - powerCorrectAmount;
//                    }
//                    set_drive_power(v_left_power_adjust, v_right_power_adjust );
//
//                }*/
//                if(v_loop_ticks_slow) {
//                    String slowmsg = "drive_inches_complete: "
//                            + "gyro th:" + v_drive_inches_heading
//                            +  " p: " + v_drive_inches_power
//                            + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
//                            + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right
//                            + ",lp:" + v_motor_left_drive.getPower() + ",rp:" + v_motor_right_drive.getPower();
//                    if(isMechDrive){
//                        slowmsg = slowmsg + ",tlf:" + v_drive_inches_ticks_target_leftfront + ",trf:" + v_drive_inches_ticks_target_rightfront
//                                + ",lf:" + v_drive_inches_ticks_leftfront + ",rf:" + v_drive_inches_ticks_rightfront;
//                    }
//                    set_second_message(slowmsg);
//                }
//                break;
//            default:
//                return true;
//        }
//
//        return false;
//    }
//
//    public boolean drive_inches_stop(){
//        if (v_motor_left_drive == null || v_motor_right_drive == null){
//            return true;
//        }
//        if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//            return true;
//        }
//        drive_set_power(0, 0);
//        int v_drive_inches_ticks_left = v_motor_left_drive.getCurrentPosition();
//        int v_drive_inches_ticks_right = v_motor_right_drive.getCurrentPosition();
//        v_motor_left_drive.setTargetPosition(v_drive_inches_ticks_left);
//        v_motor_right_drive.setTargetPosition(v_drive_inches_ticks_right);
//        String msg = "drive_inches_stop: "
//                        + "gyro th:" + v_drive_inches_heading
//                        +  " p: " + v_drive_inches_power
//                        + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
//                        + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right;
//
//        if(isMechDrive){
//            int v_drive_inches_ticks_leftfront = v_motor_leftfront_drive.getCurrentPosition();
//            int v_drive_inches_ticks_rightfront = v_motor_rightfront_drive.getCurrentPosition();
//            v_motor_leftfront_drive.setTargetPosition(v_drive_inches_ticks_leftfront);
//            v_motor_rightfront_drive.setTargetPosition(v_drive_inches_ticks_rightfront);
//            msg = msg + ",tlf:" + v_drive_inches_ticks_target_leftfront + ",trf:" + v_drive_inches_ticks_target_rightfront
//                    + ",lf:" + v_drive_inches_ticks_leftfront + ",rf:" + v_drive_inches_ticks_rightfront;
//        }
//        msg = msg  + ",lp:" + v_motor_left_drive.getPower() + ",rp:" + v_motor_right_drive.getPower();
//        set_second_message(msg);
//        return true;
//    }
//
//    /**
//     * Inits the led 7 segment counter to start a count down in seconds
//     * @param seconds
//     * @return
//     */
//    public boolean led7seg_timer_init(int seconds){
//        if (v_ledseg != null){
//            v_ledseg.writeSeconds(seconds);
//            return true;
//        }
//        return true;
//    }
//
//    private boolean isFirstButtonPress = true;
//
//    public boolean manualModeButtonPress(){
//        if (isFirstButtonPress){
//            isFirstButtonPress = false;
//            v_ledseg.startTimer(120);
//        }
//        return true;
//    }
//
//    /**
//     * Inits the led 7 segment counter to start a count down in seconds
//     * @return
//     */
//
//    public boolean led7seg_test(){
//        if (v_ledseg != null){
//
//            v_ledseg.writetest();
//            return true;
//        }
//        return false;
//    }
//
//    /**
//     * starts led 7 segment counter to to count down in seconds
//     * @param seconds
//     * @return
//     */
//    int v_led7seg_timer_seconds = 0;
//    boolean v_led7seg_timer_running = false;
//    public boolean led7seg_timer_start(int seconds){
//        if (v_ledseg != null){
//
//            v_ledseg.startTimer(seconds);
//            return true;
//        }
//        return false;
//    }
//
//    public boolean led7seg_timer_complete(){
//        if (v_ledseg != null) {
//            return v_ledseg.is_timer_complete();
//        }else{
//            return true;
//        }
//    }
//    public boolean led7seg_is_enabled(){
//        if (v_ledseg != null){
//            return v_ledseg.isEnabled();
//        }
//        return false;
//    }
//    public boolean led7seg_enabled(boolean enabled){
//        if (v_ledseg != null){
//            return v_ledseg.enabled(enabled);
//        }
//        return false;
//    }
//
//    private int v_drive_ToPosition_ticks_target_right;
//    private int v_drive_ToPosition_ticks_target_left;
//    private int v_drive_ToPosition_ticks_target_rightfront;
//    private int v_drive_ToPosition_ticks_target_leftfront;
//    public void drive_ToPosition(int leftTicks, int rightTicks, float leftPower, float rightPower){
//        setupDriveToPosition();
//        if( v_motor_left_drive == null || v_motor_right_drive == null){
//            return;
//        }
//        if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//            return;
//        }
//        int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
//        int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
//        v_drive_ToPosition_ticks_target_left = v_motor_left_position + leftTicks;
//        v_drive_ToPosition_ticks_target_right = v_motor_right_position + rightTicks;
//        v_motor_right_drive.setTargetPosition(v_drive_ToPosition_ticks_target_right);
//        v_motor_left_drive.setTargetPosition(v_drive_ToPosition_ticks_target_left);
//        String msg = "drive_ToPosition: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position;
//        if(isMechDrive){
//            int v_motor_rightfront_position = v_motor_rightfront_drive.getCurrentPosition();
//            int v_motor_leftfront_position = v_motor_leftfront_drive.getCurrentPosition();
//            v_drive_ToPosition_ticks_target_leftfront = v_motor_leftfront_position + leftTicks;
//            v_drive_ToPosition_ticks_target_rightfront = v_motor_rightfront_position + rightTicks;
//            v_motor_rightfront_drive.setTargetPosition(v_drive_ToPosition_ticks_target_rightfront);
//            v_motor_leftfront_drive.setTargetPosition(v_drive_ToPosition_ticks_target_leftfront);
//            msg = msg + "lft:" + v_drive_ToPosition_ticks_target_leftfront + " rft:" + v_drive_ToPosition_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_position + ", lfe:" + v_motor_leftfront_position;
//        }
//
//        drive_set_power(leftPower,rightPower);
//
//        set_second_message(msg );
//    }
//
//    public boolean drive_ToPosition_Complete(){
//        if( v_motor_left_drive == null || v_motor_right_drive == null){
//            return true;
//        }
//        if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//            return true;
//        }
//        if (v_motor_left_drive.isBusy() == false
//                &&  v_motor_right_drive.isBusy() == false
//                && (
//                        isMechDrive == false
//                    ||
//                        (isMechDrive == true && v_motor_leftfront_drive.isBusy() == false && v_motor_rightfront_drive.isBusy() == false )
//                    )
//            ) {
//            drive_set_power(0.0f, 0.0f);
//
//            if(v_debug) {
//                int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
//                int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
//                String msg = "drive_ToPosition_Complete: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position;
//                if (isMechDrive) {
//                    int v_motor_rightfront_position = v_motor_rightfront_drive.getCurrentPosition();
//                    int v_motor_leftfront_position = v_motor_leftfront_drive.getCurrentPosition();
//                    msg = msg + "lft:" + v_drive_ToPosition_ticks_target_leftfront + " rft:" + v_drive_ToPosition_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_position + ", lfe:" + v_motor_leftfront_position;
//                }
//                set_second_message(msg);
//            }
//            return true;
//        }else{
//            if(is_slow_tick()){
//                if(v_debug) {
//                    int v_motor_right_position = v_motor_right_drive.getCurrentPosition();
//                    int v_motor_left_position = v_motor_left_drive.getCurrentPosition();
//                    String msg = "drive_ToPosition_Complete: lt:" + v_drive_ToPosition_ticks_target_left + " rt:" + v_drive_ToPosition_ticks_target_right + ", re:" + v_motor_right_position + ", le:" + v_motor_left_position;
//                    if (isMechDrive) {
//                        int v_motor_rightfront_position = v_motor_rightfront_drive.getCurrentPosition();
//                        int v_motor_leftfront_position = v_motor_leftfront_drive.getCurrentPosition();
//                        msg = msg + "lft:" + v_drive_ToPosition_ticks_target_leftfront + " rft:" + v_drive_ToPosition_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_position + ", lfe:" + v_motor_leftfront_position;
//                    }
//                    set_second_message(msg);
//                }
//            }
//            return false;
//        }
//
//    }
//
//    private int v_turn_degrees_ticks_target_right;
//    private int v_turn_degrees_ticks_target_left;
//    private int v_turn_degrees_ticks_target_rightfront;
//    private int v_turn_degrees_ticks_target_leftfront;
//    private boolean v_turn_degrees_usingGyro;
//    private boolean v_turn_degrees_iscwturn;
//    private boolean v_turn_degrees_isSlowTurn;
//    private int v_turn_degrees_state;
//    /**
//     *
//     * @param degrees the amount in degrees you want to turn postive number is to the right negitive to the left
//     * @param turnSlow make a slowTurn
//     * @param useGyro use the Gyro to turn if false then ticks of the encoder will be used
//     *
//     */
//
//
//
//    public void turn_degrees(int degrees, boolean turnSlow, boolean useGyro){
//        //Do nothing is turn is zero
//        if(degrees == 0 || v_motor_left_drive == null || v_motor_right_drive == null
//                || (isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null) )
//           ){
//            return;
//        }
//
//        setupDriveToPosition();
//        v_turn_degrees_state = 0;
//        v_turn_degrees_usingGyro = useGyro;
//        v_turn_degrees_isSlowTurn = turnSlow;
//        if (degrees > 0) {
//            //greater then 0 turn cw or to the right
//            v_turn_degrees_iscwturn = true;
//        } else {
//            v_turn_degrees_iscwturn = false;
//        }
//
//        //Turn using just encoder ticks
//        int ticks = Math.round(Math.abs(degrees) * v_drive_turn_ticks_per_degree);
//        if (v_turn_degrees_iscwturn) {
//            v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() + ticks;
//            v_turn_degrees_ticks_target_right = v_motor_right_drive.getCurrentPosition() - ticks;
//        } else {
//            v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() - ticks;
//            v_turn_degrees_ticks_target_right =  v_motor_right_drive.getCurrentPosition() + ticks;
//        }
//        v_motor_right_drive.setTargetPosition(v_turn_degrees_ticks_target_right);
//        v_motor_left_drive.setTargetPosition(v_turn_degrees_ticks_target_left);
//        String msg = "turn_degrees: ticks:" + ticks + ", lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition();
//        if(isMechDrive) {
//            if (v_turn_degrees_iscwturn) {
//                v_turn_degrees_ticks_target_leftfront = v_motor_leftfront_drive.getCurrentPosition() + ticks;
//                v_turn_degrees_ticks_target_rightfront = v_motor_rightfront_drive.getCurrentPosition() - ticks;
//            } else {
//                v_turn_degrees_ticks_target_leftfront = v_motor_leftfront_drive.getCurrentPosition() - ticks;
//                v_turn_degrees_ticks_target_rightfront = v_motor_rightfront_drive.getCurrentPosition() + ticks;
//            }
//            v_motor_rightfront_drive.setTargetPosition(v_turn_degrees_ticks_target_rightfront);
//            v_motor_leftfront_drive.setTargetPosition(v_turn_degrees_ticks_target_leftfront);
//            msg = msg +  ", lft:" + v_turn_degrees_ticks_target_leftfront + " rft:" + v_turn_degrees_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_drive.getCurrentPosition() + ", lfe:" + v_motor_leftfront_drive.getCurrentPosition();
//        }
//        set_second_message(msg );
//    }
//
//    public void turn_power_override(float turn_motorspeed, float turn_motorspeed_slow){
//        v_turn_motorspeed = turn_motorspeed;
//        v_turn_motorspeed_slow = turn_motorspeed_slow;
//    }
//
//    /**
//     * Used to tell if the turn is complete turn_degrees must be called first
//     *
//     * @return true if the turn is complete false if not
//     */
//
//    public boolean turn_complete(){
//
//        try{
//
//            if(v_motor_left_drive == null || v_motor_right_drive == null){
//                set_second_message("turn_complete: no motors");
//                v_turn_degrees_state = -1;
//                return true;
//            }
//            if(isMechDrive && (v_motor_leftfront_drive == null || v_motor_rightfront_drive == null)){
//                set_second_message("turn_complete: no front motors");
//                v_turn_degrees_state = -1;
//                return true;
//            }
//
//            switch(v_turn_degrees_state){
//                case 0:
//                    if (v_turn_degrees_iscwturn) {
//                        if (v_turn_degrees_isSlowTurn) {
//                            //have an issue where motors not turning on at same time so need to call directly
//                            //turning right so turn on left motor first
//                            v_motor_left_drive.setPower(v_turn_motorspeed_slow);
//                            v_motor_right_drive.setPower(0 - v_turn_motorspeed_slow);
//                            if(isMechDrive){
//                                v_motor_leftfront_drive.setPower(v_turn_motorspeed_slow);
//                                v_motor_rightfront_drive.setPower(0 - v_turn_motorspeed_slow);
//                            }
//                        }else {
//                            //turning right so turn on left motor first
//                            v_motor_left_drive.setPower(v_turn_motorspeed);
//                            v_motor_right_drive.setPower(0 - v_turn_motorspeed);
//                            if(isMechDrive){
//                                v_motor_leftfront_drive.setPower(v_turn_motorspeed);
//                                v_motor_rightfront_drive.setPower(0 - v_turn_motorspeed);
//                            }
//                        }
//                    } else {
//                        if (v_turn_degrees_isSlowTurn) {
//                            //turning left so turn on right motor first
//                            v_motor_right_drive.setPower(v_turn_motorspeed_slow);
//                            v_motor_left_drive.setPower(0 - v_turn_motorspeed_slow);
//                            if(isMechDrive){
//                                v_motor_rightfront_drive.setPower( v_turn_motorspeed_slow);
//                                v_motor_leftfront_drive.setPower(0 - v_turn_motorspeed_slow);
//                            }
//                        }else {
//                            //turning left so turn on right motor first
//                            v_motor_right_drive.setPower(v_turn_motorspeed);
//                            v_motor_left_drive.setPower(0 - v_turn_motorspeed);
//                            if(isMechDrive){
//                                v_motor_rightfront_drive.setPower( v_turn_motorspeed);
//                                v_motor_leftfront_drive.setPower(0 - v_turn_motorspeed);
//                            }
//                        }
//                    }
//                    v_turn_degrees_state++;
//                    break;
//                case 1:
//                    if (v_motor_left_drive.isBusy() == false &&  v_motor_right_drive.isBusy() == false
//                            && (isMechDrive == false
//                                ||
//                                (isMechDrive == true && v_motor_leftfront_drive.isBusy() == false &&  v_motor_rightfront_drive.isBusy() == false)
//                                )
//                        ) {
//                        drive_set_power(0.0f, 0.0f);
//                        String msg = "turn_complete: encoders reached value lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition();
//                        if(isMechDrive){
//                            msg = msg + " lft:" + v_turn_degrees_ticks_target_leftfront + " rft:" + v_turn_degrees_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_drive.getCurrentPosition() + ", lfe:" + v_motor_leftfront_drive.getCurrentPosition();
//                        }
//                        set_second_message(msg);
//                        v_turn_degrees_state++;
//                        return true;
//                    }else {
//                        if (v_debug && is_slow_tick()){
//                            String msg = "turn_complete: Waiting on encoders lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() + ", rp:" + v_motor_right_drive.getPower() + ", lp:" + v_motor_left_drive.getPower();
//                            if(isMechDrive){
//                                msg = msg + "turn_complete: Waiting on encoders lft:" + v_turn_degrees_ticks_target_leftfront + " rft:" + v_turn_degrees_ticks_target_rightfront + ", rfe:" + v_motor_rightfront_drive.getCurrentPosition() + ", lfe:" + v_motor_leftfront_drive.getCurrentPosition() + ", rfp:" + v_motor_rightfront_drive.getPower() + ", lfp:" + v_motor_leftfront_drive.getPower();
//                            }
//                            set_second_message(msg );
//                        }
//                    }
//
//                    break;
//                default:
//                    return true;
//            }
//
//            return false;
//        } catch (Exception p_exeception)
//        {
//            debugLogException("turn_complete:", "error " + p_exeception.getMessage() , p_exeception);
//            return false;
//        }
//    }
//
//
//
//
////    //lifter On
////    boolean lifter_On ()
////    {
////        m_winch_power(v_motor_winch_Speed);
////        return true;
////
////    } // rpaarm_moveUp
////    //--------------------------------------------------------------------------
////    //
////    // m_winch_power
////    //
////    /**
////     * Access the winch motor's power level.
////     */
////    void m_lifter_power (double p_level)
////    {
////        if (v_motor_winch != null)
////        {
////            if(p_level > 0){
////                //move the rpa base arm down at same time so not to fight the winch with the servo
////                rpabase_moveDown(true);
////                v_motor_winch.setPower(p_level);
////            }
////            else{
////                v_motor_winch.setPower(0);
////            }
////        }
////
////    } // m_winch_power
////
////    private boolean v_motors_blockgrabbers_on = false;
////    public void blockgrabbers_start ()
////    {
////        try {
////            if (v_motor_bgleft != null && v_motor_bgright != null ) {
////                v_motor_bgleft.setPower(1.0f);
////                v_motor_bgright.setPower(1.0f);
////                v_motors_blockgrabbers_on = true;
////            }
////            set_third_message("blockgrabbers_start");
////        }catch (Exception p_exeception)
////        {
////            debugLogException("blockgrabbers_start", "error", p_exeception);
////        }
////    }
////
////    public void blockgrabbers_stop ()
////    {
////        try {
////            if (v_motor_bgleft != null && v_motor_bgright != null ) {
////                v_motor_bgleft.setPower(0.0f);
////                v_motor_bgright.setPower(0.0f);
////                v_motors_blockgrabbers_on = false;
////            }
////            set_third_message("blockgrabbers_start");
////        }catch (Exception p_exeception)
////        {
////            debugLogException("blockgrabbers_start", "error", p_exeception);
////        }
////    }
////
////    public void blockgrabbers_toggle()
////    {
////        try {
////            if (v_motors_blockgrabbers_on) {
////                blockgrabbers_stop();
////            }else{
////                blockgrabbers_start();
////            }
////        }catch (Exception p_exeception)
////        {
////            debugLogException("blockgrabbers_toggle", "error", p_exeception);
////        }
////    }
//
//    public void blockgrabber_toggle ()
//    {
//        try {
//            if (v_servo_blockgrabber_is_extended == true){
//                blockgrabber_close();
//            }else{
//                blockgrabber_open();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockgrabber_toggle", "error", p_exeception);
//        }
//    }
//    public void blockgrabber_open ()
//    {
//        try {
//            if (v_servo_blockgrabber != null) {
//                v_servo_blockgrabber.setPosition(v_servo_blockgrabber_MinPosition);
//                v_servo_blockgrabber_is_extended = true;
//            }
//            set_third_message("blockgrabber_open " + v_servo_blockgrabber_MinPosition);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockgrabber_extend", "error", p_exeception);
//        }
//    }
//
//    public void blockgrabber_close ()
//    {
//        try {
//            if (v_servo_blockgrabber != null) {
//                v_servo_blockgrabber_is_extended = false;
//                v_servo_blockgrabber.setPosition(v_servo_blockgrabber_MaxPosition);
//            }
//            set_third_message("blockgrabber_close " + v_servo_blockgrabber_MaxPosition);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockgrabber_retract", "error", p_exeception);
//        }
//    }
//    ///Pass in 1 2 3
//    public void blockslide_position( int position)
//    {
//        try {
//            if (v_servo_blockslide != null) {
//                v_servo_blockslide_position = position;
//                switch(position){
//                    case 0:
//                        v_servo_blockslide.setPosition(v_servo_blockslide_MinPosition);
//                        break;
//                    case 1:
//                        v_servo_blockslide.setPosition(v_servo_blockslide_MiddlePosition);
//                        break;
//                    case 2:
//                        v_servo_blockslide.setPosition(v_servo_blockslide_MaxPosition);
//                        break;
//
//                }
//                set_first_message("block_slide " + v_servo_blockslide_position);
//                v_servo_blockslide_is_extended = true;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockslide_left", "error", p_exeception);
//        }
//    }
//
//    public void blockslide_left ()
//    {
//        try {
//            if (v_servo_blockslide != null) {
//                if(v_servo_blockslide_position > 0){
//                    v_servo_blockslide_position--;
//                }
//                blockslide_position(v_servo_blockslide_position);
//                set_first_message("blockslide_left " + v_servo_blockslide_position);
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockslide_left", "error", p_exeception);
//        }
//    }
//
//    public void blockslide_right ()
//    {
//        try {
//            if (v_servo_blockslide != null) {
//                if(v_servo_blockslide_position < 2){
//                    v_servo_blockslide_position++;
//                    blockslide_position(v_servo_blockslide_position);
//                }
//
//                set_first_message("blockslide_right " + v_servo_blockslide_position);
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockslide_right", "error", p_exeception);
//        }
//    }
//
//    public void blockslide_stop ()
//    {
//        try {
//            if (v_servo_blockslide != null) {
//                v_servo_blockslide_is_extended = false;
//                v_servo_blockslide.setPosition(v_servo_blockslide_MiddlePosition);
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("blockslide_stop", "error", p_exeception);
//        }
//    }
//
//
//
//
//    public void jewel_toggle ()
//    {
//        try {
//            if (v_servo_jewel_is_extending == true){
//                jewel_lower();
//            }else{
//                jewel_raise();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("jewel_toggle", "error", p_exeception);
//        }
//    }
//    public void jewel_raise ()
//    {
//        try {
//            if (v_servo_jewel != null) {
//                v_servo_jewel.setPosition(v_servo_jewel_MaxPosition);
//                set_second_message("jewel extend " + v_servo_jewel_MaxPosition);
//                v_servo_jewel_is_extending = true;
//                v_servo_jewel_is_retracting = false;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("jewel_extend", "error", p_exeception);
//        }
//    }
//
//    public void jewel_lower ()
//    {
//        try {
//            if (v_servo_jewel != null) {
//                v_servo_jewel_position = v_servo_jewel_MinPosition + v_servo_jewel_retract_ease;
//                v_servo_jewel.setPosition(v_servo_jewel_position);
//                set_second_message("jewel retract " + (v_servo_jewel_position));
//
//                v_servo_jewel_is_extending = false;
//                v_servo_jewel_is_retracting = true;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("jewel_retract", "error", p_exeception);
//        }
//    }
//
//    //Warning there is no way to read the servos current position ie there is no feed back
//    // to the electronics So calling this function a hundred times while a button is down will make it hard to control
//    public void jewel_step(double stepAmount)
//    {
//        try {
//            if (v_servo_jewel != null) {
//                double position= v_servo_jewel.getPosition() + stepAmount;
//                if(position >= v_servo_jewel_MinPosition && position <= v_servo_jewel_MaxPosition) {
//                    v_servo_jewel.setPosition(position);
//                    set_second_message("jewel step " + position);
//                }
//
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("jewel_step", "error", p_exeception);
//        }
//    }
//    public void jewel_setposition (double position)
//    {
//        try {
//            if (v_servo_jewel != null) {
//                v_servo_jewel.setPosition(position);
//                set_second_message("jewel setposition " + position);
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("jewel_setposition", "error", p_exeception);
//        }
//    }
//
/////////wrist
//
//
//
//    public void wrist_toggle ()
//    {
//        try {
//            if (v_servo_wrist_is_extending == true){
//                wrist_retract();
//            }else{
//                wrist_extend();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("wrist_toggle", "error", p_exeception);
//        }
//    }
//    public void wrist_extend ()
//    {
//        try {
//            if (v_servo_wrist != null) {
//                v_servo_wrist.setPosition(v_servo_wrist_MaxPosition);
//                v_servo_wrist_is_extending = true;
//            }
//            set_third_message(config_servo_wrist + " " + v_servo_wrist.getPosition());
//        }catch (Exception p_exeception)
//        {
//            debugLogException("wrist_extend", "error", p_exeception);
//        }
//    }
//
//    public void wrist_retract ()
//    {
//        try {
//            if (v_servo_wrist != null) {
//                //v_servo_wrist.setPosition(v_servo_wrist_MinPosition-.2);
//                //wait(300);
//                v_servo_wrist.setPosition(v_servo_wrist_MinPosition);
//                v_servo_wrist_is_extending = false;
//            }
//            set_third_message(config_servo_wrist + " " + v_servo_wrist.getPosition());
//        }catch (Exception p_exeception)
//        {
//            debugLogException("wrist_retract", "error", p_exeception);
//        }
//    }
//
//    //Warning there is no way to read the servos current position ie there is no feed back
//    // to the electronics So calling this function a hundred times while a button is down will make it hard to control
//    public void wrist_step(double stepAmount)
//    {
//        try {
//            if (v_servo_wrist != null) {
//                v_servo_wrist.setPosition(v_servo_wrist.getPosition() + stepAmount);
//
//            }
//            set_third_message(config_servo_wrist + " " + v_servo_wrist.getPosition());
//        }catch (Exception p_exeception)
//        {
//            debugLogException("wrist_step", "error", p_exeception);
//        }
//    }
//    public void wrist_setposition (double position)
//    {
//        try {
//            if (v_servo_wrist != null) {
//                v_servo_wrist.setPosition(position);
//            }
//            set_third_message(config_servo_wrist + " " + v_servo_wrist.getPosition());
//        }catch (Exception p_exeception)
//        {
//            debugLogException("wrist_setposition", "error", p_exeception);
//        }
//    }
//
//
//
//
//
//    public void hand_toggle ()
//    {
//        try {
//            if (v_servo_hand_is_extending == true){
//                hand_close();
//            }else{
//                hand_open();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("hand_toggle", "error", p_exeception);
//        }
//    }
//    public void hand_open ()
//    {
//        try {
//            if (v_servo_hand != null) {
//                v_servo_hand.setPosition(v_servo_hand_MaxPosition);
//                v_servo_hand_is_extending = true;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("hand_open", "error", p_exeception);
//        }
//    }
//
//    public void hand_close ()
//    {
//        try {
//            if (v_servo_hand != null) {
//                v_servo_hand.setPosition(v_servo_hand_MinPosition);
//                v_servo_hand_is_extending = false;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("hand_retract", "error", p_exeception);
//        }
//    }
//
//
//    public void shoulder_toggle ()
//    {
//        try {
//            if (v_servo_shoulder_is_extended == true){
//                shoulder_retract();
//            }else{
//                shoulder_extend();
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("shoulder_toggle", "error", p_exeception);
//        }
//    }
//    public void shoulder_extend ()
//    {
//        try {
//            if (v_servo_shoulder != null) {
//                v_servo_shoulder.setPosition(v_servo_shoulder_MaxPosition);
//                set_second_message("shoulder_extend " + v_servo_shoulder_MaxPosition);
//                v_servo_shoulder_is_extended = true;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("shoulder_extend", "error", p_exeception);
//        }
//    }
//
//    public void shoulder_retract ()
//    {
//        try {
//            if (v_servo_shoulder != null) {
//                v_servo_shoulder_is_extended = false;
//                v_servo_shoulder.setPosition(v_servo_shoulder_MinPosition);
//                set_second_message("shoulder_retract " + v_servo_shoulder_MinPosition);
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("shoulder_retract", "error", p_exeception);
//        }
//    }
//
//    //Warning there is no way to read the servos current position ie there is no feed back
//    // to the electronics So calling this function a hundred times while a button is down will make it hard to control
//    public void shoulder_step(double stepAmount)
//    {
//        try {
//            double v_servo_shoulderPosition = 0;
//            if (v_servo_shoulder != null) {
//
//                v_servo_shoulderPosition = v_servo_shoulder.getPosition() + stepAmount;
//                if (v_servo_shoulderPosition >= v_servo_shoulder_MinPosition && v_servo_shoulderPosition <= v_servo_shoulder_MaxPosition ) {
//                    v_servo_shoulder.setPosition(v_servo_shoulderPosition);
//                }
//            }
//            set_second_message("shoulder_step " + v_servo_shoulderPosition);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("shoulder_step", "error", p_exeception);
//        }
//    }
//    public void shoulder_setposition (double position)
//    {
//        try {
//            if (v_servo_shoulder != null) {
//
//                v_servo_shoulder.setPosition(position);
//            }
//            set_second_message("shoulder_setposition " + position);
//        }catch (Exception p_exeception)
//        {
//            debugLogException("shoulder_setposition", "error", p_exeception);
//        }
//    }
//
//    public void debugOff(){
//        v_debug = false;
//        set_second_message("Debug is Off no Telemetry Enabled");
//        update_telemetry();
//        opMode.updateTelemetry(opMode.telemetry);
//    }
//
//    public boolean debugMode(){
//        return v_debug;
//    }
//
//    public void debugOn(){
//        v_debug = true;
//        set_second_message("Debug is On");
//        update_telemetry();
//        opMode.updateTelemetry(opMode.telemetry);
//    }
//
//    //--------------------------------------------------------------------------
//    //
//    // arm_wrist_moveRight
//    //
//    /**
//     * move the arm_wrist servo to the Right.
//     */
//  /*  double arm_wrist_moveRight (boolean fast)
//    {
//        double l_temptarget;
//        if (fast) {
//            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta_Fast;
//        }else{
//            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta;
//        }
//        return m_arm_wrist_position(l_temptarget);
//    } // arm_wrist_moveRight
//
//    //--------------------------------------------------------------------------
//    //
//    // m_arm_wrist_position
//    //
//    /**
//     * Mutate the arm wrist position.
//     */
//  /*  double m_arm_wrist_position (double p_position)
//    {
//        //
//        // Ensure the specific value is legal.
//        //
//        l_arm_wrist_position = Range.clip
//                ( p_position
//                        , ArmWristServo_MinPosition
//                        , ArmWristServo_MaxPosition
//                );
//        try {
//            if (v_servo_arm_wrist != null) {
//                v_servo_arm_wrist.setPosition(l_arm_wrist_position);
//                return l_arm_wrist_position;
//            } else {
//                return ServoErrorResultPosition;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("arm_wrist", "missing", p_exeception);
//            return ServoErrorResultPosition;
//        }
//
//
//    } // m_arm_jewel_position
//
//*/
//
//    /**
//     * Mutate the flip right position.
//     */
//    /*double m_flip_right_position (double p_position)
//    {
//        //
//        // Ensure the specific value is legal.
//        //
//        l_flip_right_position = Range.clip
//                ( p_position
//                        , FlipRightServo_MinPosition
//                        , FlipRightServo_MaxPosition
//                );
//        try {
//            if (v_servo_flip_right != null) {
//                v_servo_flip_right.setPosition(l_flip_right_position);
//                return l_flip_right_position;
//            } else {
//                return ServoErrorResultPosition;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("flip_right", "m_flip_right_position", p_exeception);
//            return ServoErrorResultPosition;
//        }
//    }*/ // m_flip_right_position
//
//    /**
//     * Access the flip_right position.
//     */
//    /**
//     * We use units of mm here because that's the recommended units of measurement for the
//     * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
//     *      <ImageTarget name="stones" size="247 173"/>
//     * You don't *have to* use mm here, but the units here and the units used in the XML
//     * target configuration files *must* correspond for the math to work out correctly.
//     */
//    static final float mmPerInch        = 25.4f;
//    static final float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
//    static final float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
//    private boolean v_vuforia_inited = false;
//    private VuforiaLocalizer v_vuforia = null;
//    private List<VuforiaTrackable> v_vuforia_allTrackables = null;
//    private List<String> v_vuforia_messages = null;
//    public enum vuforia_targets{
//        Wheels (0,"Wheels"),
//        Tools(1, "Tools"),
//        Logos(2, "Logos"),
//        Gears(3, "Gears"),
//        BlueRedBeacon(4,"BlueRedBeacon"),
//        RedBlueBeacon(5,"RedBlueBeacon");
//        vuforia_targets(int targetIndex, String name) {
//            this.targetIndex = targetIndex;
//            this.name = name;
//        }
//        private final int targetIndex;   // in kilograms
//        private final String name;
//
//        public int targetIndex() { return targetIndex; }
//        public static vuforia_targets getEnum(int targetIndex){
//            switch (targetIndex){
//                case 0:
//                    return vuforia_targets.Wheels;
//                case 1:
//                    return vuforia_targets.Tools;
//                case 2:
//                    return vuforia_targets.Logos;
//                case 3:
//                    return vuforia_targets.Gears;
//                case 4:
//                    return vuforia_targets.BlueRedBeacon;
//                case 5:
//                    return vuforia_targets.RedBlueBeacon;
//
//            }
//            return null;
//        }
//    }
//    public void vuforia_Init(){
//        try{
//            VuforiaLocalizer.Parameters parameters;
//            if(v_debug) {
//                //parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
//                parameters = new VuforiaLocalizer.Parameters();
//            }else{
//                parameters = new VuforiaLocalizer.Parameters();
//            }
//            parameters.vuforiaLicenseKey = "ARr2v+H/////AAAAGbhUuRSeZUkynkK8ae61PIdjct7sVAoB5ItOs7Txvqsc9KlYRYHyftgUouhc+2Db+lSdUHCFdKp/CTYa3oWdQO3Bt3jkFplXQThhCFPnq0urXzwO0Mm5Jj1tYYuGZIU0anvdpA6DZVP95tL/FwRVO1BatviHrgurUy3L/TL7lPse5gI30PNKjgraalsKhmTxd13leA3dg+i/kqaTz3ot4iAmHEV6HBzsa3WUFSo1b6ig4Eo44j/O5J3CEQLWJYqRjlQwLUWB5QJi84YmhK2i+dSwdAXBc14Nb2QwsCjbbZA+XbSNxdDMKOTvCbVxHj+wL5Xare3nDZsPNTpEbKJ7ozaI7dcRJYCJK71X4Nv3fKn0";
//            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//            v_vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//            // only care about 1 at the moment so commented this line
//            // for performance Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
//            /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//            v_vuforia_allTrackables = new ArrayList<VuforiaTrackable>();
//            /**
//             * Load the data sets that for the trackable objects we wish to track. These particular data
//             * sets are stored in the 'assets' part of our application (you'll see them in the Android
//             * Studio 'Project' view over there on the left of the screen). You can make your own datasets
//             * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
//             * example "StonesAndChips", datasets can be found in in this project in the
//             * documentation directory.
//             */
//            VuforiaTrackables ftc = v_vuforia.loadTrackablesFromAsset("FTC_2016-17");
//
//            /**
//             * Create a transformation matrix describing where the phone is on the robot. Here, we
//             * put the phone on the right hand side of the robot with the screen facing in (see our
//             * choice of BACK camera above) and in landscape mode. Starting from alignment between the
//             * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
//             *
//             * When determining whether a rotation is positive or negative, consider yourself as looking
//             * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
//             * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
//             * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
//             * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
//             */
//            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                    .translation(0,0,0)
//                    .multiplied(Orientation.getRotationMatrix(
//                            AxesReference.EXTRINSIC, AxesOrder.YZY,
//                            AngleUnit.DEGREES, 180, 0, 0));
//
//            int trackableIndex = 0;
//            for (VuforiaTrackable trackable : ftc) {
//                /**
//                 * getUpdatedRobotLocation() will return null if no new information is available since
//                 * the last time that call was made, or if the trackable is not currently visible.
//                 * getRobotLocation() will return null if the trackable is not currently visible.
//                 */
//                vuforia_targets myTarget = vuforia_targets.getEnum(trackableIndex);
//                trackable.setName(myTarget.name);
//                VuforiaTrackableDefaultListener myListener = (VuforiaTrackableDefaultListener)trackable.getListener();
//                /**
//                 * Let the trackable listeners we care about know where the phone is. We know that each
//                 * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//                 * we have not ourselves installed a listener of a different type.
//                 */
//                myListener.setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//                //trackable.setListener(myListener);
//                v_vuforia_allTrackables.add(trackable);
//                trackableIndex++;
//            }
//            /** Start tracking the FTC targets. */
//            ftc.activate();
//
////                /*//Get our custom beacon trackables
////                ftc = v_vuforia.loadTrackablesFromAsset("FTC_Beacons");
////                for (VuforiaTrackable trackable : ftc) {
////                    *//**
////                     * getUpdatedRobotLocation() will return null if no new information is available since
////                     * the last time that call was made, or if the trackable is not currently visible.
////                     * getRobotLocation() will return null if the trackable is not currently visible.
////                     *//*
////                    vuforia_targets myTarget = vuforia_targets.getEnum(trackableIndex);
////                    trackable.setName(myTarget.name);
////                    VuforiaTrackableDefaultListener myListener = (VuforiaTrackableDefaultListener)trackable.getListener();
////                    *//**
////                     * Let the trackable listeners we care about know where the phone is. We know that each
////                     * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
////                     * we have not ourselves installed a listener of a different type.
////                     *//*
////                    myListener.setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
////                    //trackable.setListener(myListener);
////                    v_vuforia_allTrackables.add(trackable);
////                    trackableIndex++;
////                }
////                *//** Start tracking our  FTC targets. *//*
////                ftc.activate();*/
//
//            //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
//
////               v_vuforia..setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
////
/////*To access the image: you need to iterate through the images of the frame object:*/
////
////                CloseableFrame frame = locale.getFrameQueue().take() //takes the frame at the head of the queue
////                Image rgb = null;
////
////                long numImages = frame.getNumImages();
////
////
////                for (int i = 0; i < numImages; i++) {
////                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
////                        rgb = frame.getImage(i);
////                        break;
////                    }//if
////                }//for
//
//
//            v_vuforia_inited = true;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("vuforia_init", "Error", p_exeception);
//            v_vuforia_inited = false;
//        }
//    }
//
//    public boolean vuforia_targetVisible(int targetIndex){
//
//        if(v_vuforia_inited == true){
//            if(targetIndex < v_vuforia_allTrackables.size()){
//                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
//                return ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
//            }
//        }
//        return false;
//    }
//
//
//    private float v_vuforia_driveToTarget_power;
//    private float v_vuforia_driveToTarget_power_slow;
//
//
//    private void drive_setup_am20(){
//        v_drive_power = 0.8f;
//        //we have to move slower backing up to prevent a wheely
//        v_drive_power_reverse = 0.5f;
//        v_drive_power_slowdown = .5f;
//        v_turn_motorspeed = .3f;
//        v_turn_motorspeed_slow = .25f;
//        v_drive_inches_ticksPerInch = 45d;
//        v_drive_inches_strife_ticksPerInch = 45.00d;
//        v_drive_turn_ticks_per_degree = 5.5f;
//        v_vuforia_driveToTarget_power = .35f; //was .5f 12/16/2016
//        v_vuforia_driveToTarget_power_slow = .15f;
//        v_vuforia_driveToTarget_slowDownFactorClose = .009f;
//        v_vuforia_driveToTarget_slowDownFactor = .020f;
//        v_vuforia_findTargetSpeed = -.07f;
//        v_drive_use_slowdown = false;
//        v_drive_inches_slowdown = 24;
//        v_drive_inches_slowdown_ticks = (int)Math.round(v_drive_inches_slowdown * v_drive_inches_ticksPerInch);
//    }
//
//    private void drive_setup_am40(){
//        v_drive_power = 1.0f;
//        v_drive_power_reverse = 1.0f;
//        v_drive_power_slowdown = .5f;
//        v_turn_motorspeed = .1f;
//        v_turn_motorspeed_slow = .5f;
//        v_drive_inches_ticksPerInch = 88.00d;
//        v_drive_inches_strife_ticksPerInch = 88.00d;
//        v_drive_turn_ticks_per_degree = 10.83f;
//        v_vuforia_driveToTarget_power = .5f;
//        v_vuforia_driveToTarget_power_slow = .2f;
//        v_vuforia_driveToTarget_slowDownFactorClose = .009f;
//        v_vuforia_driveToTarget_slowDownFactor = .020f;
//        v_vuforia_findTargetSpeed = -.07f;
//        v_drive_use_slowdown = false;
//        v_drive_inches_slowdown = 12;
//        v_drive_inches_slowdown_ticks = (int)Math.round(v_drive_inches_slowdown * v_drive_inches_ticksPerInch);
//    }
//
//    private int v_vuforia_driveToTarget_Index = -1;
//    private ElapsedTime v_vuforia_driveToTarget_elapsedtime;
//    public boolean vuforia_driveToTarget(int targetIndex){
//
//        if(v_vuforia_inited == true){
//            v_vuforia_driveToTarget_elapsedtime = new ElapsedTime();
//            if(targetIndex < v_vuforia_allTrackables.size()){
//                setupAutoDrive();
//                v_vuforia_driveToTarget_Index = targetIndex;
//                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
//                    return false;
//                }else{
//
//                    drive_set_power(v_vuforia_driveToTarget_power,v_vuforia_driveToTarget_power);
//                    return true;
//                }
//            }
//        }
//        return false;
//    }
//    /*public boolean vuforia_driveToTargetFast(int targetIndex){
//
//        if(v_vuforia_inited == true){
//            if(targetIndex < v_vuforia_allTrackables.size()){
//                setupAutoDrive();
//                v_vuforia_driveToTarget_Index = targetIndex;
//                VuforiaTrackable trackable = v_vuforia_allTrackables.get(targetIndex);
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
//                    return false;
//                }else{
//
//                    set_drive_power(v_vuforia_driveToTarget_power_fast,v_vuforia_driveToTarget_power_fast);
//                    return true;
//                }
//            }
//        }
//        return false;
//    }*/
//
//    //    public void drive_setMaxSpeed(int tickPerSecond){
////        v_motor_left_drive.setMaxSpeed(tickPerSecond);
////        v_motor_right_drive.setMaxSpeed(tickPerSecond);
////    }
//    static final private int v_vuforia_driveToTarget_xmin = -140; //130
//    static final private int v_vuforia_driveToTarget_xmin_slow = -400;
//    private float v_vuforia_driveToTarget_slowDownFactorClose = .007f; //130
//    private float v_vuforia_driveToTarget_slowDownFactor = .020f;
//
//    static float v_vuforia_findTargetSpeed = -.1f;
//
//    public boolean vuforia_driveToTargetComplete(){
//        if(v_vuforia_inited == true){
//
//            VuforiaTrackable trackable = v_vuforia_allTrackables.get(v_vuforia_driveToTarget_Index);
//            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
//                if(v_vuforia_driveToTarget_elapsedtime.seconds() > 1 && v_vuforia_driveToTarget_elapsedtime.seconds() < 2) {
//                    v_motor_right_drive.setPower(v_vuforia_findTargetSpeed);
//                    v_motor_left_drive.setPower(0);
//                }else if(v_vuforia_driveToTarget_elapsedtime.seconds() > 2 && v_vuforia_driveToTarget_elapsedtime.seconds() < 4) {
//                    v_motor_right_drive.setPower(0);
//                    v_motor_left_drive.setPower(v_vuforia_findTargetSpeed);
//                }else {
//                    drive_set_power(0f, 0f);
//                }
//                //set_first_message("Target Not Visable stoping");
//            }else{
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
//
//                if (pose != null) {
//                    VectorF translation = pose.getTranslation();
//                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
//                    float v_motorpower_left ;
//                    float v_motorpower_right;
//                    float v_motorpower_slowDownFactor;
//                    float distanceX = translation.get(2);
//                    if(distanceX > v_vuforia_driveToTarget_xmin){
//                        drive_set_power(0.0f,0.0f);
//                        //set_first_message("Target: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString());
//                        return true;
//                    }
//                    if(distanceX > v_vuforia_driveToTarget_xmin_slow){
//                        v_motorpower_left = v_vuforia_driveToTarget_power_slow;
//                        v_motorpower_right = v_vuforia_driveToTarget_power_slow;
//                        v_motorpower_slowDownFactor = v_vuforia_driveToTarget_slowDownFactorClose;
//                    }else{
//                        v_motorpower_left = v_vuforia_driveToTarget_power;
//                        v_motorpower_right = v_vuforia_driveToTarget_power;
//                        v_motorpower_slowDownFactor = v_vuforia_driveToTarget_slowDownFactor;
//                    }
//                    if(degreesToTurn < 0 ){
//                        float rightPowerAdjust = (180f - (float)(0-degreesToTurn)) * v_motorpower_slowDownFactor;
//                            /*if(rightPowerAdjust > v_vuforia_maxslowDownFactor){
//                                rightPowerAdjust = v_vuforia_maxslowDownFactor;
//                            }*/
//                        v_motorpower_right = (v_motorpower_right -rightPowerAdjust);
//                    }else {
//                        float leftPowerAdjust = (180f - (float)degreesToTurn) * v_motorpower_slowDownFactor;
//                            /*if(leftPowerAdjust > v_vuforia_maxslowDownFactor){
//                                leftPowerAdjust = v_vuforia_maxslowDownFactor;
//                            }*/
//                        v_motorpower_left = (v_motorpower_left - leftPowerAdjust);
//                    }
//                    drive_set_power ( v_motorpower_left,v_motorpower_right);
//                    //set_first_message("searching: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString() );
//                }
//            }
//
//        }
//        return false;
//
//    }
//
//
///*
//    static final private int v_vuforia_driveToTargetFast_xmin = -2000;
//    static final private int v_vuforia_driveToTargetFast_xmin_slow = -1350;
//    public boolean vuforia_driveToTargetFastComplete(){
//        if(v_vuforia_inited == true){
//
//            VuforiaTrackable trackable = v_vuforia_allTrackables.get(v_vuforia_driveToTarget_Index);
//            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() == false){
//                set_drive_power(0f,0f);
//                //set_first_message("Target Not Visable stoping");
//            }else{
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
//
//                if (pose != null) {
//                    VectorF translation = pose.getTranslation();
//                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
//                    float v_motorpower_left ;
//                    float v_motorpower_right;
//                    float v_motorpower_slowDownFactor;
//                    float distanceX = translation.get(2);
//                    if(distanceX > v_vuforia_driveToTargetFast_xmin){
//                        set_drive_power(0.0f,0.0f);
//                        //set_first_message("Target: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString());
//                        return true;
//                    }
//                    if(distanceX > v_vuforia_driveToTargetFast_xmin_slow){
//                        v_motorpower_left = v_vuforia_driveToTarget_power_slow;
//                        v_motorpower_right = v_vuforia_driveToTarget_power_slow;
//                        v_motorpower_slowDownFactor = .007f;
//                    }else{
//                        v_motorpower_left = v_vuforia_driveToTarget_power_fast;
//                        v_motorpower_right = v_vuforia_driveToTarget_power_fast;
//                        v_motorpower_slowDownFactor = .01f;
//                    }
//                    if(degreesToTurn < 0 ){
//                        float rightPowerAdjust = (180f - (float)(0-degreesToTurn)) * v_motorpower_slowDownFactor;
//                            */
///*if(rightPowerAdjust > v_vuforia_maxslowDownFactor){
//                                rightPowerAdjust = v_vuforia_maxslowDownFactor;
//                            }*//*
//
//                        v_motorpower_right = (v_motorpower_right -rightPowerAdjust);
//                    }else {
//                        float leftPowerAdjust = (180f - (float)degreesToTurn) * v_motorpower_slowDownFactor;
//                            */
///*if(leftPowerAdjust > v_vuforia_maxslowDownFactor){
//                                leftPowerAdjust = v_vuforia_maxslowDownFactor;
//                            }*//*
//
//                        v_motorpower_left = (v_motorpower_left - leftPowerAdjust);
//                    }
//                    set_drive_power ( v_motorpower_left,v_motorpower_right);
//                    //set_first_message("searching: " + trackable.getName() + ":" + distanceX  + ":" + degreesToTurn + ":" + translation.toString() );
//                }
//            }
//
//        }
//        return false;
//
//    }
//*/
//
//
//
//    /**
//     * Turn on the red led located in the Device Interface Module
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//    public boolean redled_on () {
//        try {
//            if (v_dim != null) {
//                v_dim.setLED(1, true);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim redled", "redled_on", p_exeception);
//            return false;
//        }
//    }
//
//    /**
//     * Turn off the red led located in the Device Interface Module
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//    public boolean redled_off () {
//        try {
//            if (v_dim != null) {
//                v_dim.setLED(1, false);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim redled", "redled_off", p_exeception);
//            return false;
//        }
//    }
//
//    /**
//     * Toggles the current state of the red led located in the Device Interface Module
//     * <p>calling the function repeataly will give a blink effect.
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//
//    public boolean redled_toggle () {
//        try {
//            if (v_dim != null) {
//                boolean isEnabled = v_dim.getLEDState(1);
//                if (isEnabled) {
//                    isEnabled = false;
//                    set_second_message("Red Led set to Off");
//                } else {
//                    isEnabled = true;
//                    set_second_message("Blue Led set to On");
//                }
//                v_dim.setLED(1, isEnabled);
//                return isEnabled;
//            }else {
//                return false;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim redled", "redled_toggle", p_exeception);
//            return false;
//        }
//    }
//
//    /**
//     * Turn on the blue led located in the Device Interface Module
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//    public boolean blueled_on () {
//        try {
//            if (v_dim != null) {
//                v_dim.setLED(0, true);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim blueled", "blueled_on", p_exeception);
//            return false;
//        }
//    }
//
//
//    /**
//     * Turn off the blue led located in the Device Interface Module
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//    public boolean blueled_off () {
//        try {
//            if (v_dim != null) {
//                v_dim.setLED(0, false);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim blueled", "blueled_off", p_exeception);
//            return false;
//        }
//    }
//
//
//
//    /**
//     * Toggle the blue led located in the Device Interface Module
//     * @return returns true is successfull in turning on the led returns false on error
//     */
//    public boolean blueled_toggle () {
//        try {
//            if (v_dim != null) {
//                boolean isEnabled = v_dim.getLEDState(0);
//                if (isEnabled) {
//                    isEnabled = false;
//                    set_second_message("Blue Led set to Off");
//                } else {
//                    isEnabled = true;
//                    set_second_message("Blue Led set to On");
//                }
//                v_dim.setLED(0, isEnabled);
//
//                return isEnabled;
//            } else {
//                return false;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("dim blueled", "blueled_toggle", p_exeception);
//            return false;
//        }
//    }
//
//
//
//    /**
//     * reset the gyro heading to zero
//     */
//    private boolean sensor_gyro_mr_resetHeading(){
//        try{
//            if(v_sensor_gyro_mr != null){
//                // get the x, y, and z values (rate of change of angle).
//
//                v_sensor_gyro_mr.resetZAxisIntegrator();
//                return true;
//            }
//            return false;
//        }catch(Exception p_exeception)
//        {
//            debugLogException("sensor_gyro", "sensor_gyro_resetHeading", p_exeception);
//            return false;
//        }
//    }
//
////    public boolean beacon_make_red() throws InterruptedException{
////        //the sensor is on the right side so the logic is  to read the
////        //rgb sensor value if Red is higher then 1000 extend the right side pushing the button to make it
////        //blue  else if Blue is over 1000 then extend the left button
////        if(v_sensor_color_i2c_enabled){
////            int v_detectedColor = sensor_color_GreatestColor();
////            if(v_detectedColor == 0){
////                set_second_message("Red Detected on Right Pushing Right Side");
////                pushbutton_right_extend();
////                ////////pushbutton_left_extend();
////                timewait2Milliseconds(700);
////                while (timewait2Milliseconds_Complete()==false){
////                    hardware_loop();
////                }
////                pushbutton_right_retract();
////                return true;
////            }else if (v_detectedColor == 2){
////                set_second_message("Blue Detected on Right Pushing Left Side");
////                pushbutton_left_extend();
////                /////////////pushbutton_right_extend();
////                timewait2Milliseconds(700);
////                while (timewait2Milliseconds_Complete()==false){
////                    hardware_loop();
////                }
////                pushbutton_left_retract();
////                return true;
////            }else{
////                //
////                set_second_message("No Red or Blue Detected nothing to push");
////            }
////        }
////        return false;
////    }
////
//
//
//    //returns -1 if neither detected, 0 if red detected, 2 if blue detected higher
//    public int sensor_color_GreatestColor(){
//        if(v_sensor_color_i2c_enabled) {
//            int[] myRGBA = sensor_color_get_rgba();
//            //make sure at least one of them is over the min threshold else return -1
//            int cRed = myRGBA[0];
//            int cBlue = myRGBA[2];
//
//            if (cRed < v_sensor_color_min_value && cBlue < v_sensor_color_min_value) {
//                set_third_message("Both Red and Blue Under Min " + myRGBA[0] + ":" + myRGBA[1] + ":" + myRGBA[2]);
//                return -1;
//            }
//
//            if (cRed > cBlue && ((cRed - cBlue) > 50)) {
//                set_third_message("Red is Greater then Blue");
//                return 0;
//            } else if(cBlue > cRed && ((cBlue - cRed) > 50))  {
//                set_third_message("Blue is Greater then Red");
//                return 2;
//            }else{
//                set_third_message("Difference Under Min " + myRGBA[0] + ":" + myRGBA[1] + ":" + myRGBA[2]);
//                return -1;
//            }
//
//        }
//        return -1;
//    }
//
//
//
////    public boolean beacon_make_blue() throws InterruptedException{
////        //the sensor is on the right side so the logic is  to read the
////        //rgb sensor value if Red is higher then 1000 extend the right side pushing the button to make it
////        //blue  else if Blue is over 1000 then extend the left button
////        if(v_sensor_color_i2c_enabled){
////            int v_detectedColor = sensor_color_GreatestColor();
////            if(v_detectedColor == 0){
////                set_second_message("Red Detected on Right Pushing Left Side");
////                pushbutton_left_extend();
////                //////////pushbutton_right_extend();
////                timewait2Milliseconds(700); //.7f
////                while (timewait2Milliseconds_Complete()==false){
////                    hardware_loop();
////                }
////                pushbutton_left_retract();
////                return true;
////            }else if (v_detectedColor == 2){
////                set_second_message("Blue Detected on Right Pushing Right Side");
////                pushbutton_right_extend();
////                /////////////pushbutton_left_extend();
////                timewait2Milliseconds(700); //.7
////                while (timewait2Milliseconds_Complete()==false){
////                    hardware_loop();
////                }
////                pushbutton_right_retract();
////                return true;
////            }else{
////                //-1 returned so no color of threshold
////                //
////                set_second_message("No Red or Blue Detected nothing to push");
////            }
////        }
////        return false;
////    }
//
//    /**
//     * Enable the Color Sensor Led
//     * @return returns true is successfull returns false on error
//     */
//    public boolean sensor_color_led(boolean enable){
//        try{
//
//            if(v_sensor_color_i2c_led !=null) {
//                v_sensor_color_i2c_led_enabled = enable;
//                v_sensor_color_i2c_led.setState( v_sensor_color_i2c_led_enabled);
//                //v_sensor_color_i2c.enableLed(enable);
//                return true;
//            }
//            set_first_message("color led " + enable);
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_color", "sensor_color_led", p_exeception);
//            return false;
//        }
//    }
//
//    /**
//     * Enable the Legecy Color Sensor
//     * @return returns true is successfull returns false on error
//     */
//    public boolean sensor_color_enable(boolean enable){
//        try{
//            // convert the RGB values to HSV values.
//            if(v_sensor_color_i2c !=null) {
//                //turn on the led this is the only way legecy color will detect anything
//                v_sensor_color_i2c_enabled = enable;
//                set_second_message("sensor_color_enable " + enable );
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_color", "sensor_color_enable", p_exeception);
//            return false;
//        }
//    }
//
//    public boolean sensor_range_init(){
//        try {
//            // get a reference to our Rangesensor object.
//            //v_sensor_rangeSensor
//            v_sensor_rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, config_i2c_range);
//            // calibrate the gyro.
//            set_third_message("Range Sensor Found" + v_sensor_rangeSensor.getDistance(DistanceUnit.INCH));
//            return true;
//        }catch(Exception p_exeception){
//            debugLogException(config_i2c_range,"missing",p_exeception);
//            v_sensor_rangeSensor = null;
//            return false;
//        }
//    }
//
//    /**
//     * Enable the Range Sensor
//     * @return returns true is successfull returns false on error
//     */
//    public boolean sensor_range_enable(boolean enable){
//        try{
//            if(v_sensor_rangeSensor != null) {
//                v_sensor_rangeSensor_enabled = enable;
//                set_third_message("sensor range enable " + enable);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_ranger", "sensor_color_enable", p_exeception);
//            return false;
//        }
//    }
//
//    public int[] sensor_color_get_rgba(){
//        try{
//            return v_sensor_color_i2c_rgbaValues;
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_color", "sensor_color_read_rgb", p_exeception);
//            return v_sensor_color_i2c_rgbaValues;
//        }
//    }
//
//
//
//    /**
//     *
//     * @return gyro heading in degrees since reset
//     */
//    public int sensor_gyro_mr_get_heading(){
//        try{
//            if(v_sensor_gyro_mr != null) {
//                return v_sensor_gyro_mr.getHeading();
//            }else{
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_gyro", "sensor_gyro_get_heading", p_exeception);
//            return 0;
//        }
//
//
//    }
//
//    /**
//     *
//     * @return sensor_range
//     */
//    public double sensor_range_get_distance(){
//        try{
//            if(v_sensor_rangeSensor != null) {
//                return v_sensor_rangeSensor_distance;
//            }else{
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_range", "sensor_range", p_exeception);
//            return 0;
//        }
//
//
//    }
//
//    /**
//     * return the rawX rate
//     * @return gyro heading in degrees since reset
//     */
//    public int sensor_gyro_mr_get_rawX(){
//        try{
//            // get the x info.
//            if(v_sensor_gyro_mr != null) {
//                return v_sensor_gyro_mr.rawX();
//            }else{
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_gyro", "sensor_gyro_get_rawX", p_exeception);
//            return 0;
//        }
//
//    }
//
//    /**
//     * return the rawX rate
//     * @return gyro heading in degrees since reset
//     */
//    public int sensor_gyro_mr_get_rawY(){
//        try{
//            // get the heading info.
//            // the Modern Robotics' gyro sensor keeps
//            // track of the current heading for the Z axis only.
//            if(v_sensor_gyro_mr != null) {
//                return v_sensor_gyro_mr.rawY();
//            }
//            else{
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_gyro", "sensor_gyro_get_rawY", p_exeception);
//            return 0;
//        }
//    }
//
//    /**
//     * return the rawX rate
//     * @return gyro heading in degrees since reset
//     */
//    public int sensor_gyro_mr_get_rawZ(){
//        try{
//            // get the heading info.
//            // the Modern Robotics' gyro sensor keeps
//            // track of the current heading for the Z axis only.
//            if(v_sensor_gyro_mr != null) {
//                return v_sensor_gyro_mr.rawZ();
//            }
//            else{
//                return 0;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_gyro", "sensor_gyro_get_rawZ", p_exeception);
//            return 0;
//        }
//    }
//
//
//
//
//    /**
//     * Enable the Legecy Color Sensor
//     * @return returns true is successfull returns false on error
//     */
///*
//    public boolean sensor_colorLegecy_start(){
//        try{
//            // convert the RGB values to HSV values.
//            if(v_sensor_colorLegecy_rgbValues !=null) {
//                //turn on the led this is the only way legecy color will detect anything
//                sensor_colorLegecy_led(true);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_start", p_exeception);
//            return false;
//        }
//    }
//*/
//    /*public int[] sensor_colorLegecy_getLast_rgba(){
//        try{
//            return v_sensor_colorLegecy_rgbValues;
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_getLast_rgb", p_exeception);
//            return v_sensor_colorLegecy_rgbValues;
//        }
//    }
//
//
//    public double sensor_ultraLegecy_distance(){
//        try{
//            if(v_sensor_ultraLegecy != null){
//                if ((v_loop_ticks % v_sensor_ultraLegecy_ticksPerRead) == 0) {
//                    v_sensor_ultraLegecy_distance = v_sensor_ultraLegecy.getUltrasonicLevel();
//                }
//                return v_sensor_ultraLegecy_distance;
//            }else{
//                return 9999.9999;
//            }
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_ultraLegecy", "sensor_ultraLegecy_distance", p_exeception);
//            return 9999.9999;
//        }
//    }
//*/
//    //Lego Light Legecy Sensor Methods
//
//    //--------------------------------------------------------------------------
//
//    //
//    // a_ods_light_detected
//    /**
//     * Disables the Legecy Color Sensor
//     * @return returns true is successfull returns false on error
//     */
///*
//    public boolean sensor_colorLegecy_stop(){
//        try{
//            // convert the RGB values to HSV values.
//            if(v_sensor_colorLegecy_rgbValues !=null) {
//                //turn on the led this is the only way legecy color will detect anything
//                sensor_colorLegecy_led(false);
//                return true;
//            }
//            return false;
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_stop", p_exeception);
//
//            return false;
//        }
//    }
//*/
//
///*
//    private int[] sensor_colorLegecy_read_rgba(){
//        try{
//            // convert the RGB values to HSV values.
//            if(v_sensor_colorLegecy_rgbValues !=null) {
//                //v_sensor_color.enableLed(true);
//                // wait one cycle.
//                //waitOneFullHardwareCycle();
//                v_sensor_colorLegecy_rgbValues[0] = v_sensor_colorLegecy.red();
//                v_sensor_colorLegecy_rgbValues[1] = v_sensor_colorLegecy.green();
//                v_sensor_colorLegecy_rgbValues[2] = v_sensor_colorLegecy.blue();
//                v_sensor_colorLegecy_rgbValues[3] = v_sensor_colorLegecy.alpha();
//                // wait one cycle.
//                //waitOneFullHardwareCycle();
//                // v_sensor_color.enableLed(false);
//            }
//            //Color.RGBToHSV(v_sensor_color.red(), v_sensor_color.green(), v_sensor_color.blue(), v_sensor_color_hsvValues);
//            return v_sensor_colorLegecy_rgbValues;
//
//        }catch (Exception p_exeception)
//        {
//            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_read_rgb", p_exeception);
//            return v_sensor_colorLegecy_rgbValues;
//        }
//    }
//*/
//    //
//    /**
//     * Access the amount of light detected by the Optical Distance Sensor.
//     */
///*
//    public double sensor_lightLegecy_amountDetected ()
//
//    {
//        double l_return = 0.0;
//
//        if (v_sensor_lightLegecy != null)
//        {
//            v_sensor_lightLegecy.getLightDetected ();
//        }
//
//        return l_return;
//
//    }
//    public boolean sensor_lightLegecy_led(boolean enable){
//        if(v_sensor_lightLegecy != null) {
//            v_sensor_lightLegecy_enabled = enable;
//            v_sensor_lightLegecy.enableLed(enable);
//            return true;
//        }else{
//            return false;
//        }
//    }
//
//    public boolean sensor_lightLegecy_led_status(){
//        return v_sensor_lightLegecy_enabled;
//    }
//    public boolean sensor_lightLegecy_white_tape_detected(){
//        return a_light_white_tape_detected();
//    }
//
//*/
//    //--------------------------------------------------------------------------
//    //
//    // a_light_white_tape_detected
//    //
//    /**
//     * Access whether the Light Sensor is detecting white tape.
//     */
///*
//    private boolean a_light_white_tape_detected ()
//    {
//
//        //
//        // Assume not.
//        //
//        boolean l_return = false;
//
//        if (v_sensor_lightLegecy != null)
//        {
//            //
//            // Is the amount of light detected above the threshold for white
//            // tape?
//            //
//            if (v_sensor_lightLegecy.getLightDetected () > 0.8)
//            {
//                l_return = true;
//            }
//        }
//
//        //
//        // Return
//        //
//        return l_return;
//
//    } // a_ods_white_tape_detected
//*/
//
//    //Don't use these inless we are in linerOpMode
////    public void waitOneFullHardwareCycle() throws InterruptedException {
////        this.waitForNextHardwareCycle();
////        Thread.sleep(1L);
////        this.waitForNextHardwareCycle();
////    }
////
////    public void waitForNextHardwareCycle() throws InterruptedException {
////        synchronized(this) {
////            this.wait();
////        }
////    }
////
//    public void sleep(long milliseconds) {
//        try{
//
//            Thread.sleep(milliseconds);
//        }catch(InterruptedException ex){
//            //do stuff
//        }
//    }
//
//    //Below are The Telmetry Code to Write Debug to the Phones
//
//    String secondMessage = "N/A";
//
//
//    //--------------------------------------------------------------------------
//    //
//    // update_telemetry
//    //
//    /**
//     * Update the telemetry with current values from the base class.
//     */
//    private void update_telemetry ()
//
//    {
//        try {
//            opMode.telemetry.addData("00", loopCounter() + ":" + hardware_loop_slowtime_milliseconds() + ":"+ a_warning_message());
//            opMode.telemetry.addData("EE", v_errormessage);
//            opMode.telemetry.addData("01", zeroMessage);
//            v_zeromessage_set = false;
//            v_errormessage_set = false;
//            if (v_debug) {
//                opMode.telemetry.addData("02", firstMessage);
//                opMode.telemetry.addData("03", secondMessage);
//                opMode.telemetry.addData("04",  thirdMessage);
//                //
//                // Send telemetry data to the driver station.
//                //
//                opMode.telemetry.addData("05", "Gyro: H:" + sensor_gyro_mr_get_heading() + ",X:" + sensor_gyro_mr_get_rawX() + ",Y:" + sensor_gyro_mr_get_rawY() + ",Z:" + sensor_gyro_mr_get_rawZ());
//
//                opMode.telemetry.addData
//                        ("06"
//                                , "Left Drive: "
//                                        + a_left_drive_power()
//                                        + ", "
//                                        + a_left_encoder_count()
//                                        + ", "
//                                        + a_left_drive_mode()
//                        );
//                opMode.telemetry.addData
//                        ("07"
//                                , "Right Drive: "
//                                        + a_right_drive_power()
//                                        + ", "
//                                        + a_right_encoder_count()
//                                        + ", "
//                                        + a_right_drive_mode()
//                        );
//                if(v_motor_slider != null) {
//                    opMode.telemetry.addData("slider", " to %7d at %7d",
//                            v_motor_slider_Position,
//                            v_motor_slider.getCurrentPosition());
//                }
//                if(v_sensor_color_i2c_enabled) {
//                    int[] v_color_rgba = sensor_color_get_rgba();
//                    opMode.telemetry.addData(
//                            "color", "Color RGBA: " + v_color_rgba[0]
//                                    + "," + v_color_rgba[1]
//                                    + "," + v_color_rgba[2]
//                                    + "," + v_color_rgba[3]
//                    );
//                }
////                opMode.telemetry.addData
////                        ("06"
////                                , "RPA Base Position: " //+ a_rpabase_position()
////                        );
////                opMode.telemetry.addData
////                        ("07"
////                                , "RPA Arm Position: " //+ a_rpa_arm_power() + ":" + rpa_arm_extended() + ":" + rpa_arm_retracted()
////                        );
////                opMode.telemetry.addData(
////                        "08", "Flip: Right:" //+ a_flip_right_position()
////                );
//                /*opMode.telemetry.addData
//                        ("05"
//                                , "Arm Shoulder: " + a_arm_shoulder_position()
//                        );
//                opMode.telemetry.addData
//                        ("06"
//                                , "Arm jewel: " + a_arm_jewel_position()
//                        );
//                opMode.telemetry.addData
//                        ("07"
//                                , "Arm Wrist: " + a_arm_wrist_position()
//                        );
//
//
//
//                opMode.telemetry.addData(
//                        "1l", "Flip: Right:" + a_flip_right_position() + ", Left:" + a_flip_left_position()
//                );
//                opMode.telemetry.addData(
//                        "12", "Ultra: " + sensor_ultraLegecy_distance()
//                );
//                opMode.telemetry.addData(
//                        "13", "Light: tape:" + sensor_lightLegecy_white_tape_detected() + "," + sensor_lightLegecy_amountDetected()
//                );*/
//                if(v_vuforia_inited == true){
//                    int position = 0;
//                    for (VuforiaTrackable trackable : v_vuforia_allTrackables) {
//                        /**
//                         * getUpdatedRobotLocation() will return null if no new information is available since
//                         * the last time that call was made, or if the trackable is not currently visible.
//                         * getRobotLocation() will return null if the trackable is not currently visible.
//                         */
//
//                        opMode.telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
//
//                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
//
//                        if (pose != null) {
//                            VectorF translation = pose.getTranslation();
//                            double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
//                            opMode.telemetry.addData(trackable.getName() + "-Degrees", degreesToTurn);
//                            /*double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
//                            double degreesToTurn2 = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
//                            double degreesToTurn3 = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
//                            opMode.telemetry.addData(trackable.getName() + "-Degrees", degreesToTurn);
//                            opMode.telemetry.addData(trackable.getName() + "-Degrees2", degreesToTurn2);
//                            opMode.telemetry.addData(trackable.getName() + "-Degrees3", degreesToTurn3);*/
//                        }
//                    }
//                }
//            }
//        }catch (Exception p_exeception)
//        {
//            set_first_message("updateTelmetry: " + p_exeception.getLocalizedMessage());
//        }
//    } // update_telemetry
//
//    //--------------------------------------------------------------------------
//    //
//    // update_gamepad_telemetry
//    //
//    /**
//     * Update the telemetry with current gamepad readings.
//     */
//    public void update_gamepad_telemetry ()
//
//    {
//        //
//        // Send telemetry data concerning gamepads to the driver station.
//        //
//        if (v_debug) {
//            opMode.telemetry.addData("14", "GP1 Left: " + -opMode.gamepad1.left_stick_y);
//            opMode.telemetry.addData("15", "GP1 Right: " + -opMode.gamepad1.right_stick_y);
//            opMode.telemetry.addData("16", "GP2 Left: " + -opMode.gamepad2.left_stick_y);
//            opMode.telemetry.addData("17", "GP2 X: " + opMode.gamepad2.x);
//            opMode.telemetry.addData("18", "GP2 Y: " + opMode.gamepad2.y);
//            opMode.telemetry.addData("19", "GP2 A: " + opMode.gamepad2.a);
//            opMode.telemetry.addData("20", "GP1 LT: " + opMode.gamepad1.left_trigger);
//            opMode.telemetry.addData("21", "GP1 RT: " + opMode.gamepad1.right_trigger);
//        }
//    } // update_gamepad_telemetry
//
//    String zeroMessage = "";
//    public void set_message (String p_message)
//
//    {
//        v_zeromessage_set = true;
//        zeroMessage = p_message;
//        //if (v_debug) {
//            //DbgLog.msg(loopCounter() + ":0:" + p_message);
//            Log.d("CFPushBotHardware",loopCounter() + ":0:" + p_message);
//        //}
//    }
//
//
//    // set_first_message
//    //--------------------------------------------------------------------------
//    //
//    // set_first_message
//    //
//    /**
//     * Update the telemetry's first message with the specified message.
//     */
//    String firstMessage = "";
//    private void set_first_message (String p_message)
//    {
//
//        firstMessage = p_message;
//        if (v_debug) {
//            //DbgLog.msg(loopCounter() + ":1:" + p_message);
//            Log.d("CFPushBotHardware",loopCounter() + ":1:" + p_message);
//        }
//    } // set_first_message
//
//    //--------------------------------------------------------------------------
//    //
//    // set_second_message
//    //
//    /**
//     * Update the telemetry's first message with the specified message.
//     */
//    private void set_second_message (String p_message)
//
//    {
//        secondMessage = p_message;
//        if (v_debug) {
//            //DbgLog.msg(loopCounter() + ":2:" + p_message);
//            Log.d("CFPushBotHardware",loopCounter() + ":2:" + p_message);
//        }
//
//    }
//    //--------------------------------------------------------------------------
//    //
//    // set_second_message
//    //
//    /**
//     * Update the telemetry's first message with the specified message.
//     */
//    String thirdMessage = "";
//    private void set_third_message (String p_message)
//
//    {
//        thirdMessage = p_message;
//        if (v_debug) {
//            Log.d("CFPushBotHardware",loopCounter() + ":3:" + p_message);
//            //DbgLog.msg(loopCounter() + ":3:" + p_message);
//        }
//
//    }
//
//    // set_first_message
//    //--------------------------------------------------------------------------
//    //
//    // set_error_message
//    //
//    /**
//     * Update the telemetry's first message to indicate an error.
//     */
//    String v_errormessage = "";
//    boolean v_errormessage_set = false;
//    public void set_error_message (String p_message)
//
//    {
//        v_errormessage_set = true;
//        v_errormessage = "ERROR: " + p_message;
//
//        Log.e("CFPushBotHardware",loopCounter() + ":set_error_message:" + p_message);
//
//
//    } // set_error_message
//
//
//    //private ElapsedTime period  = new ElapsedTime();
//    /***
//     *
//     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//     * periodic tick.  This is used to compensate for varying processing times for each cycle.
//     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//     *
//     * @param periodMs  Length of wait cycle in mSec.
//     * @throws InterruptedException
//     */
//    /*private void waitForTick(long periodMs) throws InterruptedException {
//
//        long  remaining = periodMs - (long)period.milliseconds();
//
//        // sleep for the remaining portion of the regular cycle period.
//        if (remaining > 0)
//            Thread.sleep(remaining);
//
//        // Reset the cycle clock for the next pass.
//        period.reset();
//    }*/
//}