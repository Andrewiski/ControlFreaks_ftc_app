package org.firstinspires.ftc.teamcode.ControlFreaks.Pixy;

//import com.qualcomm.ftccommon.DbgLog;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;
import org.firstinspires.ftc.teamcode.ControlFreaks.Wire;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.List;

/**
 * Created by adevries on 11/13/2017.
 * Requires Special Pixy Firmware
 */
public class PixyCamera {
    static final String logId = "Pixy:";       // Tag identifier in FtcRobotController.LogCat
    private Boolean debug = false;
    private String v_deviceName = "Not Set";
    //Put the Pixy in Lego Mode running my special firmware
    private byte I2CAddress = 0x54;
    private Wire v_pixy;

    private PixyBlock v_emptyBlock;
    private PixyBlockList v_emptyBlockList;
    // Signature 0 is Max Area Block 1-7 is Signatures 1-7, 8 is Color Code Query
    private PixyBlock[] v_signatureBlocks;
    private PixyBlockList[] v_signatureMaxBlocks;

    //This is Used to enable querys for a particular Signature
    //Call Signature
    private Boolean[] v_signatureEnable;
    private Boolean[] v_maxSignatureEnable;

    //83 in decimal converted to Oct is 123 so we are looking for sig 1 next to sig 2 next to sig 3
    //In windows open calculator click view then programmer
    public int color_code = 83;

    // Communication/misc parameters
    private static final byte PIXY_SERVO_SYNC = (byte) 0xff;
    private static final byte PIXY_CAM_BRIGHTNESS_SYNC = (byte) 0xfe;
    private static final byte PIXY_LED_SYNC = (byte) 0xfd;
    private static final int PIXY_MAX_SIGNATURE = 7;  //if Signature number is higher then 7 then its is a ColorCode Signature

    private static final int PIXY_SIGNATURE_BYTES = 4;
    private static final int PIXY_LARGEST_SIGNATURE_BYTES = 6;
    private static final int PIXY_CC_SIGNATURE_BYTES = 5;


    private static final int PIXY_MAXSIGNATURES_BYTES = 26;
    private static final int PIXY_MAXSIGNATURE_BYTES = 25;
    private static final int PIXY_MAXCCSIGNATURE_BYTES = 25;

    // RC-servo values
    private static final int PIXY_RCS_MIN_POS = 0;
    private static final int PIXY_RCS_MAX_POS = 1000;
    private static final int PIXY_RCS_CENTER_POS = ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2);

    public enum BlockType {
        NORMAL_BLOCK,
        CC_BLOCK
    }

    private BlockType g_blockType;
    private boolean v_pixy_enabled = false;


    public PixyCamera(HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception {
        try {
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();
        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
            throw p_exeception;
        }
    }

    public PixyCamera(HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize class members.
        //
        // All via self-construction.
        try {
            v_deviceName = deviceName;
            //Depends on FTC Wire Class https://github.com/OliviliK/FTC_Library
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();

        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
            throw p_exeception;
        }

    }

    private void init() {
        v_emptyBlock = new PixyBlock();
        v_emptyBlockList = new PixyBlockList(1);
        v_signatureBlocks = new PixyBlock[9];
        v_signatureEnable = new Boolean[9];
        v_maxSignatureEnable = new Boolean[9];
        //common code called by the two contructors above
        //Init all the blocks to empty blocks
        for (int i = 0; i < v_signatureBlocks.length; i++) {
            v_signatureBlocks[i] = new PixyBlock();
        }
        for (int i = 0; i < v_signatureEnable.length; i++) {
            v_signatureEnable[i] = false;
        }
        for (int i = 0; i < v_maxSignatureEnable.length; i++) {
            v_maxSignatureEnable[i] = false;
        }

        v_signatureMaxBlocks = new PixyBlockList[9];
        for (int i = 0; i < v_signatureMaxBlocks.length; i++) {
            if (i == 0) {
                v_signatureMaxBlocks[i] = new PixyBlockList(5);

            } else if (i == 8) {
                v_signatureMaxBlocks[i] = new PixyBlockList(4);
            } else {
                v_signatureMaxBlocks[i] = new PixyBlockList(6);
            }

        }
    }

    //When the Pixy is Enabled we start the i2c request to get the data we care about
    private void beginrequests() {
        try {
            if (v_pixy != null) {
                if (v_signatureEnable[0] == true) {
                    //Six bytes for the get Largest Blocks
                    v_pixy.requestFrom(0x50, PIXY_LARGEST_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[1] == true) {
                    v_pixy.requestFrom(0x51, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[2] == true) {
                    v_pixy.requestFrom(0x52, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[3] == true) {
                    v_pixy.requestFrom(0x53, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[4] == true) {
                    v_pixy.requestFrom(0x54, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[5] == true) {
                    v_pixy.requestFrom(0x55, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[6] == true) {
                    v_pixy.requestFrom(0x56, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[7] == true) {
                    v_pixy.requestFrom(0x57, PIXY_SIGNATURE_BYTES);
                }
                if (v_signatureEnable[8] == true) {
                    v_pixy.writeLH(0x58, color_code);
                    v_pixy.requestFrom(0x58, PIXY_CC_SIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[0] == true) {
                    //Six bytes for the get Largest Blocks
                    v_pixy.requestFrom(0x70, PIXY_MAXSIGNATURES_BYTES);
                }
                if (v_maxSignatureEnable[1] == true) {
                    v_pixy.requestFrom(0x71, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[2] == true) {
                    v_pixy.requestFrom(0x72, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[3] == true) {
                    v_pixy.requestFrom(0x73, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[4] == true) {
                    v_pixy.requestFrom(0x74, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[5] == true) {
                    v_pixy.requestFrom(0x75, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[6] == true) {
                    v_pixy.requestFrom(0x76, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[7] == true) {
                    v_pixy.requestFrom(0x77, PIXY_MAXSIGNATURE_BYTES);
                }
                if (v_maxSignatureEnable[8] == true) {
                    v_pixy.requestFrom(0x78, PIXY_MAXCCSIGNATURE_BYTES);
                }
            } else {
                warningPrint("beginrequests v_pixy is null");
            }
        } catch (Exception p_exeception) {
            debugLogException("Error beginrequests()", p_exeception);
            throw p_exeception;
        }

    }

    //    public int getColorCode(){
//        return v_color_code;
//    }
//    public void setColorCode(int colorCode){
//        v_color_code = colorCode;
//    }
    private PixyBlock v_signatureBlock;


    /**
     * put this in your state machine loop so its called over and over in the loop
     **/

    public void loop() {
        if (v_pixy_enabled && v_pixy != null) {
            if (v_pixy.responseCount() > 0) {
                v_pixy.getResponse();
                int regNumber = v_pixy.registerNumber();
                if (v_pixy.isRead()) {
                    int regCount = v_pixy.available();
                    int signum;
                    switch (regNumber) {
                        case 0x50: //Largest Block
                            if (regCount == PIXY_LARGEST_SIGNATURE_BYTES) {
                                v_signatureBlock = new PixyBlock();
                                v_signatureBlock.numBlocks = 1;
                                v_signatureBlock.signature = v_pixy.readLH();
                                v_signatureBlock.x = v_pixy.read();
                                v_signatureBlock.y = v_pixy.read();
                                v_signatureBlock.width = v_pixy.read();
                                v_signatureBlock.height = v_pixy.read();
                                debugPrint("largestBlock:" + v_signatureBlock.print());
                                //ask for largestBlock again
                                v_signatureBlocks[0] = v_signatureBlock;
                                if (v_signatureEnable[0]) {
                                    //v_pixy.beginWrite(0x50);
                                    //v_pixy.endWrite();
                                    v_pixy.requestFrom(0x50, PIXY_LARGEST_SIGNATURE_BYTES);
                                }

                            } else {
                                warningPrint("largestBlock: Error  only " + regCount + " bytes");
                                //ask for largestBlock again
                                if (v_signatureEnable[0]) {
                                    //v_pixy.beginWrite(0x50);
                                    //v_pixy.endWrite();
                                    v_pixy.requestFrom(0x50, PIXY_LARGEST_SIGNATURE_BYTES);
                                }
                            }
                            break;
                        case 0x51: //Signature 1 Block
                        case 0x52: //Signature 2 Block
                        case 0x53: //Signature 3 Block
                        case 0x54: //Signature 4 Block
                        case 0x55: //Signature 5 Block
                        case 0x56: //Signature 6 Block
                        case 0x57: //Signature 7 Block
                            signum = (regNumber & 0x07);
                            if (regCount == PIXY_SIGNATURE_BYTES) {
                                v_signatureBlock = new PixyBlock();
                                v_signatureBlock.signature = signum;
                                v_signatureBlock.numBlocks = v_pixy.read();
                                v_signatureBlock.x = v_pixy.read();
                                v_signatureBlock.y = v_pixy.read();
                                v_signatureBlock.width = v_pixy.read();
                                v_signatureBlock.height = v_pixy.read();
                                debugPrint("signature" + signum + ":" + v_signatureBlock.print());
                                v_signatureBlocks[signum] = v_signatureBlock;
                                //ask for SignatureBlock 1 again
                                if (v_signatureEnable[signum]) {
                                    //v_pixy.beginWrite((0x50 | signum));
                                    //v_pixy.endWrite();
                                    v_pixy.requestFrom((0x50 | signum), PIXY_SIGNATURE_BYTES);
                                }
                            } else {
                                warningPrint("signature" + signum + ": Error  only " + regCount + " bytes");
                                //ask for SignatureBlock again
                                if (v_signatureEnable[signum]) {
                                    v_pixy.requestFrom((0x50 | signum), PIXY_SIGNATURE_BYTES);
                                }
                            }
                            break;
                        case 0x58: //Color Code Query Signature  Block
                            if (regCount == PIXY_CC_SIGNATURE_BYTES) {
                                v_signatureBlock = new PixyBlock();
                                v_signatureBlock.numBlocks = v_pixy.read();
                                v_signatureBlock.signature = color_code; // v_pixy.readLH();;
                                v_signatureBlock.x = v_pixy.read();
                                v_signatureBlock.y = v_pixy.read();
                                v_signatureBlock.width = v_pixy.read();
                                v_signatureBlock.height = v_pixy.read();
                                debugPrint("cc signature" + color_code + ":" + v_signatureBlock.print());
                                v_signatureBlocks[8] = v_signatureBlock;
                                //ask for SignatureBlock 1 again
                                if (v_signatureEnable[8]) {
                                    v_pixy.writeLH(0x58, color_code);
                                    v_pixy.requestFrom(0x58, PIXY_CC_SIGNATURE_BYTES);
                                }
                            } else {
                                warningPrint("cc signature" + color_code + ": Error  only " + regCount + " bytes");
                                //ask for SignatureBlock again
                                if (v_signatureEnable[8]) {
                                    v_pixy.writeLH(0x58, color_code);
                                    v_pixy.requestFrom(0x58, PIXY_CC_SIGNATURE_BYTES);
                                }
                            }
                            break;
                        //FTC MaxBlocks Code
                        case 0x70: //Largest Signature 1-7 Blocks
                            if (regCount == PIXY_MAXSIGNATURES_BYTES) {
                                v_signatureMaxBlocks[0].BlockCount = v_pixy.read();
                                for (int i = 0; i < 5; i++) {
                                    PixyBlock v_signatureBlock = v_signatureMaxBlocks[0].Blocks[i];
                                    v_signatureBlock.signature = v_pixy.read();
                                    v_signatureBlock.x = v_pixy.read();
                                    v_signatureBlock.y = v_pixy.read();
                                    v_signatureBlock.width = v_pixy.read();
                                    v_signatureBlock.height = v_pixy.read();
                                    v_signatureBlock.angle = 0;
                                    debugPrint("maxSignatureBlocks:" + v_signatureBlock.print());
                                    //ask for largestBlock again
                                    if (v_maxSignatureEnable[0]) {
                                        v_pixy.requestFrom(0x70, PIXY_MAXSIGNATURES_BYTES);
                                    }
                                }
                            } else {
                                warningPrint("largestBlock: Error  only " + regCount + " bytes");
                                //ask for largestBlock again
                                if (v_signatureEnable[0]) {
                                    //v_pixy.beginWrite(0x50);
                                    //v_pixy.endWrite();
                                    v_pixy.requestFrom(0x70, PIXY_MAXSIGNATURES_BYTES);
                                }
                            }
                            break;
                        case 0x71: //Signature 1 Block
                        case 0x72: //Signature 2 Block
                        case 0x73: //Signature 3 Block
                        case 0x74: //Signature 4 Block
                        case 0x75: //Signature 5 Block
                        case 0x76: //Signature 6 Block
                        case 0x77: //Signature 7 Block
                            signum = (regNumber & 0x07);
                            if (regCount == PIXY_MAXSIGNATURE_BYTES) {
                                v_signatureMaxBlocks[signum].BlockCount = v_pixy.read();
                                for (int i = 0; i < 6; i++) {
                                    PixyBlock v_signatureBlock = v_signatureMaxBlocks[signum].Blocks[i];
                                    v_signatureBlock.signature = signum;
                                    v_signatureBlock.numBlocks = v_pixy.read();
                                    v_signatureBlock.x = v_pixy.read();
                                    v_signatureBlock.y = v_pixy.read();
                                    v_signatureBlock.width = v_pixy.read();
                                    v_signatureBlock.height = v_pixy.read();
                                    debugPrint("signature" + signum + ":" + v_signatureBlock.print());

                                    //ask for SignatureBlock 1 again
                                    if (v_maxSignatureEnable[signum]) {
                                        //v_pixy.beginWrite((0x70 | signum));
                                        //v_pixy.endWrite();
                                        v_pixy.requestFrom((0x70 | signum), PIXY_MAXSIGNATURE_BYTES);
                                    }
                                }
                            } else {
                                warningPrint("signature" + signum + ": Error  only " + regCount + " bytes");
                                //ask for SignatureBlock again
                                if (v_signatureEnable[signum]) {
                                    v_pixy.requestFrom((0x70 | signum), PIXY_MAXSIGNATURE_BYTES);
                                }
                            }
                            break;
                        case 0x78: //Color Code Max Signature  Block
                            if (regCount == PIXY_MAXCCSIGNATURE_BYTES) {
                                v_signatureMaxBlocks[8].BlockCount = v_pixy.read();
                                for (int i = 0; i < 4; i++) {
                                    PixyBlock v_signatureBlock = v_signatureMaxBlocks[8].Blocks[i];
                                    v_signatureBlock.signature = v_pixy.readLH();
                                    ;
                                    v_signatureBlock.x = v_pixy.read();
                                    v_signatureBlock.y = v_pixy.read();
                                    v_signatureBlock.width = v_pixy.read();
                                    v_signatureBlock.height = v_pixy.read();
                                    debugPrint("cc Max signatures:" + v_signatureBlock.print());
                                }
                                //ask for SignatureBlock 1 again
                                if (v_maxSignatureEnable[8]) {
                                    v_pixy.requestFrom(0x78, PIXY_MAXCCSIGNATURE_BYTES);
                                }
                            } else {
                                warningPrint("cc signature" + color_code + ": Error  only " + regCount + " bytes");
                                //ask for SignatureBlock again
                                if (v_signatureEnable[8]) {
                                    v_pixy.writeLH(0x58, color_code);
                                    v_pixy.requestFrom(0x58, PIXY_CC_SIGNATURE_BYTES);
                                }
                            }
                            break;
                    }
                }
                if (v_pixy.isWrite()) {
                    int regValue = v_pixy.read();
                    debugPrint(String.format("Pixy Write reg 0x%02X = 0x%02X", regNumber, regValue));
                }
            }
        }
    }


    //Signature 0 is max area Block request, Signature 8 is Color Code Query
    public void signature_enable(int signature, Boolean enable) {
        if (signature >= 0 && signature <= 8) {
            v_signatureEnable[signature] = enable;
            //if we are already enabled we need to send the first request to set things in motion
            if (v_pixy_enabled && enable == true) {
                //make the intial request to set things in motion
                if (signature == 0) {
                    //Six bytes for the get Largest Blocks
                    v_pixy.requestFrom(0x50, PIXY_LARGEST_SIGNATURE_BYTES);
                }
                if (signature >= 1 && signature <= 7) {
                    v_pixy.requestFrom((0x50 & signature), PIXY_SIGNATURE_BYTES);
                }

                if (signature == 8) {
                    v_pixy.writeLH(0x58, color_code);
                    v_pixy.requestFrom(0x58, PIXY_CC_SIGNATURE_BYTES);
                }
            }
        }
    }

    public boolean signature_isEnabled(int signature, Boolean enable) {
        if (signature >= 0 && signature <= 8) {
            return v_signatureEnable[signature];
        } else {
            return false;
        }
    }

    //Signature 0 is max area Block request, Signature 8 is Color Code Query
    public void maxSignature_enable(int signature, Boolean enable) {
        if (signature >= 0 && signature <= 8) {
            v_maxSignatureEnable[signature] = enable;
            //if we are already enabled we need to send the first request to set things in motion
            if (v_pixy_enabled && enable == true) {
                //make the intial request to set things in motion
                if (signature == 0) {
                    //Six bytes for the get Largest Blocks
                    v_pixy.requestFrom(0x70, PIXY_MAXSIGNATURES_BYTES);
                }
                if (signature >= 1 && signature <= 7) {
                    v_pixy.requestFrom((0x70 & signature), PIXY_MAXSIGNATURE_BYTES);
                }

                if (signature == 8) {
                    v_pixy.requestFrom(0x78, PIXY_MAXCCSIGNATURE_BYTES);
                }
            }
        }
    }

    public boolean maxSignature_isEnabled(int signature, Boolean enable) {
        if (signature >= 0 && signature <= 8) {
            return v_maxSignatureEnable[signature];
        } else {
            return false;
        }
    }

    public boolean set_servos(int s0, int s1) {
        if (v_pixy != null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_SERVO_SYNC);
            v_pixy.write(s0); //two bytes here may need shift
            v_pixy.write(s1);
            v_pixy.endWrite();
            return true;
        } else {
            return false;
        }
    }

    public boolean set_led(byte red, byte green, byte blue) {
        if (v_pixy != null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_LED_SYNC);
            v_pixy.write(red);
            v_pixy.write(green);
            v_pixy.write(blue);
            v_pixy.endWrite();
            return true;
        } else {
            return false;
        }
    }

    public boolean set_brightness(byte brightness) {
        if (v_pixy != null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_CAM_BRIGHTNESS_SYNC);
            v_pixy.write(brightness);
            v_pixy.endWrite();
            return true;
        } else {
            return false;
        }
    }

    public PixyBlock emptyBlock() {
        return v_emptyBlock;
    }

    public PixyBlockList emptyBlockList() {
        return v_emptyBlockList;
    }

    public PixyBlock largestSignatureBlock(int signature) {
        if (signature >= 0 && signature <= 8) {
            return v_signatureBlocks[signature];
        } else {
            return v_emptyBlock;
        }
    }

    public PixyBlockList maxSignatureBlocks(int signature) {
        if (signature >= 0 && signature <= 8) {
            return v_signatureMaxBlocks[signature];
        } else {
            return v_emptyBlockList;
        }
    }


    public boolean isEnabled() {
        return v_pixy_enabled;
    }

    public void enabled(boolean enable) {
        try {
            if (v_pixy_enabled != enable) {
                v_pixy_enabled = enable;
                if (enable == true) {
                    //Make the intial requests
                    beginrequests();
                }
            }

        } catch (Exception p_exeception) {
            debugLogException("Error enabled()", p_exeception);
            throw p_exeception;
        }
    }

    public void stop() {
        if (v_pixy != null) {
            v_pixy_enabled = false;
            v_pixy.close();
        }
    }

    void debugLogException(String msg, Exception ex) {

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            } else {
                debugMessage = debugMessage + " error. is null\n";
            }
            String stackTrace = Log.getStackTraceString(ex);
            if (stackTrace != null) {
                debugMessage = debugMessage + "\n" + stackTrace;
            }
        } else {
            debugMessage = debugMessage + " error is null";
        }
        android.util.Log.e("pixyCamera", debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }

    void debugPrint(String msg) {
        if (debug) {
            String debugMessage = logId + msg;
            android.util.Log.i("pixyCamera", debugMessage);
            //DbgLog.msg(debugMessage);
            //telemetry.addData(line, debugMessage);
        }
    }

    void warningPrint(String msg) {

        String debugMessage = logId + msg;
        android.util.Log.e("pixyCamera", debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }







}
