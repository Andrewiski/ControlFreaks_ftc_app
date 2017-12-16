package org.firstinspires.ftc.teamcode.ControlFreaks.Pixy;

import java.util.Arrays;

/**
 * Created by adevries on 12/6/2017.
 */

public class PixyBlockList {

        public int BlockCount;
        public PixyBlock[] Blocks;

        public PixyBlockList(int maxBlockCount) {
            BlockCount = 0;
            Blocks = new PixyBlock[maxBlockCount];
            for (int k = 0; k < Blocks.length; k++) {
                Blocks[k] = new PixyBlock();
            }
        }

        public String print() {
            try {
                String retval = "blockCount: " + BlockCount;
                for (int i = 0; i < Blocks.length && i < BlockCount ; i++) {
                    retval = retval + "\n" + Blocks[i].print();
                }
                return retval + "\n";
            } catch (Exception ex) {
                return "Error BlockList.print() " + ex.getMessage();
            }
        }

        public void SortBottomLeftTopRight(int yFudge){
            PixyBlock.BlockYXComparatorYFudge = yFudge;
            Arrays.sort(Blocks,PixyBlock.BlockYXComparator);
        }

}
