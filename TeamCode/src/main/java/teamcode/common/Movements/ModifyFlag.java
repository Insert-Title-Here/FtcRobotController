package teamcode.common.Movements;

public class ModifyFlag extends Movement{

    private int index;
    private boolean val;

    public ModifyFlag(boolean val, int index){
        this.index = index;
        this.val = val;
    }

    public boolean getVal(){
        return val;
    }
    public int getIndex() {
        return index;
    }


}
