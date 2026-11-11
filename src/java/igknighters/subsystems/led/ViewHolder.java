package igknighters.subsystems.led;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ViewHolder {
    private ArrayList<AddressableLEDBufferView> bufferViews;
    private ArrayList<Segment> segments;
    private AddressableLEDBuffer buffer;
    private class Segment{
        public String name;
        public int start;
        public int length;
        public int offset;
        public AddressableLEDBuffer bufferViews;
        public LEDPattern patternGetter();
        
        public void Segment(String name, int start, int length, int offset){
            this.name = name;
            this.start = start;
            this.length = length;
            this.offset = offset;
        }

    }   

    public void ViewHolder(AddressableLEDBuffer buffer){
        this.buffer = buffer;
    }

    public void addSegment(String Name, int length, int offset){
        Segment segment = new Segment(Name, length, );
        AddressableLEDBufferView newBufferView = buffer.createView(segment.start, segment.start + segment.length -1);
    }

    public void 

}
