
import processing.serial.*;
import shapes3d.*;

Serial serialport;

Box shape;


void setup(){
  size(800, 600, P3D);
  //perspective(PI/3, float(width)/height, 1, 200);
  printArray(Serial.list());
  serialport = new Serial(this, "COM23",115200);
  shape = new Box(
    100,    // width
    200,    // height
    40      // depth
    );
}

float x,y,z = 0;
//------------------------------------------------------------------ 
void draw(){

  while (serialport.available() > 0) {
    String inByte = serialport.readString();
    println(inByte);
    float innums[] = float(split(inByte,','));
    println(innums);
    x = innums[0]+(3.141592/2);
    y = innums[1];
    z = innums[2];
  }
  
  
  background(0);
  pushMatrix();
  // Move the world view coordinates [0,0,0] 
  // to the centre of the display.
  translate(width/2, height/2);
  // The next few lines rotate the graphics 
  // context so we view the shape from dfiierent 
  // angles making it appear to be tumbling.
  rotateX(x);
  rotateY(z);
  rotateZ(y);

  // Render the shape on the main display area
  shape.draw(getGraphics());

  popMatrix();
  // Display a status bar and show the results from the
  // last shape pick operation (see mouseClicked method)
  fill(200, 255, 200);
}
