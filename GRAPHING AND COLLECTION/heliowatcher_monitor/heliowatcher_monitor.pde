//HelioWatcher Display Code
//ECE4760
//Jeremy Blum & Jason Wright

import processing.serial.*;
Serial port;
String data = "";

PImage img;
PFont font;

boolean first_comm = true;

float[] opt_rot_array = new float[900];
float[] opt_angle_array = new float[900];
float[] curr_rot_array = new float[900];
float[] curr_angle_array = new float[900];
float[] voltage_array = new float[900];

String filePath = "C:\\Users\\Jeremy Blum\\Dropbox\\08 Senior Second Semester (11-12)\\ECE 4760 - Designing with Microcontrollers\\HelioWatcher\\GRAPHING AND COLLECTION\\logs\\log-" + year() + month() + day() + hour() + minute() + second() + ".csv"; 

String date="";
String time= "";
String opt_rot="";
String opt_angle="";
String curr_rot="";
String curr_angle="";
String voltage="";
String quad_rot_adj="";
String quad_angle_adj="";

void setup(){
  size(1400, 1000);
  port = new Serial(this, "COM50", 9600);
  
  delay(500); //Let remote system boot
  //Tell it we want data!
  port.write("data\r\n");
  
  //Then wait till we've received our first valid string of data.
  port.bufferUntil(';');
  smooth();
  img = loadImage("display.jpg");
  font = loadFont("BrowalliaNew-Italic-40.vlw");
  textFont(font);
  frameRate(1);
  //Write to a CSV File!
  String[] csv_title ={"Date,Time,Optimal Rotation,Optimal Angle,Current Rotation,Current Angle,Voltage,Quadrature Rotation Adjust,Quadrature Angle Adjust"};
  appendToFile(filePath, csv_title);
  
  
}

void draw(){
  image(img, 0, 0);
  fill(50,50,50);
  
  //Axes Labels
  textSize(45);
  text("0\u00B0  /  0V", 10, 650);;
  text("+45\u00B0", 10, 475);
  text("+90\u00B0  /  25V", 10, 300);
  text("-45\u00B0", 10, 825);
  text("-90\u00B0", 10, 1000);
  
  //Date & Time
  textSize(50);
  text(date , 1000, 400);
  text(time.substring(0, time.length() - 3), 1200, 400);
  
  //Quadrature Info
  textSize(30);
  text("Quadrature Rotation Adjust:",   1000, 540);
  text("Quadrature Angle Adjust:",  1000, 580);
  
  //Graph Legend
  fill(109,207,71);  text("Current Rotation:",       1000, 620);
  fill(109,41,71);  text("Optimal Rotation:",       1000, 660);
  fill(37,87,223);  text("Current Angle:",          1000, 700);
  fill(238,58,0);  text("Optimal Angle:",          1000, 740);
  fill(222,0,18);  text("Panel Voltage:",          1000, 780);
  
  //Data
  fill(115,115,115);
  text(quad_rot_adj + "\u00B0",   1250, 540);
  text(quad_angle_adj + "\u00B0", 1250, 580);
  text(curr_rot + "\u00B0",       1250, 620);
  text(opt_rot + "\u00B0",        1250, 660);
  text(curr_angle + "\u00B0",     1250, 700);
  text(opt_angle + "\u00B0",      1250, 740);
  text(voltage + "V",             1250, 780);
  
  
  //Grid Lines
  for(int i = 0 ;i<=width/20;i++)
  {
    strokeWeight(1);
    stroke(200);
    line((-frameCount%20)+i*20-450,300,(-frameCount%20)+i*20-450,height);
    line(0,i*20 +300,width-450,i*20+300);
  }
  
  //Current Rotation
  //(-90 to 90 degrees)
  noFill();
  stroke(109,207,71);
  strokeWeight(5);
  beginShape();
  for(int i = 0; i<curr_rot_array.length;i++)
  {
    vertex(i,650-curr_rot_array[i]);
  }
  endShape();
  for(int i = 1; i<curr_rot_array.length;i++)
  {
    curr_rot_array[i-1] = curr_rot_array[i];
  }
  curr_rot_array[curr_rot_array.length-1]=(int) ((float(curr_rot)/90.0)*350.0);
  
  //Optimal Rotation
  //(-90 to 90 degrees)
  noFill();
  stroke(109,41,71);
  strokeWeight(5);
  beginShape();
  for(int i = 0; i<opt_rot_array.length;i++)
  {
    vertex(i,650-opt_rot_array[i]);
  }
  endShape();
  for(int i = 1; i<opt_rot_array.length;i++)
  {
    opt_rot_array[i-1] = opt_rot_array[i];
  }
  opt_rot_array[opt_rot_array.length-1]=(int) ((float(opt_rot)/90.0)*350.0);
  
  //Current Angle
  //(30 to 60 degrees)
  noFill();
  stroke(37,87,223);
  strokeWeight(5);
  beginShape();
  for(int i = 0; i<curr_angle_array.length;i++)
  {
    vertex(i,650-curr_angle_array[i]);
  }
  endShape();
  for(int i = 1; i<curr_angle_array.length;i++)
  {
    curr_angle_array[i-1] = curr_angle_array[i];
  }
  curr_angle_array[curr_angle_array.length-1]=(int) ((float(curr_angle)/90.0)*350.0);
  
  //Optimal Angle
  //(30 to 60 degrees)
  noFill();
  stroke(238,58,0);
  strokeWeight(5);
  beginShape();
  for(int i = 0; i<opt_angle_array.length;i++)
  {
    vertex(i,650-opt_angle_array[i]);
  }
  endShape();
  for(int i = 1; i<opt_angle_array.length;i++)
  {
    opt_angle_array[i-1] = opt_angle_array[i];
  }
  opt_angle_array[opt_angle_array.length-1]=(int) ((float(opt_angle)/90.0)*350.0);
  
  //Voltage (corresponds to amount of Sunlight
  //5V-25V
  noFill();
  stroke(222,0,18);
  strokeWeight(5);
  beginShape();
  for(int i = 0; i<voltage_array.length;i++)
  {
    vertex(i,650-voltage_array[i]);
  }
  endShape();
  for(int i = 1; i<voltage_array.length;i++)
  {
    voltage_array[i-1] = voltage_array[i];
  }
  voltage_array[voltage_array.length-1]=(int) ((float(voltage)/25)*350.0);
  
  
}

void serialEvent(Serial port){
  int start = 3;
  if (first_comm == true)
  {
    start = 6;
    first_comm = false;
  }
  data = port.readStringUntil(';');
  data = data.substring(start, data.length() - 1);
  println(data);
  //Grab the Actual values from this data string
  String[] values = splitTokens(data, ",");
  date = values[0].substring(0,2) + "/" + values[0].substring(2,4) + "/" + values[0].substring(4,6);
  time = values[1].substring(0,2) + ":" + values[1].substring(2,4) + ":" + values[1].substring(4,6);
  opt_rot=values[2];
  opt_angle=values[3]; 
  curr_rot=values[4];
  curr_angle=values[5];
  voltage=values[6];
  quad_rot_adj=values[7];
  quad_angle_adj=values[8];
  
  //Write to a CSV File!
  String[] csv_data ={date + "," + time + "," + opt_rot + "," + opt_angle + "," + curr_rot + "," + curr_angle + "," + voltage + "," + quad_rot_adj + "," + quad_angle_adj};
  appendToFile(filePath, csv_data);
}

//http://forum.processing.org/topic/log-data-on-a-csv-file-is-it-possible
void appendToFile(String filePath, String[] data)
{
  PrintWriter pw = null;
  try
  {
    pw = new PrintWriter(new BufferedWriter(new FileWriter(filePath, true))); // true means: "append"
    for (int i = 0; i < data.length; i++)
    {
      pw.println(data[i]);
    }
  }
  catch (IOException e)
  {
    // Report problem or handle it
    e.printStackTrace();
  }
  finally
  {
    if (pw != null)
    {
      pw.close();
    }
  }
}
