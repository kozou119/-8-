import processing.serial.*;

// Set actual three ZigBee dongle port names from Serial.list(). Empty = offline.
String[] PORT_NAMES = {"", "", ""};
Serial[] radios = new Serial[3];
TelemetryFrame[] frames = new TelemetryFrame[3];
int[] receivedAt = new int[3];
int[] source = {-1, -1, -1};
boolean[] idConflict = new boolean[3];
String[] states = {"WAIT", "ENTER", "SEARCH", "APPROACH", "ALIGN", "DELIVER",
                   "PUSH", "RELEASE", "RETREAT", "TURN", "FINISHED", "FAULT"};
String[] reasons = {"OK", "NOT CONFIGURED", "SENSOR", "TIME UP", "MANUAL STOP",
                   "BOUNDARY", "LOST CUP", "TIMEOUT", "SLOPE"};
String[] floors = {"UNKNOWN", "WHITE", "BLACK", "RED", "BLUE"};

void setup() {
  size(1230, 630);
  textFont(createFont("SansSerif", 18));
  println(Serial.list());
  for (int i=0; i<3; ++i) {
    if (PORT_NAMES[i].length()==0) continue;
    try {
      radios[i] = new Serial(this, PORT_NAMES[i], 9600);
      radios[i].clear();
      radios[i].bufferUntil('\n');
    } catch (Exception e) { println("Cannot open " + PORT_NAMES[i] + ": " + e.getMessage()); }
  }
}

synchronized void serialEvent(Serial port) {
  String line = port.readStringUntil('\n');
  TelemetryFrame frame = TelemetryFrame.parse(line);
  if (frame==null) return;
  int robot = frame.id-1;
  int radio = -1;
  for (int i=0; i<3; ++i) if (radios[i]==port) radio=i;
  if (radio<0) return;
  if (source[robot]>=0 && source[robot]!=radio) {
    idConflict[robot] = true; // Latch until dashboard restart after fixing ROBOT_ID.
    return;
  }
  source[robot]=radio;
  frames[robot]=frame;
  receivedAt[robot]=millis();
}

synchronized void draw() {
  background(18, 24, 34);
  fill(240); textSize(26); text("ZUMO CONTEST / LIVE STATE", 25, 38);
  textSize(15); fill(170);
  text("Delivery attempts are NOT confirmed points. Position and cup identity are not measured.", 25, 66);
  for (int i=0; i<3; ++i) {
    float x=20+i*405;
    TelemetryFrame f=frames[i];
    boolean online=f!=null && millis()-receivedAt[i]<1500 && !idConflict[i];
    fill(29, 39, 53); noStroke(); rect(x, 90, 390, 510, 14);
    fill(online ? color(70, 230, 150) : color(255, 130, 90));
    textSize(22); text("ROBOT " + (i+1) + " / " + (idConflict[i] ? "ID CONFLICT" : online ? "LIVE" : "OFFLINE"), x+18, 126);
    if (f==null) { textSize(16); fill(185); text("Waiting for V1 telemetry", x+18, 170); continue; }
    fill(240); textSize(29); text(states[f.state], x+18, 180);
    textSize(17); fill(190);
    text("Reason: " + reasons[f.reason], x+18, 216);
    text("Match elapsed: " + nf(f.elapsedMs/1000.0, 0, 1) + " s", x+18, 247);
    text("Floor: " + floors[f.floor], x+18, 278);
    text("Front range: " + (f.rangeMm==-1 ? "no target" : f.rangeMm+" mm"), x+18, 309);
    text("Cup capture estimate: " + (f.held==1 ? "LIKELY" : "NO"), x+18, 340);
    text("Motor L / R: " + f.left + " / " + f.right, x+18, 371);
    text("Delivery attempts: " + f.attempts, x+18, 402);
    text("Sensor reads: " + (f.sensorsOK==1 ? "OK" : "INVALID"), x+18, 433);
    text("Packet age: " + (millis()-receivedAt[i]) + " ms", x+18, 464);
    text("Heading: " + (f.heading<0 ? "invalid" : f.heading+" deg"), x+18, 495);
    if (online && f.sensorsOK==1 && f.heading>=0) {
      pushMatrix(); translate(x+315, 530); rotate(radians(f.heading));
      stroke(70, 200, 255); strokeWeight(3); line(0, 25, 0, -25);
      line(0, -25, -9, -10); line(0, -25, 9, -10); popMatrix();
    }
    if (!online) { fill(255, 130, 90); text("STALE VALUES / check radio and ID", x+18, 560); }
  }
}
