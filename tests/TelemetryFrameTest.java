public class TelemetryFrameTest {
  static void check(boolean ok) { if (!ok) throw new AssertionError(); }
  public static void main(String[] args) {
    String valid = "V1,1,1234,5,0,1,320,359,1,90,90,2,1\n";
    TelemetryFrame f = TelemetryFrame.parse(valid);
    check(f != null && f.id == 1 && f.state == 5 && f.rangeMm == 320 && f.attempts == 2);
    check(TelemetryFrame.parse("V1,3,0,0,1,0,-1,-1,0,0,0,0,0") != null);
    check(TelemetryFrame.parse(null) == null);
    check(TelemetryFrame.parse(valid.replace("V1", "H")) == null);
    check(TelemetryFrame.parse(valid.replace(",359,", ",NaN,")) == null);
    check(TelemetryFrame.parse(valid.replace(",359,", ",360,")) == null);
    check(TelemetryFrame.parse(valid.replace(",320,", ",0,")) == null);
    check(TelemetryFrame.parse(valid.replace(",90,90,", ",401,90,")) == null);
    check(TelemetryFrame.parse(valid.replace(",90,90,", ",-2147483648,90,")) == null);
    check(TelemetryFrame.parse(valid.replace("V1,1,", "V1,4,")) == null);
    check(TelemetryFrame.parse(valid + ",") == null);
    check(TelemetryFrame.parse("V1,1,1234,5,0,1,320,359,1,90,90,2,") == null);
    System.out.println("PASS: telemetry parsing, bounds, corrupted and incomplete frames");
  }
}
