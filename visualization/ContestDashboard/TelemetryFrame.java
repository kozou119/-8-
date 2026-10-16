// Shared parser used by Processing and the host validation test.
public final class TelemetryFrame {
  public final int id, elapsedMs, state, reason, floor, rangeMm, heading, held, left, right, attempts, sensorsOK;
  private TelemetryFrame(int[] v) {
    id=v[0]; elapsedMs=v[1]; state=v[2]; reason=v[3]; floor=v[4]; rangeMm=v[5];
    heading=v[6]; held=v[7]; left=v[8]; right=v[9]; attempts=v[10]; sensorsOK=v[11];
  }
  public static TelemetryFrame parse(String line) {
    if (line == null || line.length() > 100) return null;
    String[] parts = line.trim().split(",", -1);
    if (parts.length != 13 || !parts[0].equals("V1")) return null;
    int[] v = new int[12];
    try {
      for (int i=0; i<v.length; ++i) v[i] = Integer.parseInt(parts[i+1]);
    } catch (NumberFormatException e) { return null; }
    if (v[0]<1 || v[0]>3 || v[1]<0 || v[2]<0 || v[2]>12 || v[3]<0 || v[3]>8 ||
        v[4]<0 || v[4]>4 || (v[5]!=-1 && (v[5]<1 || v[5]>5000)) ||
        v[6]<-1 || v[6]>359 || v[7]<0 || v[7]>1 || Math.abs((long)v[8])>400 ||
        Math.abs((long)v[9])>400 || v[10]<0 || v[10]>65535 || v[11]<0 || v[11]>1) return null;
    return new TelemetryFrame(v);
  }
}
