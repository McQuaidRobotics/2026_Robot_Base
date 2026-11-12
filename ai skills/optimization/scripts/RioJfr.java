import java.io.*;
import java.lang.management.*;
import java.util.*;
import javax.management.remote.*;
import jdk.management.jfr.FlightRecorderMXBean;

/** Record JFR on the robot over JMX and download it: java RioJfr.java host:port seconds out.jfr */
public class RioJfr {
    public static void main(String[] a) throws Exception {
        var url = new JMXServiceURL("service:jmx:rmi:///jndi/rmi://" + a[0] + "/jmxrmi");
        try (var c = JMXConnectorFactory.connect(url)) {
            var fr = ManagementFactory.newPlatformMXBeanProxy(
                    c.getMBeanServerConnection(), "jdk.management.jfr:type=FlightRecorder", FlightRecorderMXBean.class);
            long id = fr.newRecording();
            Map<String, String> s = new HashMap<>(fr.getConfigurations().stream()
                    .filter(x -> x.getName().equals("default")).findFirst().get().getSettings());
            s.put("jdk.ExecutionSample#enabled", "true");
            s.put("jdk.ExecutionSample#period", "5 ms");
            s.put("jdk.NativeMethodSample#enabled", "true");
            s.put("jdk.NativeMethodSample#period", "5 ms");
            s.put("jdk.ObjectAllocationSample#enabled", "true");
            fr.setRecordingSettings(id, s);
            fr.startRecording(id);
            Thread.sleep(Integer.parseInt(a[1]) * 1000L);
            fr.stopRecording(id);
            long stream = fr.openStream(id, Map.of("blockSize", "500000"));
            try (var out = new FileOutputStream(a[2])) {
                byte[] b;
                while ((b = fr.readStream(stream)) != null) out.write(b);
            }
            fr.closeStream(stream);
            fr.closeRecording(id);
            System.out.println("saved " + new File(a[2]).length() + " bytes");
        }
    }
}
