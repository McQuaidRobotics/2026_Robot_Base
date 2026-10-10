import java.lang.management.*;
import java.util.*;
import javax.management.*;
import javax.management.remote.*;

/** Per-thread CPU + GC over JMX: java RioThreads.java 10.99.99.2:1198 [seconds] */
public class RioThreads {
    public static void main(String[] a) throws Exception {
        var url = new JMXServiceURL("service:jmx:rmi:///jndi/rmi://" + a[0] + "/jmxrmi");
        int secs = a.length > 1 ? Integer.parseInt(a[1]) : 10;
        try (var c = JMXConnectorFactory.connect(url)) {
            var mbs = c.getMBeanServerConnection();
            var th = ManagementFactory.newPlatformMXBeanProxy(mbs, ManagementFactory.THREAD_MXBEAN_NAME, ThreadMXBean.class);
            var gcs = ManagementFactory.getPlatformMXBeans(mbs, GarbageCollectorMXBean.class);
            var mem = ManagementFactory.newPlatformMXBeanProxy(mbs, ManagementFactory.MEMORY_MXBEAN_NAME, MemoryMXBean.class);
            Map<Long, Long> t0 = new HashMap<>();
            for (long id : th.getAllThreadIds()) t0.put(id, th.getThreadCpuTime(id));
            long gc0 = 0, gcn0 = 0;
            for (var g : gcs) { gc0 += g.getCollectionTime(); gcn0 += g.getCollectionCount(); }
            Thread.sleep(secs * 1000L);
            List<String> rows = new ArrayList<>();
            for (long id : th.getAllThreadIds()) {
                long d = th.getThreadCpuTime(id) - t0.getOrDefault(id, 0L);
                var info = th.getThreadInfo(id, 8);
                if (info == null) continue;
                StringBuilder sb = new StringBuilder();
                for (var f : info.getStackTrace()) sb.append("\n      at ").append(f);
                rows.add(String.format("%08.1f%% %s [%s]%s", d / 1e7 / secs, info.getThreadName(), info.getThreadState(), sb));
            }
            rows.sort(Comparator.reverseOrder());
            rows.stream().limit(15).forEach(System.out::println);
            long gc1 = 0, gcn1 = 0;
            for (var g : gcs) { gc1 += g.getCollectionTime(); gcn1 += g.getCollectionCount(); System.out.println("GC " + g.getName() + " total=" + g.getCollectionCount() + " " + g.getCollectionTime() + "ms"); }
            System.out.printf("GC in window: %d collections, %d ms (%.1f%% of wall)%n", gcn1 - gcn0, gc1 - gc0, (gc1 - gc0) / 10.0 / secs);
            System.out.println("Heap: " + mem.getHeapMemoryUsage());
        }
    }
}
