#!/bin/sh
# CPU of EVERY thread on the roboRIO (all processes, kernel threads, IRQs) over N seconds.
# Run ON the roboRIO:   ssh admin@10.TE.AM.2 'sh -s' < rio_all_threads.sh
# Percent = % of ONE core (2 cores = 200% max). Uses one awk per snapshot so sampling itself is
# cheap and the window is timed with /proc/uptime (a slow fork-per-thread loop skews results).
# Native threads in the JVM all show as "java"; map them to Java names with
#   jfr print --events jdk.ThreadDump rec.jfr   (nid=0x... is the OS tid)
SECS=${SECS:-10}
HZ=$(getconf CLK_TCK 2>/dev/null || echo 100)
snap(){ awk '{ s=$0; sub(/^[0-9]+ \(/,"",s); name=s; sub(/\) .*/,"",name); sub(/^.*\) /,"",s);
  split(s,f," "); split(FILENAME,p,"/"); print p[3], p[5], f[12]+f[13], name }' \
  /proc/[0-9]*/task/[0-9]*/stat 2>/dev/null; }
read t0 _ < /proc/uptime; snap > /tmp/at_a; head -1 /proc/stat > /tmp/at_sa
sleep $SECS
read t1 _ < /proc/uptime; snap > /tmp/at_b; head -1 /proc/stat > /tmp/at_sb
el=$(awk -v a=$t0 -v b=$t1 'BEGIN{print b-a}')
echo "window ${el}s, HZ=$HZ"
cat /tmp/at_sa /tmp/at_sb | awk -v hz=$HZ -v el=$el 'NR==1{for(i=2;i<=8;i++)a[i]=$i;next}
  {for(i=2;i<=8;i++)d[i]=$i-a[i]; t=0; for(i=2;i<=8;i++)t+=d[i];
   printf "whole CPU: user %.0f%%  sys %.0f%%  softirq %.0f%%  idle %.0f%%  (of %.1f cores)\n",
   100*d[2]/t, 100*d[4]/t, 100*d[8]/t, 100*d[5]/t, t/hz/el}'
echo " %core   pid   tid name"
awk -v hz=$HZ -v el=$el 'NR==FNR{a[$2]=$3;next} ($2 in a){d=$3-a[$2]; if(d>0){n=$4; for(i=5;i<=NF;i++)n=n" "$i;
  printf "%6.1f %5s %5s %s\n", 100*d/hz/el, $1, $2, n}}' /tmp/at_a /tmp/at_b | sort -rn | head -${TOP:-25}
echo; echo "pid -> command:"
for p in $(awk 'NR==FNR{a[$2]=$3;next} ($2 in a)&&$3-a[$2]>0{print $1}' /tmp/at_a /tmp/at_b | sort -u); do
  c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | cut -c1-70); [ -n "$c" ] && printf "%5s %s\n" $p "$c"; done
