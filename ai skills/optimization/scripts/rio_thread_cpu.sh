#!/bin/sh
# Per-OS-thread CPU (user/sys) of the robot JVM over N seconds. Run ON the roboRIO:
#   ssh admin@10.TE.AM.2 'sh -s' < rio_thread_cpu.sh
# Java thread names are not visible here (all show as "java"); use RioThreads.java over JMX
# for names. This view is still useful: it shows native threads (NetworkTables, CTRE) and
# whether time is user (Java/compute) or sys (syscalls: CAN, network, NT).
SECS=${SECS:-5}
P=$(pidof java | tr " " "\n" | head -1)
[ -z "$P" ] && { echo "no java process"; exit 1; }
snap(){ for t in /proc/$P/task/*; do id=${t##*/}; v=$(sed "s/.*) //" $t/stat | awk '{print $12, $13}'); echo "$id $v"; done; }
snap > /tmp/cpu_a; sleep $SECS; snap > /tmp/cpu_b
echo "tid user% sys% wchan"
awk -v s=$SECS 'NR==FNR{u[$1]=$2;y[$1]=$3;next}{du=($2-u[$1])/s; ds=($3-y[$1])/s; if(du+ds>2) print $1, du, ds}' /tmp/cpu_a /tmp/cpu_b \
  | sort -k2 -rn | while read id u s; do echo "$id $u $s $(cat /proc/$P/task/$id/wchan 2>/dev/null)"; done
echo; top -bn1 | head -4
echo; free -m | head -2
