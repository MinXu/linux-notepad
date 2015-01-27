
LOG=./logfile
echo "cpu status of system and some processes" >> "$LOG"
echo 111
while [ true ];
do
    echo 222
    date >> "${LOG}"
    vmstat 1 1 >> "${LOG}"
    mpstat -P ALL 1 1 >> "${LOG}"
    ps aux| sort -r -k 3 |head -6 >>"${LOG}"
    sleep 10
done
