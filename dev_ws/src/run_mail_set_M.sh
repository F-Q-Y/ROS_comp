colcon build --packages-select cpp_mail_topic
. install/setup.bash
for m in 1 3 5
do
    for n in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
    do
        echo $m $n | timeout 5s ros2 run cpp_mail_topic topic_mail
    done
done
python3 paintM_mail.py