for n in 1 3 5
do
    for m in 1 2 3 4 5 6 7 8 9 10 11 12
    do
        colcon build --packages-select cpp_mail_topic
        . install/setup.bash
        echo $m $n | timeout 5s ros2 run cpp_mail_topic topic_mail
    done
done
python3 paintN_mail.py