colcon build --packages-select cpp_smart_topic
. install/setup.bash
for n in 1 3 5
do
    for m in 1 2 3 4 5 6 7 8 9 10 11 12
    do
        echo $m $n | timeout 5s ros2 run cpp_smart_topic topic_smart
    done
done
python3 paintN_smart.py