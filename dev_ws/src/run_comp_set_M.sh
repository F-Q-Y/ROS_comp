for m in 1 3 5
do
    for n in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
    do
        colcon build --packages-select cpp_comp_topic
        . install/setup.bash
        echo $m $n | timeout 3s ros2 run cpp_comp_topic topic_comp
    done
done
python3 paintM_comp.py