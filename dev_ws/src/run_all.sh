if [ $#==0 ]
then
    bash run_ours_set_M.sh
    bash run_ours_set_N.sh
    bash run_comp_set_M.sh
    bash run_comp_set_N.sh
elif [ $#==1 ]
then
    if [$1 == "ours"]
    then
        bash run_ours_set_M.sh
        bash run_ours_set_N.sh
    elif [$1 == "comp"]
    then
        bash run_comp_set_M.sh
        bash run_comp_set_N.sh
    elif [$1 == "M"]
    then
        bash run_comp_set_M.sh
        bash run_ours_set_M.sh
    elif [$1 == "N"]
    then
        bash run_comp_set_N.sh
        bash run_ours_set_N.sh
    else
        echo "wrong param 1"
    fi
else
    echo "wrong param num"
fi