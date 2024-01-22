legs=("FR" "FL" "RR" "RL")
joints=("hip" "thigh" "calf")
damages=("blocked_joint" "free_joint")
folders=("FootCalfKnee" "Foot" "FootCalf" "FootCalfThigh")


path="/git/limbo/exp/fyp-a-1-unidirectional-adaptation/Data"


for f in ${folders[@]}; do 
        bd=`cat $path/$f/BD`
        echo $bd
        for d in $path/$f/*/; do 
                archive=$d/archive* 
                basename "$archive"
                archive_name="$(basename -- $archive)"
                echo $archive
                mkdir $d"Videos/"

                for leg in ${legs[@]}; do
                        mkdir $d"Videos/"${leg}
                        for joint in ${joints[@]}; do
                                mkdir $d"Videos"/${leg}/${joint}
                                for damage in ${damages[@]}; do
                                        dmg=${leg}"_"${joint}"_joint "${damage}
                                        dmg_name=${leg}"_"${joint}"_joint_"${damage}
                                        echo ""
                                        echo "Damage:                 "$dmg
                                        echo "Archive_name:           "$archive
                                        ./build/exp/fyp-a-1-unidirectional-adaptation/experiment_adapt_${bd} --load $archive --damages $dmg  2>/dev/null 1>/dev/null
                                        index=$?
                                        echo "Compensatory Behaviour: "$index
                                        mv texel02* $dmg_name
                                        mv $dmg_name* $d"Videos/"${leg}/${joint}
                                done
                        done
                done
                mv Results $d
        done 
done 
