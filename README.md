# Build the container 

1. Clone the repo with the --recurse-submodules option 

2. In the singularity directory, run ./start_container

## Run the MAP Elites Experiment 

1. In /git/sferes run ./setup.sh

2. depending on which BD, fitness function and target velocity you want to use run one of the binaries in ./build/exp/fyp-a1-unidirectional-adaptation/... 

## Run the Adaptation Experiment 

1. First, run mv /git/limbo2/setup.sh /git/limbo 

2. In /git/limbo, run ./setup.sh 

3. Then, depending on the archive BD, run 

./build/exp/a1_adapt_<graphic/video/ or neither>_d_<BD_size> --load <archive_path> --damages <joint> <damage type> 2>/dev/null 

## View individual Behaviours 

1. The limbo experiment has to be compiled, use step 1 and 2 of the previous section 

2. In /git/limbo, run 

./build/exp/fyp-a1-unidirectional-adaptation/playback_adapt_graphic_d_4 --load <archive> --index <indiv index> --damages <optional>




