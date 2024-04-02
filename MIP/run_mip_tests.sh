instances="../problem_instances"
exec_dir="Release"
exec="MIP"

instance_sizes="5 7 10 12 15 18 20 22 25 27 30"
instance_class="1 2 3 4 5"

exec_time=10800
num_threads=4
parameters="ON"

results=results_"$exec"
mkdir $results
mkdir $results/solutions
mkdir $results/cplex_logs
mkdir $results/tikz
mkdir $results/tex
mkdir $results/PDFs
mkdir $results/aux

echo "instance;objective;threads;root_LB;root_status;root_time(s);BB_LB;BB_UB;BB_status;gap;total_time(s);num_BB_nodes;vehicle_swaps" >> $results/compilation_"$exec".csv

for i in $instance_sizes;
do
    for c in $instance_class;
    do
        echo Executando: R1_4_"$c"_"$i"r
        ./"$exec_dir"/"$exec" -t $exec_time -p $parameters -th $num_threads -i $instances/R1_4_"$c"_"$i"r.txt -l $results/cplex_logs/R1_4_"$c"_"$i"r.log -s $results/solutions/R1_4_"$c"_"$i"r.sol -tikz $results/tikz/R1_4_"$c"_"$i"r.tikz -c TCT >> $results/compilation_"$exec".csv
    done
done
