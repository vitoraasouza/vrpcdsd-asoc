instances_dir="../problem_instances"
exec_dir="Release"
exec="GVNS"

instance_sizes="5 7 10 12 15 18 20 22 25 27 30 40 50 75 100 150 200"
instance_classes="1 2 3 4 5"

exec_time=3600
strats="best"

results=results_"$exec"
mkdir $results
mkdir $results/solutions
mkdir $results/logs
mkdir $results/tikz
mkdir $results/tex
mkdir $results/PDFs
mkdir $results/aux

echo "instance;solution;time_to_best(s);time(s);seed" >> $results/compilation_"$exec".csv

for i in $instance_sizes;
do
    for c in $instance_classes;
    do
        for iter in {1..30};
        do
            echo Executando: R1_4_"$c"_"$i"r - iter $iter
            ./"$exec_dir"/"$exec" -t $exec_time -i $instances_dir/R1_4_"$c"_"$i"r.txt -l $results/logs/R1_4_"$c"_"$i"r_"$iter".log -s $results/solutions/R1_4_"$c"_"$i"r_"$iter".sol -tikz $results/tikz/R1_4_"$c"_"$i"r_"$iter".tikz -strat best >> $results/compilation_"$exec".csv
        done
    done
done
