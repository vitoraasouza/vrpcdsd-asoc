results="results_GVNS"
exec_dir="../utils"
exec="draw_solution"

instance_sizes="5 7 10 12 15 18 20 22 25 27 30 40 50 75 100 150 200"
instance_class="1 2 3 4 5"

for i in $instance_sizes;
do
    for c in $instance_class;
    do
        for iter in {1..30};
        do
            echo Executando: R1_4_"$c"_"$i"r - iter $iter
            ./"$exec_dir"/"$exec" $results/tikz/R1_4_"$c"_"$i"r_"$iter".tikz $results/tex/R1_4_"$c"_"$i"r_"$iter".tex
            pdflatex -aux-directory=$results/aux -output-directory=$results/PDFs $results/tex/R1_4_"$c"_"$i"r_"$iter".tex > /dev/null 2>&1
            mv $results/PDFs/*.aux $results/PDFs/*.log $results/aux/.
        done
    done
done
