# GVNS

## Dependencies

- g++
- make

## How to build it?

From the current folder, execute the following commands:

```
cd Release
make
```

The `make` command generates the executable, `GVNS`, in the folder `Release`.

## Running instructions

After a successful build, the program needs the following parameters to run properly.

| parameter flag | description                                                    |
|----------------|----------------------------------------------------------------|
| -i             | (input file) VRPCDSD instance file path                        |
| -s             | (output file) VRPCDSD solution file path                       |
| -l             | (output file) GVNS log file path                               |
| -tikz          | (output file) draw_solution input file path                    |
| -t             | Time limit (in seconds)                                        |
| -strat         | Neighbor selection strategy (best = best improvement strategy) |

Example:

```
./GVNS -i ../../problem_instances/R1_4_1_7r.txt -s solution.sol -l gvns.log -tikz solution.tikz -t 3600 -strat best
```

## Script to run the tests of the paper

If you want to execute the same tests performed in the paper, build the program and run the following command from the current folder:

```
chmod +x run_gvns_tests.sh
./run_gvns_tests.sh
```

The outputs are written into the folder `results_GVNS`. The subfolders `tex`, `PDFs` and `aux` are initially empty; they are filled only after running the draw_solution's script (next step).

## (Optional) Script to draw the solutions

After running the script `run_gvns_tests.sh`, you can draw the obtained solutions using the script `draw_solutions.sh`. First, you need to go in the `utils` folder (in the root of this repository) and compile the source code `draw_solution.cpp` using the command:

```
g++ draw_solution.cpp -o draw_solution
```

Then, you have to come back to `GVNS` folder and execute the following commands to draw the solutions into the folder `results_GVNS/PDFs` (the LaTeX source files are stored in `results_GVNS/tex`):

```
chmod +x draw_solutions.sh
./draw_solutions.sh
```

Obs.: The script starts by generating a .tex file from solution's .tikz file. Next, it uses `pdflatex` to generate the solution drawings (.pdf files) from .tex files. If you don't have any LaTeX packages installed on your computer, we recommend you to install the package `texlive-full` (available for Ubuntu Linux).
