# MIP

## Dependencies

- g++
- make
- [**CPLEX**](https://www.ibm.com/products/ilog-cplex-optimization-studio) (version 20.1.0 or newer)

## Before building the code

Replace the placeholder `/YOUR_CPLEX_INSTALLATION_FOLDER` by your CPLEX installation path in the following files:

- Release/makefile
- Release/src/subdir.mk
- Release/src/Infos/subdir.mk
- Release/src/MathModel/subdir.mk
- Release/src/Persistent/subdir.mk
- Release/src/Utils/subdir.mk

## How to build it?

From the current folder, execute the following commands:

```
cd Release
make
```

The `make` command generates the executable, `MIP`, in the folder `Release`.

## Running instructions

After a successful build, the program needs the following parameters to run properly.

| parameter flag | description                                                                   |
|----------------|-------------------------------------------------------------------------------|
| -i             | (input file) VRPCDSD instance file path                                       |
| -s             | (output file) VRPCDSD solution file path                                      |
| -l             | (output file) CPLEX log file path                                             |
| -tikz          | (output file) draw_solution input file path                                   |
| -p             | ON: uses default CPLEX parameters; OFF: disables CPLEX preprocessing and cuts |
| -th            | Limits the number of threads used by CPLEX (0 = unlimited)                    |
| -t             | Time limit (in seconds)                                                       |
| -c             | Objective function (TCT = Total Completion Time)                              |

Example:

```
./MIP -i ../../problem_instances/R1_4_1_7r.txt -s solution.sol -l cplex.log -tikz solution.tikz -p ON -th 4 -t 3600 -c TCT
```

## Script to run the tests of the paper

If you want to execute the same tests performed in the paper, build the program and run the following command from the current folder:

```
chmod +x run_mip_tests.sh
./run_mip_tests.sh
```

The outputs are written into the folder `results_MIP`. The subfolders `tex`, `PDFs` and `aux` are initially empty; they are filled only after running the draw_solution's script (next step).

## (Optional) Script to draw the solutions

After running the script `run_mip_tests.sh`, you can draw the obtained solutions using the script `draw_solutions.sh`. First, you need to go in the `utils` folder (in the root of this repository) and compile the source code `draw_solution.cpp` using the command:

```
g++ draw_solution.cpp -o draw_solution
```

Then, you have to come back to `MIP` folder and execute the following commands to draw the solutions into the folder `results_MIP/PDFs` (the LaTeX source files are stored in `results_MIP/tex`):

```
chmod +x draw_solutions.sh
./draw_solutions.sh
```

Obs.: The script starts by generating a .tex file from solution's .tikz file. Next, it uses `pdflatex` to generate the solution drawings (.pdf files) from .tex files. If you don't have any LaTeX packages installed on your computer, we recommend you to install the package `texlive-full` (available for Ubuntu Linux).
