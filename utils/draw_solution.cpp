#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>

using namespace std;

#define NODE_SIZE 4.5
#define CD_SIZE 5.2
#define AXIS_LIM 15.0
#define NUM_AXIS_TIKS 10
#define REAL_NODES_LIM 200.0
#define NORMALIZE_FACTOR (REAL_NODES_LIM / AXIS_LIM) 
#define LABEL_STEP (REAL_NODES_LIM / NUM_AXIS_TIKS)
#define AXIS_STEP (AXIS_LIM / NUM_AXIS_TIKS)

#define SCDL_LENGTH 15.0
#define SCDL_LOAD_MIDDLES 0.8
#define SCDL_LOAD_HEIGHT 0.4
#define SCDL_TIKS 5
#define SCDL_AXIS_STEP (SCDL_LENGTH/SCDL_TIKS)

int main (int argc, char * argv[]) {

	ifstream solution;
	solution.open(argv[1]);

	FILE * texfile, * sol;
	texfile = fopen(argv[2], "w");
	
	string trash, instance_name;
	int num_req, num_vhc;
	char aux[500];
	double solution_cost;
	char criterion[100];

	solution >> trash >> instance_name;
	solution >> trash >> num_req;
	solution >> trash >> num_vhc;
	solution >> trash >> solution_cost;
	solution >> trash;
	// getline(solution, criterion); 
	solution.getline(criterion, 100);

	fprintf(texfile, "\\documentclass[10pt]{article}\n");
	fprintf(texfile, "\\usepackage[a4paper, margin=1cm]{geometry}\n");
	fprintf(texfile, "\\usepackage{caption}\n");
	fprintf(texfile, "\\usepackage{tikz}\n");
	fprintf(texfile, "\\usetikzlibrary{arrows, arrows.meta, backgrounds, positioning, fit, patterns, patterns.meta, shapes}\n");
	fprintf(texfile, "\\pagestyle{empty}\n");
	fprintf(texfile, "\\usepackage[utf8]{inputenc}\n\n");

	fprintf(texfile, "\\tikzset{pickup/.style={regular polygon,regular polygon sides=6, draw=blue!70, fill=blue!20, thick, inner sep=0pt, minimum size=%lfmm}}\n", NODE_SIZE);
	fprintf(texfile, "\\tikzset{delivery/.style={circle, draw=green!70, fill=green!20, thick, inner sep=0pt, minimum size=%lfmm}}\n", NODE_SIZE);
	fprintf(texfile, "\\tikzset{cross-dock/.style={rectangle, draw=red!70, fill=red!20, thick, inner sep=0pt, minimum size=%lfmm}}\n\n", CD_SIZE);

	fprintf(texfile, "\\tikzset{travel/.style={rectangle, pattern=crosshatch, pattern color=black!50}}\n");
	fprintf(texfile, "\\tikzset{preparation/.style={rectangle, draw=black!50, fill=black!20, pattern=horizontal lines light gray, thick}}\n");
	fprintf(texfile, "\\tikzset{unload/.style={rectangle, draw=blue!70, fill=blue!20, thick, densely dashed}}\n");
	fprintf(texfile, "\\tikzset{reload/.style={rectangle, draw=green!70, fill=green!20, thick}}\n\n");

	fprintf(texfile, "\\begin{document}\n\n");
	fprintf(texfile, "   \\begin{figure}\n");
	fprintf(texfile, "   \\begin{tikzpicture}[every node/.style={font=\\footnotesize}]\n");

	fprintf(texfile, "      \\draw[step=%.1lfcm, color=black!10, dashed] (0, 0) grid (%.1lf, %.1lf);\n", AXIS_STEP, AXIS_LIM + 0.9, AXIS_LIM + 0.9);
    fprintf(texfile, "      \\draw[->, >= latex] (-1,0) -- (%d,0);\n", (int)AXIS_LIM + 1);
    fprintf(texfile, "      \\node[below] at (%.1lf,0) {$x(km)$};\n", AXIS_LIM + 1.4);
    fprintf(texfile, "      \\draw[->, >= latex] (0,-1) -- (0,%d) node[left]{$y(km)$};\n", (int)AXIS_LIM + 1);
    fprintf(texfile, "      \\draw (0,0) node[below left]{0};\n");

    sprintf(aux, "      \\foreach \\x/\\xtext in {%.1lf/%d", AXIS_STEP, (int)LABEL_STEP);
    for (int x = 2, xtext = 2 * LABEL_STEP; x <= NUM_AXIS_TIKS; ++x, xtext += LABEL_STEP) {
    	sprintf(aux + strlen(aux), ", %.1lf/%d", x * AXIS_STEP, xtext);
    }
    sprintf(aux + strlen(aux), "} {\n");    

	fprintf(texfile, "%s", aux);
    fprintf(texfile, "         \\draw (\\x cm, -1pt) -- (\\x cm, 1pt);\n");
    fprintf(texfile, "         \\draw (\\x cm, 0) node[below]{\\xtext};\n");
   	fprintf(texfile, "      }\n");

    sprintf(aux, "      \\foreach \\y/\\ytext in {%.1lf/%d", AXIS_STEP, (int)LABEL_STEP);
    for (int y = 2, ytext = 2 * LABEL_STEP; y <= NUM_AXIS_TIKS; ++y, ytext += LABEL_STEP) {
    	sprintf(aux + strlen(aux), ", %.1lf/%d", y * AXIS_STEP, ytext);
    }
    sprintf(aux + strlen(aux), "} {\n");

   	fprintf(texfile, "%s", aux);
    fprintf(texfile, "         \\draw (-1 pt, \\y cm) -- (1 pt, \\y cm);\n");
    fprintf(texfile, "         \\draw (0, \\y cm) node[left]{\\ytext};\n");
   	fprintf(texfile, "      }\n");

    double x, y;

    solution >> trash >> trash >> x >> y;
    fprintf(texfile, "      \\node[cross-dock] (cd) at (%.1lf, %.1lf) {DS};\n", x/NORMALIZE_FACTOR, y/NORMALIZE_FACTOR);

    for (int i = 1; i <= num_req; ++i) {
    	solution >> trash >> x >> y;
		fprintf(texfile, "      \\node[pickup] (p%d) at (%.1lf, %.1lf) {%d};\n", i, x/NORMALIZE_FACTOR, y/NORMALIZE_FACTOR, i);
		solution >> x >> y >> trash;
		fprintf(texfile, "      \\node[delivery] (d%d) at (%.1lf, %.1lf) {%d};\n", i, x/NORMALIZE_FACTOR, y/NORMALIZE_FACTOR, i);
    }    

	string current_node, previous_node;

	solution >> trash;
	for (int k = 0; k < num_vhc; ++k) {
		solution >> trash >> trash >> trash >> trash >> trash >> trash;
		previous_node = "cd";
		do {
			solution >> current_node;
			if (previous_node == "cd") { 
				fprintf(texfile, "      \\draw [thick] (cd) to (p%s);\n", current_node.c_str());
			} else if (current_node == "0)") {
				fprintf(texfile, "      \\draw [thick] (p%s) to (cd);\n", previous_node.c_str());
			} else {
				fprintf(texfile, "      \\draw [thick] (p%s) to (p%s);\n", previous_node.c_str(), current_node.c_str());
			}
			previous_node = current_node;
		} while (current_node != "0)");
	}

	solution >> trash;
	for (int k = 0; k < num_vhc; ++k) {
		solution >> trash >> trash >> trash >> trash >> trash >> trash;
		previous_node = "cd";
		do {
			solution >> current_node;
			if (previous_node == "cd") { 
				fprintf(texfile, "      \\draw [thick] (cd) to (d%s);\n", current_node.c_str());
			} else if (current_node == "0)") {
				fprintf(texfile, "      \\draw [thick] (d%s) to (cd);\n", previous_node.c_str());
			} else {
				fprintf(texfile, "      \\draw [thick] (d%s) to (d%s);\n", previous_node.c_str(), current_node.c_str());
			}
			previous_node = current_node;
		} while (current_node != "0)");
	}

	fprintf(texfile, "   \\end{tikzpicture}\n");
	fprintf(texfile, "   \\caption*{Routing part. Instance with %d requests. Pickup + delivery + %s = %.3lf (min)}\n", num_req, criterion, solution_cost);
	fprintf(texfile, "   \\end{figure}\n");
	// fprintf(texfile, "   ~\\\\ ~\\\\\n");

	while (trash != "START_SCHEDULING:") {
		solution >> trash;
	}

	double start_scdl, makespan, cd_window;
	solution >> start_scdl >> trash >> makespan >> trash;

	cd_window = ceil(makespan) - floor(start_scdl);

	fprintf(texfile, "   \\begin{figure}\n");
	fprintf(texfile, "   \\begin{tikzpicture}[every node/.style={font=\\footnotesize}]\n");

	sprintf(aux, "      \\foreach \\y/\\k in {%.1lf/%d", num_vhc * SCDL_LOAD_MIDDLES, 1);
	for (int i = 1; i < num_vhc; ++i) {
		sprintf(aux + strlen(aux), ", %.1lf/%d", (num_vhc - i) * SCDL_LOAD_MIDDLES, i + 1);
	}
	sprintf(aux + strlen(aux), "} {\n");

	fprintf(texfile, "%s", aux);
    fprintf(texfile, "         \\draw[color=black!10, dashed] (0,\\y) -- (%.1lf,\\y);\n", SCDL_LENGTH + 0.9);
    fprintf(texfile, "         \\draw (0,\\y) node[left]{\\k};\n");
    fprintf(texfile, "      }\n\n");

    fprintf(texfile, "      \\draw (0,0) node[below]{%.1lf};\n", floor(start_scdl));

    double scdl_label_step = cd_window / SCDL_TIKS;
    sprintf(aux, "      \\foreach \\x/\\xtext in {%.1lf/%.1lf", SCDL_AXIS_STEP, floor(start_scdl) + scdl_label_step);
	for (int x = 2; x <= SCDL_TIKS; ++x) {
		sprintf(aux + strlen(aux), ", %.1lf/%.1lf", x * SCDL_AXIS_STEP, floor(start_scdl) + x * scdl_label_step);
	}
	sprintf(aux + strlen(aux), "} {\n");	

	fprintf(texfile, "%s", aux);
    fprintf(texfile, "         \\draw[color=black!20] (\\x,0) -- (\\x,%.1lf);\n", (num_vhc + 0.9) * SCDL_LOAD_MIDDLES);
    fprintf(texfile, "         \\draw (\\x,-1pt) -- (\\x,1pt);\n");
    fprintf(texfile, "         \\draw (\\x,0) node[below]{\\xtext};\n");
    fprintf(texfile, "      }\n\n");
	
	char type;
	int vhc, req;
	double start_load, proc_time;
	while (solution >> type) {
		if (type == 'U') {
			solution >> vhc >> req >> start_load >> proc_time;
	   		fprintf(texfile, "      \\draw[unload] (%.1lf,%.1lf) rectangle node{%d} (%.1lf, %.1lf);\n", SCDL_LENGTH * (start_load - start_scdl)/cd_window, 
	   			(double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES - SCDL_LOAD_HEIGHT / 2, req, SCDL_LENGTH * (start_load + proc_time - start_scdl)/cd_window, (double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES + SCDL_LOAD_HEIGHT / 2);
	   	} else if (type == 'R') {
	   		solution >> vhc >> req >> start_load >> proc_time;
	   		fprintf(texfile, "      \\draw[reload] (%.1lf,%.1lf) rectangle node{%d} (%.1lf, %.1lf);\n", SCDL_LENGTH * (start_load - start_scdl)/cd_window, 
	   			(double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES - SCDL_LOAD_HEIGHT / 2, req, SCDL_LENGTH * (start_load + proc_time - start_scdl)/cd_window, (double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES + SCDL_LOAD_HEIGHT / 2);
	   	} else if (type == 'P') {
	   		solution >> vhc >> start_load >> proc_time;
	   		fprintf(texfile, "      \\draw[preparation] (%.1lf,%.1lf) rectangle (%.1lf, %.1lf);\n", SCDL_LENGTH * (start_load - start_scdl)/cd_window, 
	   			(double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES - SCDL_LOAD_HEIGHT / 2, SCDL_LENGTH * (start_load + proc_time - start_scdl)/cd_window, (double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES + SCDL_LOAD_HEIGHT / 2);
	   	} else if (type == 'T') {
	   		solution >> vhc >> start_load >> proc_time;
	   		fprintf(texfile, "      \\fill[travel] (%.1lf,%.1lf) rectangle (%.1lf, %.1lf);\n", SCDL_LENGTH * (start_load - start_scdl)/cd_window, 
	   			(double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES - 0.85 * SCDL_LOAD_HEIGHT / 2, SCDL_LENGTH * (start_load + proc_time - start_scdl)/cd_window, (double)(num_vhc - vhc + 1) * SCDL_LOAD_MIDDLES + 0.85 * SCDL_LOAD_HEIGHT / 2);
	   	}
   	}
   
   	fprintf(texfile, "\n      \\draw[->, >= latex] (0,0) -- (%.1lf,0);\n", SCDL_LENGTH + 1.0);
   	fprintf(texfile, "      \\node[below] at (%.1lf,0) {$t(min)$};\n", SCDL_LENGTH + 1.4);
   	fprintf(texfile, "      \\draw[-, >= latex] (0,0) -- (0,%.1lf) node[left]{$k$};\n", (num_vhc + 1.0) * SCDL_LOAD_MIDDLES);

	fprintf(texfile, "   \\end{tikzpicture}\n");
	fprintf(texfile, "   \\caption*{Scheduling part. Instance with %d requests. Pickup + delivery + %s = %.3lf (min)}\n", num_req, criterion, solution_cost);
	fprintf(texfile, "   \\end{figure}\n");
   	fprintf(texfile, "\\end{document}\n");

   	fclose(texfile);
	solution.close();

	return 0;
}
