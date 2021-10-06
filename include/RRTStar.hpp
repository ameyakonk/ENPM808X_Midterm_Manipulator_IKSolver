#ifndef RRT_STAR_H
#define RRT_STAR_H

#include<iostream>
#include<bits/stdc++.h>

class RRTStar {
   private:
           std::vector <double> initial_node;
           std::vector <double> final_node;
	   double max_distance;
	   double node_step_size;
	   double n_iters;
	   double goal_bias;
	   double last_goal;
	   double overall_path;
   public:
           void set_initial_node(double);
	   std::vector <double> get_initial_node();
	   double get_distance(double, double);
	   void set_n_iters(int n);
	   void generate_random_node();
	   void cal_no_nodes();
	   void get_nearest_nodes();
	   void search_best_goal();
	   void check_collisions();
	   void rewire_goal();
	   void choose_parent();
	   void extend_closest_node();
	   void smooth();
           void ik_solver(std::vector<double>);
};


void RRTStar::set_initial_node(double _n)
{
        n_iters = _n;
}

std::vector <double> RRTStar:: get_initial_node() {
	std::vector <double> temp;
	return temp;
}

double RRTStar::get_distance(double p1, double p2) {
	return 0;
}

void RRTStar::set_n_iters(int n) {}
void RRTStar::generate_random_node() {}
void RRTStar::cal_no_nodes() {}
void RRTStar::get_nearest_nodes() {}
void RRTStar::search_best_goal() {}
void RRTStar::check_collisions() {}
void RRTStar::rewire_goal() {}
void RRTStar::choose_parent() {}
void RRTStar::extend_closest_node() {}
void RRTStar::smooth() {}

#endif

