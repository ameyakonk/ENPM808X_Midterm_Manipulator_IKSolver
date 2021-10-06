#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <bits/stdc++.h>
#include <iostream>

class Kinematics
{
   private:
	   std::vector <double> joint_coordinates;
	   std::vector <double> joint_angles;
   public:
	   std::vector<double> fk_solver(std::vector<double>);
	   std::vector<double> ik_solver(std::vector<double>);
};

std::vector<double> Kinematics::fk_solver(std::vector<double>)
{
	return 0;
}

std::vector<double> Kinematics::ik_solver(std::vector<double>)
{
        return 0;
}

#endif
