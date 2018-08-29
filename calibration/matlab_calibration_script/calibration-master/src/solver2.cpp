/*!
		\file solver.cpp
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Estimates odometry and sensor parameters
*/

#include <stdio.h> 
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>

#include <gsl/gsl_statistics_double.h>
#include <options/options.h>
#include <csm/csm_all.h>

#include "aux_functions.h"
#include "solver_options.h"
#include "solver_utils.h"

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>


#include "calib_tuple.h"



extern void solver_options(solver_params*p, struct option*ops);
int solve_jackknife(const  std::vector <calib_tuple>& tuples, int mode, double max_cond_number, struct calib_result& res);
int solve(const std::vector <calib_tuple>& tuple, int mode, double max_cond_number, struct calib_result& res);
void estimate_noise(std::vector<calib_tuple>&tuples, const calib_result&res, double&std_x,double&std_y,double&std_th);

    
const char* banner = 
	"This program estimates parameters (odometry and sensor) given a list of tuples generated by synchronizer.\n";

/*!
	Solver routine
	\brief This program has been developed to estimate parameters (odometry and sensor) given a list of tuples generated by synchronizer. 
	
	It uses gsl libraries to handle matrix and vectors, as well as minimization and eigenvalues solution.
	
*/
int main(int argc, const char * argv[]) {
	sm_set_program_name(argv[0]);
	
	/*!<!--###############		Verify options		################-->*/
	solver_params params;	
	struct option* ops = options_allocate(100);
	solver_options(&params, ops);
	options_banner(banner);
	if(!options_parse_args(ops, argc, argv)) {
		options_print_help(ops, stderr);
		return 1;
	}		
	
	if(strlen(params.output_file)==0) {
		char base[PATH_MAX];
		my_no_suffix(params.input_file, base);
        char buf[PATH_MAX];
		sprintf(buf, "%s_results.json", base);
		params.output_file = my_strdup(buf);
		sm_info("No output filename given; writing on file '%s'.\n", params.output_file);
	}

	if(params.outliers_percentage <= 0 || params.outliers_percentage > 0.5) {
		sm_error("Outlier percentage %f does not seem a sane value.\n", params.outliers_percentage);
		return 2;
	}
	
	sm_debug_write(params.debug);


	/*!<!--#################		 Read input		##################-->*/
	vector <JO> jos;
	if (!read_jsonlist(params.input_file, &jos) ) {
		sm_error("Error opening input files.\n");
		return 2;
	}
	
	vector <calib_tuple> tuples;
	for(int i=0;i<jos.size();i++) {
		calib_tuple t;
		if(!json2tuple(jos[i], t)) {
			sm_error("Cannot convert JSON #%d.\n", i);
			return 2;
		}
		tuples.push_back(t);
	}
	
	vector <calib_tuple> original_tuples = tuples;
	
    vector <calib_tuple> tuple_history[params.outliers_iterations+1];
    
	calib_result res;

	for(int iteration=0;iteration<=params.outliers_iterations;iteration++) {
	    tuple_history[iteration] = tuples;
	    
		// Calibration 
		if(!solve(tuples, params.mode, params.max_cond_number, res)) {
			sm_error("Failed calibration.\n");
			return 3;
		}
		
		// Compute residuals
		for(int i=0;i<tuples.size();i++) 
			tuples[i].compute_disagreement(res);
			
		// Sort residuals and compute thresholds
		vector<double> err_theta;
		for(int i=0;i<tuples.size();i++) 
			err_theta.push_back( fabs(tuples[i].err[2]) );
		
		vector<double> err_xy;
		for(int i=0;i<tuples.size();i++) {
			double x = tuples[i].err[0];
			double y = tuples[i].err[1];
			err_xy.push_back( sqrt(x*x + y*y) );
		}
		
		vector<double> err_theta_sorted(err_theta);
		sort( err_theta_sorted.begin(), err_theta_sorted.end() );
		
		vector<double> err_xy_sorted(err_xy);
		sort( err_xy_sorted.begin(), err_xy_sorted.end() );		
		
		int threshold_index = (int) round ( (1-params.outliers_percentage) * tuples.size() );
		if(threshold_index<0) threshold_index = 0;
		if(threshold_index>=tuples.size()) threshold_index = tuples.size()-1;
		
		double threshold_theta = err_theta_sorted[threshold_index];
		double threshold_xy = err_xy_sorted[threshold_index];
		
		int noutliers = 0;
		int noutliers_theta = 0;
		int noutliers_xy = 0;
		int noutliers_both = 0;
		
		for(int i=0;i<tuples.size();i++) {
			int xy = err_xy[i] > threshold_xy;
			int theta = err_theta[i] > threshold_theta;
			
			tuples[i].mark_as_outlier = xy | theta;
			
			if(xy) noutliers_xy ++;
			if(theta) noutliers_theta ++;
			if(xy&&theta) noutliers_both ++;
			if(xy||theta) noutliers ++;
		}

		// Output statistics
		sm_info("Outlier detection: #tuples = %d, removing percentage = %.0f%% (index=%d)\n", tuples.size(),
			100*params.outliers_percentage, threshold_index);
		sm_info("    err_theta  min = %f  max = %f   threshold = %f \n", 
			err_theta[0], err_theta[tuples.size()-1], threshold_theta);
		sm_info("    err_xy  min = %f  max = %f   threshold = %f \n",
			err_xy[0], err_xy[tuples.size()-1], threshold_xy);
		
		sm_info("    num_outliers = %d; for theta = %d; for xy = %d; for both = %d\n", noutliers, noutliers_theta, noutliers_xy, noutliers_both);
		
		
		// Write debug info
		char filename[256];
        char base[255];
        my_no_suffix(params.output_file, base);
        
		sprintf(filename, "%s_iter%d.asc", base, iteration);
		ofstream ofs(filename);
		sm_info("%s\n",filename);
		if(!ofs) sm_error("could not open '%s'\n",filename);
		for(int i=0;i<tuples.size();i++)
			tuples[i].write_as_long_line(ofs);
			
			
		// Write new tuples
		vector<calib_tuple> n;
		for(int i=0;i<tuples.size();i++) 
			if(!tuples[i].mark_as_outlier)
				n.push_back(tuples[i]);

		tuples = n;
	}
	
	// Implemented but useless
	// solve_jackknife(tuples, params.mode, params.max_cond_number,res); 
    
	/* Let's estimate the scan matching noise */
    double laser_std_x, laser_std_y, laser_std_th;
    //estimate_noise(tuples, res, laser_std_x, laser_std_y, laser_std_th);
    int estimate_with = 1;
    estimate_noise(tuple_history[estimate_with], res, laser_std_x, laser_std_y, laser_std_th);
    
//    laser_std_th = deg2rad(1);
  //  laser_std_x = laser_std_y = 0.001;
    
    /* Now compute the FIM */
    sm_info("Using sm errors (std dev):  %g mm, %g mm, %g deg\n", laser_std_x * 1000, laser_std_y * 1000, rad2deg(laser_std_th));
    
    gsl_matrix* laser_fim = gsl_matrix_alloc(3,3);
    gsl_matrix_set_zero(laser_fim);
    gsl_matrix_set(laser_fim,0,0, 1 / (laser_std_x*laser_std_x));
    gsl_matrix_set(laser_fim,1,1, 1 / (laser_std_y*laser_std_y));
    gsl_matrix_set(laser_fim,2,2, 1 / (laser_std_th*laser_std_th));
    
    gsl_matrix* fim = gsl_matrix_alloc(6,6);
    gsl_matrix_set_zero(fim);
    for(int i=0;i<tuples.size();i++) {
        gsl_matrix*fimi = tuples[i].compute_fim(res, laser_fim);
        gsl_matrix_add(fim, fimi);
        gsl_matrix_free(fimi);
    }
    
    /* Inverting the FIM to get covariance */
    gsl_matrix * state_cov = gsl_matrix_alloc(6,6);
    gsl_permutation *perm = gsl_permutation_alloc (6);
    int signum;
    gsl_linalg_LU_decomp (fim, perm, &signum);
    gsl_linalg_LU_invert (fim, perm, state_cov);
    
    sm_info("Uncertainty r_L  = %g mm \n", 1000 * sqrt(gsl_matrix_get(state_cov, 0, 0)));
    sm_info("Uncertainty r_R  = %g mm \n", 1000 * sqrt(gsl_matrix_get(state_cov, 1, 1)));
    sm_info("Uncertainty axle = %g mm \n", 1000 * sqrt(gsl_matrix_get(state_cov, 2, 2)));
    sm_info("Uncertainty l_x  = %g mm \n", 1000 * sqrt(gsl_matrix_get(state_cov, 3, 3)));
    sm_info("Uncertainty l_y  = %g mm \n", 1000 * sqrt(gsl_matrix_get(state_cov, 4, 4)));
    sm_info("Uncertainty l_th = %g deg \n", rad2deg( sqrt(gsl_matrix_get(state_cov, 5, 5))  ));
    
	
	/* Write results and quit */
	JO new_obj = calib_result2json(res)	;
    jo_add(new_obj, "covariance", matrix_to_json(state_cov) );
	
	json_set_float_format("%g");
    
	// Write json object to file
	FILE * output;
	if(!strcmp(params.output_file, "stdout")) { output = stdout;}
	else {output = fopen(params.output_file, "w");}
	fprintf(output, "%s\n", json_object_to_json_string(new_obj) );
	fclose(output);


	fprintf(stderr, "output: %s\n", json_object_to_json_string(new_obj) );

	return 0;
}


