#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <string>

class twiddle{
public:

  
  twiddle();
  
  virtual ~twiddle();
  
  /*
   * Tell the twiddler what the PID is.  Used at init.
   */
  void set_PID_coeffs(double kp, double ki, double kd);
  
  /*
   * Collects error during the episode
   */
  void updateError(double cte);
  
  
  /*
   * Get updated PID coeffs based on error up to this point
   */
  std::vector<double> twiddle_PID_coeffs();
  
  std::vector<double> get_best_PID_coeffs();
  
  int get_update_count();
  
private:
  /*
   * Accumulate error during the episode to compare
   */ 
  double error_episode;
  
  double error_best;
   /*
   * PID parameters to tune
   */
  std::vector<double> coeffs_new;
  //Retain the best coeffs while trying new ones
  std::vector<double> coeffs_best;
  
  //These set how large the steps are for each param
  std::vector<double> step_size;
  /*
   * Keep track of how many times the update has been called
   */
  int update_count;
  
  //this pointer keeps track of which coefficient to update
  int update_pointer;
  
  //the routine goes up first, then down.  When this is true, it's the first time through for the param.
  bool up_phase;
  
  void write(std::string filename);
};


#endif