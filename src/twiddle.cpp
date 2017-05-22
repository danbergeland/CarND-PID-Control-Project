
#include "twiddle.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>


#define ENABLE_LOG


twiddle::twiddle()
{
  error_episode = 0;
  coeffs_new.clear();
  coeffs_new.resize(3);
  coeffs_best.clear();
  coeffs_best.resize(3);
  //based on nominal values of .2,.004,2
  step_size = {.01,.001,.1};
  update_count  = 0;
  error_best = -1;
  update_pointer = 0;
  up_phase=true;
}

twiddle::~twiddle(){}

/*
 * Tell the twiddler what the PID is.  Used at init.
 */
void twiddle::set_PID_coeffs(double kp, double ki, double kd)
{
  coeffs_new[0] = kp;
  coeffs_new[1] = ki;
  coeffs_new[2] = kd;
  update_count = 0;
  error_episode = 0;
  step_size = {.1*kp,.1*ki,.1*kd};
}

/*
 * Collects error during the episode
 */
void twiddle::updateError(double cte)
{
  error_episode += fabs(cte);
  update_count += 1;
}


/*
 * Get updated PID coeffs based on error up to this point
 */
std::vector<double> twiddle::twiddle_PID_coeffs()
{
  write("twiddle_out.txt");
  //only allow twiddles if a little data has been collected
  if(update_count>5)
  {
    //Check new error vs best error.  If best error is negative, it's not been set yet
    if(error_best<0 || error_episode<error_best)
    {
      //If this run has the best performance
      //save the coeffs
      for(int i=0;i<3;i++)
      {
        coeffs_best[i] = coeffs_new[i];
      }
      //save the error
      error_best = error_episode;
      //increase the size of this step if it decreases overall error
      step_size[update_pointer] *= 1.1;
      //this will cause it to go to the next parameter, if up_phase is true.
      up_phase =false;
    }
    else
    {
      //If this didn't reduce the error, go back to the previous value
      //
      if(up_phase)
      {
        coeffs_new[update_pointer] -= step_size[update_pointer];
      }
      else
      {
        coeffs_new[update_pointer] += step_size[update_pointer];
        //since down phase comes second, make this step smaller if it didn't reduce best error
        step_size[update_pointer] *= .9;
      }

    }
    
    //if on the down phase, go to the up phase on the next parameter
    if(!up_phase)
    {
      update_pointer++;
      if(update_pointer>2) update_pointer=0;
      up_phase=true;
      coeffs_new[update_pointer] += step_size[update_pointer];
    }
    //if on the up phase, go to the down phase
    else
    {
      up_phase=false;
      coeffs_new[update_pointer] -= step_size[update_pointer];
    }

    //initialize new episode
    update_count=0;
    error_episode=0;
    
  }
  return coeffs_new;
}

std::vector<double> twiddle::get_best_PID_coeffs()
{
  return coeffs_best;
}

int twiddle::get_update_count()
{
  return update_count;
}

void twiddle::write(std::string filename) {
  
#ifdef ENABLE_LOG
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
  dataFile<<"New_PID_Params (p,i,d)\n";
	for (int i = 0; i < 3; ++i) {
		dataFile << coeffs_new[i]<< " ";
	}
  dataFile << " Error: " <<error_episode<<"\n";
  dataFile<<"Best_PID_Params\n";
	for (int i = 0; i < 3; ++i) {
		dataFile << coeffs_best[i]<< " ";
	}
  dataFile << " Error: " <<error_best<<"\n";
	dataFile.close();
#endif
}