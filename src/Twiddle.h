#ifndef Twiddle
#define Twiddle

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>


  /*
  * Twiddle
  */
  std::vector<double> p = {0.1,0.0,0.005};
  std::vector<double> dp = {.001,.001,.001};
  int twiddle_case = 0;
  int twiddle_index = 0;
  double best_err = 1000000;

  void PrintResults()
  {
	std::cout << "P = [ " << p[0] <<","<< p[1] << "," << p[2] << "]" << std::endl;
	std::cout << "DP = [ " << dp[0] <<","<< dp[1] << "," << dp[2] << "]" << std::endl;
	std::cout << "Best Error " << best_err << std::endl;
  }
  
  double GetBestError()
  {
	return best_err;
  }

  /*
  * Update the Twiddle and the pid parameters
  */
  PID UpdateTwiddle(PID pid, double err)
  {
     if(twiddle_case == 0)
     {
	best_err = err;
	p[twiddle_index] += dp[twiddle_index];
	twiddle_case++;
	pid.Init(p[0],p[1],p[2]);
     }
     else if(twiddle_case == 1)
     {
	if(err < best_err)
	{
		best_err = err;
		dp[twiddle_index] *= 1.1;
		twiddle_index = (twiddle_index+1)%3;
		p[twiddle_index] += dp[twiddle_index];
		twiddle_case = 1;
		pid.Init(p[0],p[1],p[2]);
	}
	else
	{
		p[twiddle_index]-= 2*dp[twiddle_index];
		twiddle_case = 2;
		pid.Init(p[0],p[1],p[2]);
	}
     }
     else
     {
	if(err < best_err)
	{
		best_err = err;
		dp[twiddle_index] *= 1.1;
		twiddle_index = (twiddle_index+1)%3;
		p[twiddle_index]+=dp[twiddle_index];
		twiddle_case = 1;
		pid.Init(p[0],p[1],p[2]);
	}
	else
	{
		p[twiddle_index]+=dp[twiddle_index];
		dp[twiddle_index]*=0.9;
		twiddle_index = (1+twiddle_index)%3;
		p[twiddle_index]+=dp[twiddle_index];
		twiddle_case=1;
		pid.Init(p[0],p[1],p[2]);
	}
    }
    return pid;
  }

#endif /* Twiddle */
