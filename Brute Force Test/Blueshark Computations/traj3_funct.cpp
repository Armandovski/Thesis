#include <iostream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <mpi.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>

using namespace std;
using namespace boost::numeric::odeint;

// Target Final State Conditions
const double tof = 150.0;
const double pi = 4.0*atan(1.0);
const double theta_target = 0.0; // [rad]
const double dockLength = 0.4; // [m]
const double rotRate_target = 1.0; // [rpm]
const double angVel_target = rotRate_target*2.0*pi/60.0; // [rad/s]
const double r_target1 = dockLength*cos(angVel_target*tof + pi/2.0);
const double r_target2 = dockLength*sin(angVel_target*tof + pi/2.0); // [m]
const double rnorm = sqrt(r_target1*r_target1+r_target2*r_target2);
const double v_target1 = angVel_target*rnorm*cos(angVel_target*tof + pi);
const double v_target2 = angVel_target*rnorm*sin(angVel_target*tof + pi); 
//% [m/s]

// Hardcode waypoint time for now
const double tof_waypoint = 0.1*tof;

// define integrator and waypoint function domain types
typedef std::array<double,5> state_type;
typedef std::array<double,4> waypoint;


// declare waypoint variables used in cost function as global
// but don't fix their values
waypoint w_val;

// set up generators for random number functions
typedef boost::mt19937 generator_type;

// begin functions
// ********************************************************************

void ZEM_ZEV_prop_waypoint(const state_type &y, state_type &dydt,\
    const double t)
{
    double t_go1 = tof_waypoint - t;
    if (t_go1 < 1e-2) {
        t_go1 = 1e-2;
    }

    double rr, r_p1, r_p2, v_p1, v_p2, zem1, zem2, zev1, zev2;
    r_p1 = y[0] + t_go1*y[2]; 
    r_p2 = y[1] + t_go1*y[3];
    v_p1 = y[2]; 
    v_p2 = y[3];
    zem1 = w_val[0]-r_p1;
    zem2 = w_val[1]-r_p2;
    zev1 = w_val[2]-v_p1;
    zev2 = w_val[3]-v_p2;

    dydt[0] = y[2];
    dydt[1] = y[3];
    dydt[2] = (6.0/(t_go1*t_go1))*zem1-(2.0/t_go1)*zev1;
    dydt[3] = (6.0/(t_go1*t_go1))*zem2-(2.0/t_go1)*zev2;
    dydt[4] = sqrt(dydt[2]*dydt[2]+dydt[3]*dydt[3]);
    // add CWH terms
    dydt[2] = dydt[2] + 3.0*0.0011*0.0011*y[0] + 2.0*0.011*y[3];
    dydt[3] = dydt[3] - 2.0*0.011*y[2];
    // add penalty
    rr = y[0]*y[0] + y[1]*y[1];
    if (rr < dockLength*dockLength) {
        dydt[4] = dydt[4] + 10.0;
    }
}

//**********************************************************************

void ZEM_ZEV_prop(const state_type &y, state_type &dydt, const double t)
 {
    double t_go2 = tof - t;
    if (t_go2 < 1e-2) {
        t_go2 = 1e-2;
    }
    double rr, r_p1, r_p2, v_p1, v_p2, zem1, zem2, zev1, zev2;
    r_p1 = y[0] + t_go2*y[2]; 
    r_p2 = y[1] + t_go2*y[3];
    v_p1 = y[2]; 
    v_p2 = y[3];
    zem1 = r_target1-r_p1;
    zem2 = r_target2-r_p2;
    zev1 = v_target1-v_p1;
    zev2 = v_target2-v_p2;
    
    dydt[0] = y[2];
    dydt[1] = y[3];
    dydt[2] = (6.0/(t_go2*t_go2))*zem1-(2.0/t_go2)*zev1;
    dydt[3] = (6.0/(t_go2*t_go2))*zem2-(2.0/t_go2)*zev2;
    dydt[4] = sqrt(dydt[2]*dydt[2]+dydt[3]*dydt[3]);
    // add CWH terms
    dydt[2] = dydt[2] + 3.0*0.0011*0.0011*y[0] + 2.0*0.011*y[3];
    dydt[3] = dydt[3] - 2.0*0.011*y[2];
    // add penalty
    rr = y[0]*y[0] + y[1]*y[1];
    if (rr < dockLength*dockLength) {
        dydt[4] = dydt[4] + 10.0;
    }
}

//*********************************************************************

// this function can be called as the 6th argument of the integrate
// routine if we want to save state values w/ time
void write_traj( const state_type &x , const double t )
{
    cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] \
        << '\t' << x[3] << '\t' << x[4] << endl;
}

//*****************************************************************************

double RotDockingCost(const waypoint &yw)
{

w_val[0] = yw[0];
w_val[1] = yw[1];
w_val[2] = yw[2];
w_val[3] = yw[3];

// Chaser Conditions
state_type y0 = {2.5, 1.3, 0, 0, 0}; // initial condition

//Time Propagation until waypoint
integrate( ZEM_ZEV_prop_waypoint, y0, 0.0, tof_waypoint, 0.01);

// Time Propagation until target
integrate( ZEM_ZEV_prop, y0, tof_waypoint, tof, 0.01);

return y0[4];
}

//********************************************************************************

int main(int argc, char *argv[]) {

int ierr = MPI_Init(&argc, &argv);

int p, rank;
ierr = MPI_Comm_size(MPI_COMM_WORLD, &p);
ierr = MPI_Comm_rank(MPI_COMM_WORLD, &rank);

const int n1 = 200;
const int n2 = 100;
const int n3 = 1000;

int work[p], startp[p];

int i, j, k, ii;

ii = n1-p*(n1/p);
startp[0] = 0;
for (i = 0; i < p; i++) {
    if (i < p - ii) {
	work[i] = n1/p;
    }
    else {
	work[i] = n1/p+1;
    }
 //   if (work[i] > ii) {
 //       ii = work[i];
 //   }
    if (i > 0) {
        startp[i] = startp[i-1]+work[i-1];
    }
    if (rank == 0) {
	cout << work[i] << '\t' << startp[i] << '\t'  << endl;
    }
}

size_t maxwork = work[p-1];
double J, minJ, minVx, minVy, rr;
waypoint w;

generator_type generator(time(0));

boost::uniform_real<> uni_dist(0,1);
boost::variate_generator<generator_type&, boost::uniform_real<> > rand(generator, uni_dist);

// offsets in variables
const double dx = 5.0/(n1-1);
const double dy = 2.6/(n2-1);

//ofstream outfile;
//outfile.open("output.txt");

// arrays to store output
std::vector<std::array<double, n2>>  Amin;
std::vector<std::array<std::array<double, 2>, n2>> Vmin;
Amin.resize(maxwork);
Vmin.resize(maxwork);

ofstream outfile;
if (rank == 0) {
    outfile.open("output_batch1.txt");
//    array<array<double, n2> n1> Aout;
  //  array<array<array<double, 2>, n2>, n1> Vout;
}

double JJ, rand1, rand2;

double* step;

for (i = 0; i < work[rank]; i++) {
    w = {0, 0, 0, 0};

    for (j = 0; j < n2; j++) {
        w[0] = -2.5 + (startp[rank]+i)*dx;
        w[1] = -1.3 + j*dy;
        minJ = 1e6;
	minVx = 1e6;
	minVy = 1e6;
        rr = w[0]*w[0] + w[1]*w[1];
	if (rr > dockLength*dockLength) {
	    for (k = 0; k < n3; k++) {
                rand1 = rand();
                rand2 = rand();

                w[2] = sqrt(rand1)*0.1*cos(rand2*2.0*pi);
                w[3] = sqrt(rand1)*0.1*sin(rand2*2.0*pi);
                JJ = RotDockingCost(w);
                
		if (JJ < minJ) {
                  minJ = JJ;
	          minVx = w[2];
	          minVy = w[3];
                }
            }
        }
        Amin[i][j] = minJ;
        Vmin[i][j][0] = minVx;
        Vmin[i][j][1] = minVy;     
   }
   if (rank == 0) {
   cout << i << endl;
   }
}



if (rank == 0) {
// output own work
for (i = 0; i < work[0]; i++) {
    for (j = 0; j < n2; j++) {
            outfile << i << '\t' << j << '\t' << Amin[i][j] << '\t' \
                << Vmin[i][j][0] << '\t' << Vmin[i][j][1] << endl;
    }  
}
}

// transmit or receive and output
MPI_Status status;
for (k = 1; k < p; k++) {
  if (rank == k) {
      ierr = MPI_Send(&Amin[0], work[k]*n2, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD);
      ierr = MPI_Send(&Vmin[0], work[k]*n2*2, MPI_DOUBLE, 0, 2, MPI_COMM_WORLD);
  }
  if (rank == 0) {
      ierr = MPI_Recv(&Amin[0], work[k]*n2, MPI_DOUBLE, k, 1, MPI_COMM_WORLD,&status);
      ierr = MPI_Recv(&Vmin[0], work[k]*n2*2, MPI_DOUBLE, \
          k, 2, MPI_COMM_WORLD,&status);
      for (i = 0; i < work[k]; i++) {
          for (j = 0; j < n2; j++) {
              outfile << startp[k]+i << '\t' << j << '\t' << Amin[i][j] << '\t' \
                << Vmin[i][j][0] << '\t' << Vmin[i][j][1] << endl;
          }
      }
  }
}

if (rank == 0) {
outfile.close();
}

MPI_Finalize();

return 0;
}

