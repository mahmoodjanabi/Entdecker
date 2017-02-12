extern "C" {
#include "rtklib.h"

void soltocov(const sol_t *sol, double *P);

}

#include "ros/ros.h"
#include "mw_rtkrcv/GpsLocation.h"

#include <sstream>

static ros::Publisher solution_pub;

extern "C" void ros_init(int argc, char **argv) {
  ros::init(argc, argv, "rtkrcv");
  ros::NodeHandle n;

  solution_pub = n.advertise<mw_rtkrcv::GpsLocation>("solution", 1000);
}

extern "C" void ros_output_solution(const sol_t *sol, const solopt_t *opt) {
    mw_rtkrcv::GpsLocation p;
    double pos[3], P[9], Q[9];

    trace(3, "ros_output_solution  :\n");

  if (ros::ok()) {
    if (sol->stat == 0 && sol->ns == 0) {
      pos[0] = 0.0;
      pos[1] = 0.0;
      pos[2] = 0.0;
    }
    else {
      ecef2pos(sol->rr, pos);
      soltocov(sol, P);
      covenu(pos, P, Q);

      if (opt->height == 1) { /* geodetic height */
        pos[2] -= geoidh(pos);
      }
    }

    p.latitude = pos[0] * R2D;
    p.longitude = pos[1] * R2D;
    p.height = pos[2];
    p.fix = sol->stat;
    p.sats = sol->ns;

    solution_pub.publish(p);
  }
}
