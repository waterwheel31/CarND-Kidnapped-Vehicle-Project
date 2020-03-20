/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

#define EPS 0.00001

using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution; 



void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  std::cout << "init" << std::endl;
  
  if (!is_initialized){
      num_particles = 50;  // TODO: Set the number of particles

      normal_distribution<double> x_noise(0, std[0]);
      normal_distribution<double> y_noise(0, std[1]);
      normal_distribution<double> theta_noise(0, std[2]);  
      std::random_device seed;
      std::minstd_rand0 engine(seed());  

      for (int i = 0; i < num_particles; i++){
        Particle p; 
        p.id = i;
        p.x = x + x_noise(engine);
        p.y = y + y_noise(engine);
        p.theta = theta + theta_noise(engine);
        p.weight = 1.0; 

        particles.push_back(p);
        
      }
      is_initialized = true; 
	}
 
  
  //std::cout << "init_end" << std::endl;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  	std::cout << "prediction" << std::endl;
  
  	normal_distribution<double> x_noise(0, std_pos[0]);
    normal_distribution<double> y_noise(0, std_pos[1]);
    normal_distribution<double> theta_noise(0, std_pos[2]);  
    std::random_device seed;
    std::minstd_rand0 engine(seed());  
  
  	for (unsigned int i = 0; i < particles.size(); i++){
      
       double theta = particles[i].theta;
      
       if(fabs(yaw_rate == 0)){ 
         particles[i].x += velocity * cos(theta) * delta_t + x_noise(engine);
         particles[i].y += velocity * sin(theta) * delta_t + y_noise(engine);   
         particles[i].theta += theta_noise(engine);   
       } else {
       	 particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta )) + x_noise(engine);
         particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t )) + y_noise(engine);
         particles[i].theta += yaw_rate * delta_t + theta_noise(engine);   
       }
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  	std::cout << "dataAssociation" << std::endl;
  
  
  	for (unsigned int i=0; i < observations.size(); i++){
    
      	int landm_ID = -1; 
      	double min_dist = std::numeric_limits<double>::max();
      
      	for(unsigned int j=0; j<predicted.size(); j++){
        	
          double x_dist = predicted[j].x - observations[i].x;
          double y_dist = predicted[j].y - observations[i].y; 
          double dist = sqrt(x_dist*x_dist + y_dist*y_dist); 
          
          if (dist < min_dist ){
            landm_ID = predicted[j].id;
            min_dist = dist;
          }
          
        }
      	observations[i].id = landm_ID; 
    }
  
  

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  	std::cout << "updateWeights" << std::endl;
  	
  	double weight_normalizer = 0.0;
  	for (unsigned int i=0; i<particles.size(); i++){
     
          double  new_weight = 1.0; 
          
      	  vector<LandmarkObs> obs_map; 
      	  double theta = particles[i].theta;
          for(unsigned int j=0; j<observations.size(); j++){
            LandmarkObs obs_trans; 
            obs_trans.x = particles[i].x + observations[j].x * cos(theta) - observations[j].y * sin(theta);
            obs_trans.y = particles[i].y + observations[j].x * sin(theta) + observations[j].y * cos(theta);
            obs_trans.id = j;
            obs_map.push_back(obs_trans); 
          }
      
      
      	  vector<LandmarkObs> range_landmarks; 
          for(unsigned int j=0; j<map_landmarks.landmark_list.size(); j++){

            double x_dist = particles[i].x - map_landmarks.landmark_list[j].x_f; 
            double y_dist = particles[i].y - map_landmarks.landmark_list[j].y_f; 

            double dist = sqrt(x_dist * x_dist + y_dist * y_dist);

            if (dist <= sensor_range){
              
              int id = map_landmarks.landmark_list[j].id_i;
              double landm_x = map_landmarks.landmark_list[j].x_f;
              double landm_y = map_landmarks.landmark_list[j].y_f;
              LandmarkObs landmark = LandmarkObs{id, landm_x, landm_y};
              range_landmarks.push_back(landmark);
            }
          }

          

          dataAssociation(range_landmarks, obs_map);
      
      
      	  particles[i].weight = 1.0;

          for (unsigned int j=0; j<obs_map.size(); j++){

            double obs_x = obs_map[j].x;
            double obs_y = obs_map[j].y;
            int landm_ID = obs_map[j].id;

            double landm_x, landm_y;

            for (unsigned int k=0; k<range_landmarks.size(); k++){
                if( range_landmarks[k].id == landm_ID){
                    landm_x = range_landmarks[k].x;
                    landm_y = range_landmarks[k].y;
                    break; 
                }
            }

            const double dx = obs_x - landm_x;
            const double dy = obs_y - landm_y;

            const double sigma_det = std_landmark[0] * std_landmark[1];   // matrix determinant. Assuming non disgonal elements are 0
            const double sigma_d = std_landmark[1];
            const double sigma_a = std_landmark[0];

            double eval_mes = exp(-1.0/2.0/sigma_det*( dx*dx*sigma_d + dy*dy*sigma_a)) / sqrt(2.0*M_PI * sigma_det);  // directly calculated
            if (eval_mes == 0){eval_mes = EPS; }
			// std::cout << "eval_mes: " << eval_mes << std::endl;
            new_weight *= eval_mes;  
            weight_normalizer += new_weight;
          }

      	  particles[i].weight = new_weight;
    }
  

  	for (unsigned int i=0; i<particles.size(); i++){
        // std::cout << "i: " << i << " particles[i].weight" << particles[i].weight << std::endl; 
    	particles[i].weight /= weight_normalizer; 
      
    }
 
 
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  	weights.clear();
    for (int i = 0; i < particles.size(); i++)
    {
      weights.push_back(particles[i].weight);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(std::begin(weights), std::end(weights));

    std::vector<Particle> resample_particles;
    for (int i = 0; i < num_particles; i++)
    {
      int index = d(gen);
      resample_particles.push_back(particles[index]);
    }

    particles = resample_particles;
 
 
 
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}