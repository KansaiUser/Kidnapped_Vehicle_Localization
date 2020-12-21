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

using std::string;
using std::vector;


/*   In this function I will 
*  1) Initialize the array of particles std::vector<Particle> particles from a 
*   Gaussian distribution   x,y,thetha.
*   I wonder about id and weight?  Weigth probably will be 1 and id consecutive??
*/
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::default_random_engine gen;
  double std_x, std_y, std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];
  num_particles = 100;  // WATCH : MAGIC number here for the number of particles

  std::normal_distribution<double> dist_x(x, std_x); 
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_thetha(theta, std_theta);

  for(int i=0; i<num_particles;i++)
  {
    Particle element;
    element.id =i;
    element.x = dist_x(gen);
    element.y = dist_y(gen);  //Here gaussian
    element.theta = dist_thetha(gen);  //Here gaussian
    element.weight = 1.0;

    particles.push_back(element);
  }

  is_initialized=true;
}


/*  The velocity and the yaw rate refers to data from the vehicle (robot)
*  The std_pos is the same as in the init function  
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;
  double std_x, std_y, std_theta;
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];

  //Here we acknoledge the gaussian noise
  //According to the lesson video this is the place we should add noise to velocity and yaw rate
  //std::normal_distribution<double> dist_x(velocity, std_velocity);
  //however we don't have these standard deviations 
  //velocity =  

  for (int i=0; i<num_particles; i++){
    //Calcualte prediction for particle particles[i]
    // we have previous velocity and previous yaw rate of the vehicle
    //also delta_t 
    // particle position 
    //particles[i].x;
    //particles[i].y;
    //particles[i].theta;

    double newx = particles[i].x + (velocity/yaw_rate)*
                                  (sin(particles[i].theta + yaw_rate* delta_t) - sin(particles[i].theta)); 

    double newy = particles[i].y + (velocity/yaw_rate)*
                                  (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate* delta_t)); 

    double newtheta = particles[i].theta + yaw_rate * delta_t;

    // I should apply Gaussian noise here instead
    std::normal_distribution<double> dist_x(newx, std_x); 
    std::normal_distribution<double> dist_y(newy, std_y);
    std::normal_distribution<double> dist_thetha(newtheta, std_theta);

    particles[i].x = dist_x(gen);   //Something is failing HERE
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_thetha(gen);

//    particles[i].x = newx;
//    particles[i].y = newy;
//    particles[i].theta = newtheta;

  }
}

/**
   * dataAssociation Finds which observations correspond to which landmarks 
   *   (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations  (in other words Miu)
   * @param observations Vector of landmark observations  In other words X)
   */
/*
   predicted is Miu
   Observations is X
   We need M the total number of measurements for one particle
   sigma is the covariance of the measurement
   sigma is in our case given by sigma_landmark which is std_landamark in the other function

   predicted: The prediction measurements between one particular particle and all of the map landmarks
              within sensor range
   observation: The actual landmark measurements gathered from the LIDAR (the robot ones)  
   perform Nearest Neighbor and assign  each sensor observation the map landmark ID associated with it        
*/
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

}




void ParticleFilter::FromObservationToMap(const ground_truth &part, 
                         double obs_x, double obs_y, double &m_x, double &m_y){

  m_x = part.x +(cos( part.theta)*obs_x)-(sin(part.theta )*obs_y);

  m_y = part.y +(sin(part.theta)*obs_x) +(cos(part.theta)*obs_y);

}

/**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
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
  double sum_weights = 0.0;
  // For each particle  particles[i]
   for (int i=0; i<num_particles; i++){
       double final_weight = 1.0;
       if(i==0)
          std::cout<<i<<">Debugging for Particle "<< particles[i].id<<
           "("<<particles[i].x<<","<<particles[i].y<<","<<particles[i].theta<<")";
      
       for (unsigned int j=0; j< observations.size();j++){

         //NOTE: Here id is not set in the observations!!!
         if(i==0 && j<5)
            std::cout<<j<<"> and observation "<< observations[j].id<<"["<<observations[j].x<<","<<observations[j].y<<"]";
         // First transform the car measurements (observations) from local car measurement system to
         // map coordinate system -> transformed observation
         // with a HOMOGENEOUS TRANSFORMATION (rotation + translation) 
         ground_truth p{particles[i].x, particles[i].y, particles[i].theta};
         double m_x,m_y;

         FromObservationToMap(p, observations[j].x, observations[j].y, m_x, m_y);
         if(i==0 &&j<5)
             std::cout<<j<<">  Transformed observation ["<<m_x<<","<<m_y<<"]"<<std::endl;
          // We have the transformed observation in m_x and m_y
         //TODO put these values somewhere 

         //This is not possible
         //observations[j].x = m_x;
         //observations[j].y = m_y;
         //observations[j].id = 0;

      //Each measurement is associated with a landmark identifier (closest landmark to each
      // transformed observation)
      // perhaps this association is in  observations[j].id =  association (LandmarkObs->id)

      // We have the transformed observation in m_x and m_y
      // We have the landmark list in map_landmarks.landmark_list  (id_i, x_f, y_f)
      // we do something like
      // NOTE: Perhaps we could put here sensor_range to limit the number of landmarks we check!
      // but that would need also the particle position!
      //particles[i].x and particles[i].y
      
      // Here we are going to filter the landmarks by limiting them to sensor_range
      vector<LandmarkObs> landmark_list;
      for(unsigned int k=0; k< map_landmarks.landmark_list.size(); k++){
         //Get id and x,y coordinates
          float lm_x = map_landmarks.landmark_list[k].x_f;
          float lm_y = map_landmarks.landmark_list[k].y_f;
          int lm_id = map_landmarks.landmark_list[k].id_i;
          if(fabs(lm_x - particles[i].x) <= sensor_range && 
             fabs(lm_y - particles[i].y) <= sensor_range) {
                landmark_list.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
                if(i==0 &&j<5)
                   std::cout<<lm_id<<",";
             }
        }
      if(i==0&& j<5)  
      std::cout<<"There are "<<landmark_list.size()<<" landmarks"<<std::endl;  
       // Now we have the interesting landmarks in landmark_list
      int closest_id= find_nearest(m_x, m_y, landmark_list); 
      //observations[j].id = closest_id;   // I don't know how necessary this is
     //NOTE this is failing
     if(i==0&&j<5){
         std::cout<<"The nearest is #"<<closest_id<<" which is ";
         }
         closest_id--;  //THIS is very important. Unfortunately the ids don't follow the C++ rule for arrays
                       // and start by 1. So to identify the correct in the map_landmarks list we need to substract one
      if(i==0&&j<5){
         map_landmarks.show(closest_id);
         }


       
      //Calculate the weigth value of the particle particle[i].weight
        double the_weight;
        double mu_x,mu_y; //these are the actual location of the landmark for this observation
        // so in this case we have found that for this observation the closest is closest_id
        // so 
        mu_x = map_landmarks.landmark_list[closest_id].x_f;
        mu_y = map_landmarks.landmark_list[closest_id].y_f;

        //std::cout<<"["<<std_landmark[0]<<"   "<<std_landmark[1]<<"]-["<<m_x<<","<<m_y<<"]["<<mu_x<<","<<mu_y<<"]"<<std::endl;

        the_weight =  multiv_prob(std_landmark[0], std_landmark[1], m_x, m_y, mu_x, mu_y);

        //std::cout<<"theweight: "<<the_weight<<"  ";  // DEBUG Here it is giving 0 

        final_weight *= the_weight;
       }  //each observation

      particles[i].weight = final_weight;
      sum_weights += final_weight;
   }  // each particle

// Do we have to normalize the weights by dividing them with sum_weights ??
// I don't think this is necessary because in resample we are going to use discrete distribution
// which uses weights without normalization

//for (int i=0; i<num_particles; i++){
//    particles[i].weight = particles[i].weight/sum_weights;
//  }

}

// NOTE  this function is failing
int ParticleFilter::find_nearest(double obs_x, double obs_y, vector<LandmarkObs> &list){
    static bool first=true;
    int found_one=-1;
    double distance= INT16_MAX;
    if(first){
      std::cout<<"found index: "<<found_one<<" with distance "<<distance<<std::endl;
    }
    if(first)
       std::cout<<"list size: "<<list.size()<<std::endl;
    // for all the landmarks we have to find the closest to obs_x and obs_y
    for(unsigned int i=0; i<list.size();i++){
      double d= dist(obs_x, obs_y, list[i].x, list[i].y);
      if(first){
        std::cout<<"  distance: "<<d<<" for landmark "<<list[i].id;
      }
      if(d<distance){
        distance = d;
        found_one =  list[i].id; // not i;
        if(first){
           std::cout<<"found index: "<<found_one<<" with distance "<<distance<<std::endl;
          }
      }
    }
  
   if(first){
      std::cout<<"found index: "<<found_one<<" with distance "<<distance<<std::endl;
    }
 
   first=false;
  return found_one;
} 

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  // Clean the vector
  weights.clear();
  // First put all the weights in a vector
  std::cout<<"Weights: ";
  for (auto const & p: particles){
    std::cout<<p.weight<<"  ";
    weights.push_back(p.weight);
  }
  std::cout<<std::endl;

  std::cout<<"Preparing distribution"<<std::endl;

  std::discrete_distribution<size_t> distr(weights.begin(), weights.end());
  std::random_device rd;

  std::cout<<"distribution prepared"<<std::endl;
  // From here we are going to get the index for the particles
  // as distr(rd)
  // Now we have to think how to resample the particles

  //std::random_device rd;
  //std::mt19937 gen(rd());
 // for (size_t i = 0; i < 20; ++i)
  //      std::cout << distr(rd) << " ";
  //std::cout<<"New Particles: ";
  std::vector<Particle> new_particles; 
  for(int i=0;i<num_particles;i++){
    int indice = distr(rd);
    std::cout<< indice <<" ";
    new_particles.push_back(particles[indice]);
  }   
  std::cout<<std::endl;  

  particles = new_particles;
/*
for(int i=0; i<num_particles;i++)
  {
    Particle element;
    element.id =i;
    element.x = dist_x(gen);
    element.y = dist_y(gen);  //Here gaussian
    element.theta = dist_thetha(gen);  //Here gaussian
    element.weight = 1.0;

    particles.push_back(element);
  }
*/



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