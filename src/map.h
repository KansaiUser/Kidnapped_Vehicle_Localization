/**
 * map.h
 *
 * Created on: Dec 12, 2016
 * Author: mufferm
 * Modified on Dec 22 2020
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <iostream>

class Map {
 public:  
  struct single_landmark_s {
    int id_i ; // Landmark ID
    float x_f; // Landmark x-position in the map (global coordinates)
    float y_f; // Landmark y-position in the map (global coordinates)
  };

  std::vector<single_landmark_s> landmark_list; // List of landmarks in the map

  void show(){
    for(auto const &l :landmark_list){
      std::cout<<"["<<l.id_i<<")"<<l.x_f<<","<<l.y_f<<"]  "<<std::endl;
    }
  }

  void show(int i) const{
    std::cout<<"["<<landmark_list[i].id_i<<")"<<landmark_list[i].x_f<<","<<landmark_list[i].y_f<<"]  "<<std::endl;
  }
};

#endif  // MAP_H_
