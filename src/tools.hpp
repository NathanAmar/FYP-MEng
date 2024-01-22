#ifndef __TOOLS__ADAPTATION_HPP__
#define __TOOLS__ADAPTATION_HPP__

#include <csignal>

#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>

#include <vector>
#include <map>
#include <assert.h>
#include <stdio.h>      /* printf */
#include <math.h>       /* round, floor, ceil, trunc */

#include "global_params.hpp"

std::vector< std::vector<float> > find_in_archive(const std::vector< std::vector<double> >& observations, const Params::archiveparams::archive_t& archive)
{

  std::vector< std::vector<float> > result;
  for(const auto & obs :observations)
  {
    std::vector<float> ctrl = archive.at(obs).controller;
    for(size_t i=0;i<ctrl.size();i++)
    {
      ctrl[i] = round( ctrl[i] * 1000.0 ) / 1000.0;// limite numerical issues
    }

    result.push_back(ctrl);  
  }
  return result;
}

std::vector< std::vector<double> > load_observation_file(std::string filename)
{
  std::vector< std::vector<double> > obs;
  std::ifstream monFlux(filename.c_str());    //what is monFlux??

  if (monFlux) {
      std::string line;
      std::getline(monFlux, line);// remove first line which is a comment
      while (std::getline(monFlux, line)) {
	      std::istringstream iss(line);
        std::vector<double> numbers;
        double num;
        while (iss >> num) {
          numbers.push_back(num);
        }

        if (numbers.size() < 9)   //if current number does not have the full data? expected # of entries is 
          continue;


        std::vector<double> candidate;
        for (int i = 0; i < 9; i++) {
          double data = numbers[i];
          if (i == 0) {
            
	          assert((size_t) std::round(data) == obs.size());  //make sure same index
          }
          if (i > 0 ) {
            candidate.push_back(data);
          }
        }
        if (candidate.size() == 8) {
          obs.push_back(candidate);
        }
      }
    }
    else {
      std::cerr << "ERROR: Could not load the observation file." << std::endl;
      return obs;
    }

    std::cout << obs.size() << " observations loaded" << std::endl;
    return obs;

}


std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> load_archive(std::string archive_name)
{

    std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;
    int linelength = global_params::ctrl_size + global_params::bd_size + 2; // +2 for fitness and index
    size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1); //what does extensions do? nothing?

    std::cout << "Loading text file..." << std::endl;
    std::ifstream monFlux(archive_name.c_str());

    if (monFlux) {
      std::string line;
      
      while (std::getline(monFlux, line)) {
        std::istringstream iss(line);
        std::vector<float> numbers;
        float num; 
        while (iss >> num) {
	        numbers.push_back(num);
	      }
        
	
        if (numbers.size() < linelength)    //43 is line length of archive
          continue;
        

	      Params::archiveparams::elem_archive elem;
	      std::vector<double> candidate(global_params::bd_size);
        elem.index = numbers[0];
        elem.duty_cycle = std::vector<double>(numbers.begin()+1, numbers.begin() + 1 + global_params::bd_size);
        
        elem.fit = numbers[global_params::bd_size + 1];
        

        elem.controller = std::vector<float>(numbers.begin()+2+global_params::bd_size, numbers.end());
        if (elem.fit > global_params::best_fit){
          global_params::best_fit = elem.fit; 
          global_params::best_fit_index = elem.index;
          global_params::best_fit_controller = elem.controller; 
        }
        archive[elem.duty_cycle] = elem;
      }
    }
    else {
      std::cerr << "ERROR: Could not load the archive." << std::endl;
      return archive;
    } 
    
    
    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}



void write_ctrls(const std::vector<std::vector<float> > &ctrls, std::string filename)
{
  std::ofstream monFlux(filename.c_str());
  for(const auto & ctrl :ctrls)
  {
    for(const auto & num : ctrl)
	    monFlux << num << "  ";
    monFlux << std::endl;
  }
}



std::vector< std::vector<float> > load_ctrl_file(std::string filename)
{
  std::vector< std::vector<float> > ctrls;
  std::ifstream monFlux(filename.c_str());
  if (monFlux) {
      std::string line;
      while (std::getline(monFlux, line)) {
	      std::istringstream iss(line);
        std::vector<float> numbers;
        float num;
        while (iss >> num) {
          numbers.push_back(num);
        }

        Params::archiveparams::elem_archive elem;
	      std::vector<double> candidate(8);
        elem.duty_cycle = std::vector<double>(numbers.begin()+1, numbers.begin() + 1 + global_params::bd_size);
        
        elem.fit = numbers[global_params::bd_size + 1];
        elem.controller = std::vector<float>(numbers.begin()+2+global_params::bd_size, numbers.end());

        ctrls.push_back(elem.controller);
      }
    }
    else {
      std::cerr << "ERROR: Could not load the observation file." << std::endl;
      return ctrls;
    }


    std::cout << ctrls.size() << " observations loaded" << std::endl;
    return ctrls;
}

#endif
