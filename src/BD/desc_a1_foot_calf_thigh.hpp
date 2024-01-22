#ifndef ROBOT_DART_DESCRIPTOR_A1_HPP
#define ROBOT_DART_DESCRIPTOR_A1_HPP

// This is a 4D BD, Combines each legs contact on Foot + Calf + Thigh 
// The thigh is included for knee walk to not be discarded 

// for size_t
#include <cstddef>

#include <algorithm>
#include <map>
#include <vector>
#include <numeric>


#include<Eigen/Core>

namespace robot_dart {

namespace descriptor {

struct DutyCycle:public BaseDescriptor{
public:
    DutyCycle(size_t desc_dump = 1):BaseDescriptor(desc_dump)
    {
        for (size_t i = 0; i<5; i++){
            _contacts[i] = std::vector<size_t>();
        }
    }
       
    virtual void operator()()
    {
        std::vector<std::string> body_part = {
            "FR_foot", "FL_foot", "RR_foot", "RL_foot",
            "FR_calf", "FL_calf", "RR_calf", "RL_calf",
            "FR_thigh", "FL_thigh", "RR_thigh", "RL_thigh",
            "FR_hip", "FL_hip", "RR_hip", "RL_hip",
            };
        const dart::collision::CollisionResult& col_res = _simu->world()->getLastCollisionResult();
        for (size_t i = 0; i < 4; i++) {

            std::string leg_name = body_part[i];
            std::string leg_name2 = body_part[i + 4];
            std::string leg_name3 = body_part[i + 8];
            dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(leg_name);
            dart::dynamics::BodyNodePtr body_to_check2 = _simu->robots().back()->skeleton()->getBodyNode(leg_name2);
            dart::dynamics::BodyNodePtr body_to_check3 = _simu->robots().back()->skeleton()->getBodyNode(leg_name3);
            _contacts[i].push_back(col_res.inCollision(body_to_check) || col_res.inCollision(body_to_check2) || col_res.inCollision(body_to_check3));
        }
        // all individuals who's body part in the loop touch the floor
        for (size_t i=12; i<body_part.size(); i++){
            std::string link_name = body_part[i];
            dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(link_name);
            _contacts[4].push_back(col_res.inCollision(body_to_check));
        }
        dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("trunk");
        _contacts[4].push_back(col_res.inCollision(body_to_check));
        
    } // operator()

    
    void get(std::vector<double>& results)
    {
        for (size_t i = 0; i < 5; i++) {
            results.push_back(std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0);
        }
    }
protected:
    std::map<size_t, std::vector<size_t>> _contacts;
    
};//struct dutycycle



} // namespace descriptor
} // namespace robot_dart

#endif
