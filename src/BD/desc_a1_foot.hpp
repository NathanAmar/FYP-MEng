#ifndef ROBOT_DART_DESCRIPTOR_A1_HPP
#define ROBOT_DART_DESCRIPTOR_A1_HPP


// THIS BD Forces the A1 to stand up and walk on it's foot (Sphere at the end of the leg)
// ALL individuals who's body part exept the foot touches the ground are killed 

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
            "FR_hip",  "FL_hip",  "RR_hip", "RL_hip",
            "FR_thigh","FL_thigh","RR_thigh", "RL_thigh"
        };
        const dart::collision::CollisionResult& col_res = _simu->world()->getLastCollisionResult();
        for (size_t i = 0; i < 4; i++) {

            std::string leg_name = body_part[i];
            dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(leg_name);
            _contacts[i].push_back(col_res.inCollision(body_to_check));
        }
        // all individuals who's body part in the loop touch the floor
        for (size_t i=4; i<body_part.size(); i++){
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
