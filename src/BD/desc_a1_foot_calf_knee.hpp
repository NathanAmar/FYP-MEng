#ifndef ROBOT_DART_DESCRIPTOR_A1_HPP
#define ROBOT_DART_DESCRIPTOR_A1_HPP

// THis BD is 8D
// The Goal is to have some individuals walking on it's foot and also using some legs on the knee 

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
        BD_dim = 8; 
        for (size_t i = 0; i<BD_dim + 1; i++){ // Adding one to include in the results contacts of the unwanted body parts 
            _contacts[i] = std::vector<size_t>();
        }
    }
       
    virtual void operator()()
    {
        std::vector<std::string> body_parts = {
            "FR_foot", "FL_foot", "RR_foot", "RL_foot",
            "FR_calf", "FL_calf", "RR_calf", "RL_calf",
            "FR_hip",  "FL_hip",  "RR_hip", "RL_hip",
            "FR_thigh","FL_thigh","RR_thigh", "RL_thigh"
        };
        std::vector<std::vector<std::string>> body_parts_groups = {
            {"FR_foot"}, {"FL_foot"}, {"RR_foot"}, {"RL_foot"},
            {"FR_calf", "FR_thigh"}, {"FL_calf", "FL_thigh"}, {"RR_calf", "RR_thigh"}, {"RL_calf", "RL_thigh"}
        };

        const dart::collision::CollisionResult& col_res = _simu->world()->getLastCollisionResult();
        for (int i = 0; i < BD_dim; i++) {

            std::string body_part = body_parts_groups[i][0];
            dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(body_part);
            bool collision = col_res.inCollision(body_to_check);

            for (int j=1; j < body_parts_groups[i].size(); j++){
                body_part = body_parts_groups[i][j];
                body_to_check = _simu->robots().back()->skeleton()->getBodyNode(body_part);
                collision = collision || col_res.inCollision(body_to_check);
            }
            _contacts[i].push_back(collision);
        }
        // all individuals who's body part in the loop touch the floor
        // for (size_t i=12; i<body_part.size(); i++){
        //     std::string link_name = body_part[i];
        //     dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode(link_name);
        //     _contacts[4].push_back(col_res.inCollision(body_to_check));
        // }
        dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("trunk");
        _contacts[BD_dim].push_back(col_res.inCollision(body_to_check));
        
    } // operator()

    
    void get(std::vector<double>& results)
    {
        for (size_t i = 0; i < BD_dim + 1; i++) {
            results.push_back(std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0);
        }
    }
protected:
    std::map<size_t, std::vector<size_t>> _contacts;
    int BD_dim;
    
};//struct dutycycle



} // namespace descriptor
} // namespace robot_dart

#endif
