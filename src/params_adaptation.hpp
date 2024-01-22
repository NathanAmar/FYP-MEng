#ifndef __PARAMS__ADAPTATION_HPP__
#define __PARAMS__ADAPTATION_HPP__

#include <limbo/limbo.hpp>

#include "global_params.hpp"

using namespace limbo;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, true);
    };

    // no noise
    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 1e-10);
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
        BO_PARAM(double, l, 0.4);
    };

    struct stop_maxiterations {
        BO_DYN_PARAM(int, iterations);
    };

    struct stop_maxpredictedvalue: public defaults::stop_maxpredictedvalue {
        BO_PARAM(double, ratio, 0.9);
    };
    
    struct acqui_ucb : public defaults::acqui_ucb {
        BO_PARAM(double, alpha, 0.2);
    };



    struct archiveparams {

        struct elem_archive {
            int index;
            std::vector<double> duty_cycle;
            float fit;
            std::vector<float> controller;
        };

        struct classcomp {
            bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const
            {
                assert(lhs.size() == global_params::bd_size && rhs.size() == global_params::bd_size);
                int i = 0;
                int test = global_params::bd_size - 1;
                int discretization_levels = 10; 
                while (i < test && std::round(lhs[i] * discretization_levels - 1) == std::round(rhs[i] * discretization_levels - 1)) 
                    i++;

                return std::round(lhs[i] * discretization_levels - 1) < std::round(rhs[i] * discretization_levels - 1 ); 
            }
        };
        typedef std::map<std::vector<double>, elem_archive, classcomp> archive_t;
        static std::map<std::vector<double>, elem_archive, classcomp> archive;
    };
};



#endif
