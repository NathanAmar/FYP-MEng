#ifndef EXAMPLE_DART_EXP_OFFICIAL_PARAMS_HPP
#define EXAMPLE_DART_EXP_OFFICIAL_PARAMS_HPP


struct Params {
    
    struct pop {
        // number of initial random points
        SFERES_CONST size_t init_size = 1000;
        // size of a batch
        SFERES_CONST size_t size = 500;
        SFERES_CONST size_t nb_gen = 40100;
        SFERES_CONST size_t dump_period = 100;
    };
    
    struct parameters {
        SFERES_CONST float min = 0;
        SFERES_CONST float max = 1;
    };
    struct evo_float {
        SFERES_CONST float cross_rate = 0.0f;
        SFERES_CONST float mutation_rate = 0.03f;
        SFERES_CONST float eta_m = 10.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
    
    struct nov {
        SFERES_CONST size_t deep = 3;
        SFERES_CONST double l = 0.01;
        SFERES_CONST double k = 15;
        SFERES_CONST double eps = 0.1;
    };
    
    struct qd {
#if defined D_3_8
        SFERES_CONST size_t behav_dim = 8;
        SFERES_ARRAY(size_t, grid_shape, 3,3,3,3,3,3,3,3);
#else
        SFERES_CONST size_t behav_dim = 4;
        SFERES_ARRAY(size_t, grid_shape, 10,10,10,10);
#endif
        
    };
    
};

#endif //EXAMPLE_DART_EXP_OFFICIAL_PARAMS_HPP
