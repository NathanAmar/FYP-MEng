//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.


#define NUM_THREADS 32
#include <sys/stat.h>

#include <iostream>

#include <sferes/eval/parallel.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/stat/qd_container.hpp>
#include <sferes/stat/qd_selection.hpp>
#include <sferes/stat/qd_progress.hpp>

#include <sferes/qd/container/archive.hpp>
#include <sferes/qd/container/grid.hpp>
#include <sferes/qd/quality_diversity.hpp>
#include <sferes/qd/selector/tournament.hpp>
#include <sferes/qd/selector/uniform.hpp>

// Importing the files from the task submodule

#include "a1_unidirectional.hpp"
#include "params_evo.hpp"


using namespace A1_unidirectional;

using namespace sferes::gen::evo_float;


int main(int argc, char **argv) 
{
    
    tbb::task_scheduler_init init(NUM_THREADS);
    using namespace sferes;
   
    // fit_t gen_t and phen_t are defined inside the task submodule
    typedef phen_t<Params> phen_t;
    
    
    // All the rest of the main is exactly similar to the code of example_dart_exp
    typedef qd::selector::Uniform<phen_t, Params> select_t;
    typedef qd::container::Grid<phen_t, Params> container_t;
    
#ifdef GRAPHIC
    typedef eval::Eval<Params> eval_t;
#else
    typedef eval::Parallel<Params> eval_t;
#endif
    
    typedef boost::fusion::vector<
    stat::BestFit<phen_t, Params>,
    stat::QdContainer<phen_t, Params>,
    stat::QdProgress<phen_t, Params>,
    stat::QdSelection<phen_t, Params>>
    stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef qd::QualityDiversity<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> qd_t;
    
    
    
    qd_t qd;
    run_ea(argc, argv, qd);
    
    std::cout<<"best fitness:" << qd.stat<0>().best()->fit().value() << std::endl;
    std::cout<<"archive size:" << qd.stat<1>().archive().size() << std::endl;
    
    return 0;
}
