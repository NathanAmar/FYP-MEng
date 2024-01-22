#! /usr/bin/env python
import sys
import os
import sferes
try:
    import limbo
except:
    print()

print(sys.path)
sys.path.insert(0, sys.path[0]+'/waf_tools')
print(sys.path)

from waflib.Configure import conf

import dart
import corrade
import magnum
import magnum_integration
import magnum_plugins
import robot_dart

def options(opt):
    opt.load('dart')
    opt.load('corrade')
    opt.load('magnum')
    opt.load('magnum_integration')
    opt.load('magnum_plugins')
    opt.load('robot_dart')

@conf
def configure(conf):
    print('conf exp:')
    conf.load('dart')
    conf.load('boost')
    conf.load('corrade')
    conf.load('magnum')
    conf.load('magnum_integration')
    conf.load('magnum_plugins')
    conf.load('robot_dart')
    
    boost_var =  'BOOST_ITE'
    conf.check_boost(lib='regex system serialization', min_version='1.46', uselib_store=boost_var)

    conf.check_dart() 
    conf.check_corrade(components='Utility PluginManager', required=False)
    conf.env['magnum_dep_libs'] = 'MeshTools Primitives Shaders SceneGraph GlfwApplication'
    if conf.env['DEST_OS'] == 'darwin':
        conf.env['magnum_dep_libs'] += ' WindowlessCglApplication'
    else:
        conf.env['magnum_dep_libs'] += ' WindowlessGlxApplication'

    conf.check_magnum(components=conf.env['magnum_dep_libs'], required=False)
    conf.check_magnum_plugins(components='AssimpImporter', required=False)
    conf.check_magnum_integration(components='Dart', required=False)
    if len(conf.env.INCLUDES_MagnumIntegration) > 0:
        conf.get_env()['BUILD_MAGNUM'] = True
        conf.env['magnum_libs'] = magnum.get_magnum_dependency_libs(conf, conf.env['magnum_dep_libs']) + magnum_integration.get_magnum_integration_dependency_libs(conf, 'Dart')

    conf.check_robot_dart()

    conf.env.LIB_THREADS = ['pthread']
    conf.env['LINKFLAGS_PTHREAD'] = '-pthread'

    print('done')

    
def build(bld):
    try:
        sferes.create_variants(bld,
                            source = 'src/dart_exp.cpp',
                            includes = '. ../../',
                            uselib = 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART',
                            use = 'sferes2',
                            target = 'a1',
                            variants = ['EVO FOOT', 'EVO FOOT_CALF', 'EVO FOOT_CALF_KNEE D_3_8', 'EVO FOOT_CALF_THIGH'])
                            
        sferes.create_variants(bld,
                            source = 'src/dart_exp.cpp',
                            includes = '. ../../',
                            uselib = bld.env['magnum_libs']  + 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART',
                            use = 'sferes2',
                            target = 'a1',
                            variants = ['EVO GRAPHIC'])
    except:

        hexa_libs = 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART BOOST_ITE LIMBO LIBCMAES NLOPT'


        limbo.create_variants(bld,
                    source = 'src/Limbo_Test.cpp',
                    includes = '. ./include',
                    uselib = hexa_libs,
                    uselib_local = 'limbo',
                    target = 'test',
                    variants = ['plain'])

        
        limbo.create_variants(bld,
                    source = 'src/adaptation_main.cpp',
                    includes = '. ./include',
                    uselib = hexa_libs,
                    uselib_local = 'limbo',
                    target = 'experiment',
                    variants = ['ADAPT D_4', 'ADAPT D_8'])

        limbo.create_variants(bld,
                        source = 'src/adaptation_main.cpp',
                        includes = '. ./include',
                        uselib = bld.env['magnum_libs']  + hexa_libs,
                        uselib_local = 'limbo',
                        target = 'experiment',
                        variants = ['ADAPT GRAPHIC D_4', 'ADAPT VIDEO D_4', 'ADAPT GRAPHIC D_8', 'ADAPT VIDEO D_8'])

        limbo.create_variants(bld,
                        source = 'src/playback.cpp',
                        includes = '. ./include',
                        uselib = bld.env['magnum_libs']  + hexa_libs,
                        uselib_local = 'limbo',
                        target = 'playback',
                        variants = ['ADAPT GRAPHIC D_4', 'ADAPT VIDEO D_4', 'ADAPT GRAPHIC D_8', 'ADAPT VIDEO D_8'])






    # sferes.create_variants(bld,
    #                        source = 'src/visualise.cpp',
    #                        includes = '. ../../',
    #                        uselib =  bld.env['magnum_libs']
    #                                  + 'ROBOTDART TBB BOOST EIGEN PTHREAD MPI DART ROBOT_DART DART_GRAPHIC',
    #                        use = 'sferes2',
    #                        target = 'visualise',
    #                        variants = ['GRAPHIC', 'VIDEO', 'GRAPHIC D_3_8', 'VIDEO D_3_8'])

    

    
