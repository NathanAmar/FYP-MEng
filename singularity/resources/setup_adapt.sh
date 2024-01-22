#!/bin/bash

./waf configure --exp fyp-a-1-unidirectional-adaptation --dart /workspace --robot_dart /workspace --magnum_install_dir /workspace --magnum_integration_install_dir /workspace --magnum_plugins_install_dir /workspace --corrade_install_dir /workspace
./waf --exp fyp-a-1-unidirectional-adaptation $@
