#ifndef A1_CONTROLLER_SIMPLE_HPP
#define A1_CONTROLLER_SIMPLE_HPP

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define ARRAY_DIM 100




namespace A1_controller {

    class A1ControllerSimple {
    public:
        typedef std::array<double, ARRAY_DIM> array_t;

        A1ControllerSimple() {}
        A1ControllerSimple(const Eigen::VectorXd& ctrl, std::vector<int> broken_legs)
            : _broken_legs(broken_legs)
        {
            
            set_parameters(ctrl);
        }

        void set_parameters(const Eigen::VectorXd& ctrl)
        {
            
            assert(ctrl.size() == 48);

            _legs0commands.clear();
            _legs1commands.clear();
            _legs2commands.clear();
            _legs3commands.clear();

            _controller = ctrl;

            _legs0commands.push_back(_control_signal(ctrl[0], ctrl[1], ctrl[2], ctrl[3]));
            _legs0commands.push_back(_control_signal(ctrl[4], ctrl[5], ctrl[6], ctrl[7]));
            _legs0commands.push_back(_control_signal(ctrl[8], ctrl[9], ctrl[10], ctrl[11]));

            _legs1commands.push_back(_control_signal(ctrl[12], ctrl[13], ctrl[14], ctrl[15]));
            _legs1commands.push_back(_control_signal(ctrl[16], ctrl[17], ctrl[18], ctrl[19]));
            _legs1commands.push_back(_control_signal(ctrl[20], ctrl[21], ctrl[22], ctrl[23]));

            _legs2commands.push_back(_control_signal(ctrl[24], ctrl[25], ctrl[26], ctrl[27]));
            _legs2commands.push_back(_control_signal(ctrl[28], ctrl[29], ctrl[30], ctrl[31]));
            _legs2commands.push_back(_control_signal(ctrl[32], ctrl[33], ctrl[34], ctrl[35]));

            _legs3commands.push_back(_control_signal(ctrl[36], ctrl[37], ctrl[38], ctrl[39]));
            _legs3commands.push_back(_control_signal(ctrl[40], ctrl[41], ctrl[42], ctrl[43]));
            _legs3commands.push_back(_control_signal(ctrl[44], ctrl[45], ctrl[46], ctrl[47]));
        }

        const Eigen::VectorXd& parameters() const
        {
            return _controller;
        }

        void set_broken(const std::vector<int> broken_legs)
        {
            _broken_legs = broken_legs;
        }

        const std::vector<int>& broken_legs() const
        {
            return _broken_legs;
        }

            
        std::vector<double> pos(double t) const
        {
            
            assert(_controller.size() == 48);
            
            std::vector<double> angles;
            int leg = 0;
            
            for (size_t i = 0; i < 12; i += 3) {
                for (size_t j = 0; j < _broken_legs.size(); j++) {
                    if (leg == _broken_legs[j]) {
                        leg++;
                        if (_broken_legs.size() > j + 1 && _broken_legs[j + 1] != leg)
                            break;
                    }
                }

                switch (leg) {
                case 0:
                    angles.push_back( amplitude_to_angle(_legs0commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 0));
                    angles.push_back( amplitude_to_angle(_legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 1));
                    angles.push_back( amplitude_to_angle(_legs0commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 2));
                    break;

                case 1:
                    angles.push_back( amplitude_to_angle(_legs1commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 0));
                    angles.push_back( amplitude_to_angle(_legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 1));
                    angles.push_back( amplitude_to_angle(_legs1commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 2));
                    break;

                case 2:
                    angles.push_back( amplitude_to_angle(_legs2commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 0));
                    angles.push_back( amplitude_to_angle(_legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 1));
                    angles.push_back( amplitude_to_angle(_legs2commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 2));
                    break;

                case 3:
                    angles.push_back( amplitude_to_angle(_legs3commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 0));
                    angles.push_back( amplitude_to_angle(_legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 1));
                    angles.push_back( amplitude_to_angle(_legs3commands[2][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM], 2));
                    break;
                }
                

                ++leg;
            }
            
            return angles;
        }
        
        

    protected:
        /**
            All parameters should have a value between 0 and 1.
        **/
        double amplitude_to_angle(double amplitude, int motor) const {
            return dtorad * (lowerb[motor] + (ranges[motor] * (amplitude+1)/2));
        }
        
        array_t _control_signal(double amplitude_max, double amplitude_min, double phase, double duty_cycle) const
        {
            array_t temp;
            int up_time = ARRAY_DIM * duty_cycle;
            for (int i = 0; i < up_time; i++)
                temp[i] = 2*amplitude_max - 1;
            for (int i = up_time; i < ARRAY_DIM; i++)
                temp[i] = 2*amplitude_min - 1;

            // filtering
            int kernel_size = ARRAY_DIM / 10;

            array_t command;

            std::vector<double> kernel(2 * kernel_size + 1, 0);
            double sigma = kernel_size / 3;

            double sum = 0;
            for (int i = 0; i < int(kernel.size()); i++) {
                kernel[i] = exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma * sigma)) / (sigma * sqrt(M_PI));
                sum += kernel[i];
            }

            for (int i = 0; i < ARRAY_DIM; i++) {
                command[i] = 0;
                for (int d = 1; d <= kernel_size; d++) {
                    if (i - d < 0)
                        command[i] += temp[ARRAY_DIM + i - d] * kernel[kernel_size - d];
                    else
                        command[i] += temp[i - d] * kernel[kernel_size - d];
                }
                command[i] += temp[i] * kernel[kernel_size];
                for (int d = 1; d <= kernel_size; d++) {
                    if (i + d >= ARRAY_DIM)
                        command[i] += temp[i + d - ARRAY_DIM] * kernel[kernel_size + d];
                    else
                        command[i] += temp[i + d] * kernel[kernel_size + d];
                }

                command[i] /= sum;
            }

            // apply phase
            array_t final_command;
            int current = 0;
            int start = std::floor(ARRAY_DIM * phase);
            for (int i = start; i < ARRAY_DIM; i++) {
                final_command[current] = command[i];
                current++;
            }
            for (int i = 0; i < start; i++) {
                final_command[current] = command[i];
                current++;
            }
            
            return final_command;
        }

        std::vector<array_t> _legs0commands;
        std::vector<array_t> _legs1commands;
        std::vector<array_t> _legs2commands;
        std::vector<array_t> _legs3commands;
        
//        std::vector<double> ranges    = {20, 50., 102.};
//        std::vector<double> lowerb = {-10, -10, -154.5};
        // std::vector<double> ranges    = {40, 140., 47.5};
        // std::vector<double> lowerb = {-20, -60, -100};
        std::vector<double> ranges    = {80., 200., 102.};
        std::vector<double> lowerb = {-40., -60., -154.5};
        double dtorad = M_PI/180;
        

    Eigen::VectorXd _controller;
        std::vector<int> _broken_legs;
    };
} // namespace A1_controller

#endif
