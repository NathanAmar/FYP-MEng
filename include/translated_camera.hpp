//
// Created by Luca Grillotti on 16/07/2021.
//

#ifndef A1_FYP_TRANSLATED_CAMERA_HPP
#define A1_FYP_TRANSLATED_CAMERA_HPP

#include <Magnum/GL/Renderer.h>

#include <robot_dart/robot.hpp>
#include <robot_dart/gui/magnum/sensor/camera.hpp>
#include <utility>

namespace robot_dart {
  namespace gui {
    namespace magnum {
      namespace sensor {

        class TranslatedCamera : public Camera
        {
        public:
          TranslatedCamera(std::shared_ptr<Robot> robot,
                           BaseApplication* app,
                           size_t width,
                           size_t height,
                           const Eigen::Vector4f& bg_color = Eigen::Vector4d(0.0f, 0.0f, 0.0f, 1.0f),
                           size_t freq = 30,
                           bool draw_debug = false)
            : m_robot(std::move(robot)),
              m_bg_color(bg_color[0], bg_color[1], bg_color[2], bg_color[3]),
              Camera(app, width, height, freq, draw_debug)
          {}

          void calculate(double t) override {
            auto final_pos = m_robot->skeleton()->getPositions().head(6).cast<double>();
            this->look_at({final_pos[3], final_pos[4] + 2., final_pos[5] + 0.55},
                          {final_pos[3], final_pos[4], final_pos[5] + 0.2});

            // Modified content of Camera::calculate(t); for adjusted background
            ROBOT_DART_EXCEPTION_ASSERT(_simu, "Simulation pointer is null!");
            /* Update graphic meshes/materials and render */
            _magnum_app->update_graphics();
            /* Update lights transformations --- this also draws the shadows if enabled */
            _magnum_app->update_lights(*_camera);

            Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::DepthTest);
            Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::FaceCulling);
            Magnum::GL::Renderer::enable(Magnum::GL::Renderer::Feature::Blending);
            Magnum::GL::Renderer::setBlendFunction(Magnum::GL::Renderer::BlendFunction::SourceAlpha, Magnum::GL::Renderer::BlendFunction::OneMinusSourceAlpha);
            Magnum::GL::Renderer::setBlendEquation(Magnum::GL::Renderer::BlendEquation::Add);

            /* Change clear color to black */
            Magnum::GL::Renderer::setClearColor(this->m_bg_color);

            /* Bind the framebuffer */
            _framebuffer.bind();
            /* Clear framebuffer */
            _framebuffer.clear(Magnum::GL::FramebufferClear::Color | Magnum::GL::FramebufferClear::Depth);

            /* Draw with this camera */
            _camera->draw(_magnum_app->drawables(), _framebuffer, _format, _simu, _magnum_app->debug_draw_data(), _draw_debug);
          }

        protected:
          std::shared_ptr<Robot> m_robot;
          Magnum::Color4 m_bg_color;
        };

      } // namespace sensor
    }   // namespace magnum
  }     // namespace gui
} // namespace robot_dart

#endif // AURORA_TRANSLATED_CAMERA_HPP
