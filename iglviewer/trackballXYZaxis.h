#pragma once

namespace igl
{
    // Applies a trackball drag to identity
    // Inputs:
    //   w  width of the trackball context
    //   h  height of the trackball context
    //   speed_factor  controls how fast the trackball feels, 1 is normal
    //   down_mouse_x  x position of mouse down
    //   down_mouse_y  y position of mouse down
    //   mouse_x  current x position of mouse
    //   mouse_y  current y position of mouse
    // Outputs:
    //   quat  the resulting rotation (as quaternion)
    template <typename Q_type>
    void trackballXYZAxis(
        double w,
        double h,
        Q_type speed_factor,
        double down_mouse_x,
        double down_mouse_y,
        double mouse_x,
        double mouse_y,
        bool rotateX, bool rotateY, bool rotateZ,
        Q_type * quat);


    // Applies a trackball drag to a given rotation
    // Inputs:
    //   w  width of the trackball context
    //   h  height of the trackball context
    //   speed_factor  controls how fast the trackball feels, 1 is normal
    //   down_quat  rotation at mouse down, i.e. the rotation we're applying the
    //     trackball motion to (as quaternion)
    //   down_mouse_x  x position of mouse down
    //   down_mouse_y  y position of mouse down
    //   mouse_x  current x position of mouse
    //   mouse_y  current y position of mouse
    // Outputs:
    //   quat  the resulting rotation (as quaternion)
    template <typename Q_type>
    void trackballXYZAxis(
        double w,
        double h,
        Q_type speed_factor,
        const Q_type * down_quat,
        double down_mouse_x,
        double down_mouse_y,
        double mouse_x,
        double mouse_y,
        bool rotateX, bool rotateY, bool rotateZ,
        Q_type * quat);


    // Eigen wrapper.
    template <typename Scalardown_quat, typename Scalarquat>
    void trackballXYZAxis(
        double w,
        double h,
        double speed_factor,
        const Quaternion<Scalardown_quat> & down_quat,
        double down_mouse_x,
        double down_mouse_y,
        double mouse_x,
        double mouse_y,
        bool rotateX, bool rotateY, bool rotateZ,
        Quaternion<Scalarquat> & quat);

}