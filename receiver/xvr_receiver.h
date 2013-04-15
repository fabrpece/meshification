#pragma once

extern "C" {
    void xvr_receiver_init();
    void xvr_receiver_draw();
    void xvr_receiver_load_static(const char* fname);
    void xvr_receiver_destroy();
    void xvr_receiver_translate(const int i, const double x, const double y, const double z);
    void xvr_receiver_rotate(const int i, const double rad, const double x, const double y, const double z);
    void xvr_receiver_reset_position(const int i);
}
