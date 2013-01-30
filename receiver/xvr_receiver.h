#pragma once

extern "C" {
    void xvr_receiver_init();
    void xvr_receiver_draw();
    void xvr_receiver_load_static(const char* fname);
    void xvr_receiver_destroy();
}
