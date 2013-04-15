#include <memory>
#include "xvr_receiver.h"
#include "Receiver.hpp"
#include "StaticModel.hpp"

static std::unique_ptr<Receiver> p;
static std::unique_ptr<StaticModel> p_static;

extern "C" {

void xvr_receiver_init()
{
    Receiver::init();
    p.reset(new Receiver);
    p_static.reset(new StaticModel);
    p->start();
}

void xvr_receiver_draw()
{
    p->draw();
    p_static->draw();
}

void xvr_receiver_destroy()
{
    p.reset();
    p_static.reset();
}

void xvr_receiver_load_static(const char* fname)
{
    p_static->load(fname);
}

void xvr_receiver_translate(const int i, const double x, const double y, const double z)
{
    p->translate(i, x, y, z);
}

void xvr_receiver_rotate(const int i, const double rad, const double x, const double y, const double z)
{
    p->rotate(i, rad, x, y, z);
}

void xvr_receiver_reset_position(const int i)
{
    p->reset_position(i);
}

}
