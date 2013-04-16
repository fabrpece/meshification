#include <memory>
#include <forward_list>
#include "xvr_receiver.h"
#include "Receiver.hpp"
#include "StaticModel.hpp"

static std::unique_ptr<Receiver> p;
static std::list<StaticModel> static_models;

extern "C" {

void xvr_receiver_init()
{
    Receiver::init();
    p.reset(new Receiver);
    p->start();
}

void xvr_receiver_draw()
{
    p->draw();
    for (const auto& m : static_models)
        m.draw();
}

void xvr_receiver_destroy()
{
    p.reset();
    static_models.clear();
}

void xvr_receiver_load_static(const char* fname)
{
    static_models.emplace_back();
    static_models.back().load(fname);
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
