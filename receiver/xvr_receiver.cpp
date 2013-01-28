#include <memory>
#include <thread>
#include "xvr_receiver.h"
#include "Receiver.hpp"

static std::unique_ptr<Receiver> p;

extern "C" {

void xvr_receiver_init()
{
    p.reset(new Receiver);
    p->start();
}

void xvr_receiver_draw()
{
    p->draw();
}

void xvr_receiver_destroy()
{
    p.reset();
}

}
