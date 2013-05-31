#include <memory>
#include <forward_list>
#include "xvr_receiver.h"
#include "Receiver.hpp"
#include "StaticModel.hpp"

#define USE_CUBEMAP_PANO

#ifdef USE_CUBEMAP_PANO
#include "ModelCubemappedPano.hpp"
#else
#include "ModelSphericalPano.hpp"
#endif



static std::unique_ptr<Receiver> p;
static std::list<StaticModel> static_models;

#ifdef USE_CUBEMAP_PANO
static std::unique_ptr<ModelCubemappedPano> panorama;
#else
static std::unique_ptr<ModelSphericalPano> panorama;
#endif

extern "C" {

void xvr_receiver_init()
{
    Receiver::init();
    p.reset(new Receiver);
#ifdef USE_CUBEMAP_PANO
    panorama.reset(new ModelCubemappedPano);
#else
    panorama.reset(new ModelSphericalPano);
#endif
    p->start();
}

void xvr_receiver_draw()
{
    p->draw();
    //panorama->draw();
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

void xvr_receiver_load_panorama(const char* fname)
{
    panorama->load(fname);
}

void xvr_receiver_translate(const char *name, const double x, const double y, const double z)
{
    p->translate(name, x, y, z);
}

void xvr_receiver_rotate(const char* name, const double rad, const double x, const double y, const double z)
{
    p->rotate(name, rad, x, y, z);
}

void xvr_receiver_reset_position(const char *name)
{
    p->reset_position(name);
}

void xvr_receiver_toggle_point_smooth()
{
    for (auto& m : static_models)
        m.tooglePointSmooth();
}

void xvr_receiver_save_view()
{
    p->save_view();
}

void xvr_receiver_get_FOV(float& fov_h, float& fov_v)
{
    p->getFOV(fov_h, fov_v);
}

}
