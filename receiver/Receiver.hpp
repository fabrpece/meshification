#include <cstdint>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

class Model;
struct Data3d;
typedef boost::mutex Mutex;
typedef boost::unique_lock<Mutex> Lock;

class Receiver
{
    boost::thread t;
    bool is_running;

    Mutex m;
    std::unordered_set<std::uint64_t> new_models;
    std::unordered_set<std::uint64_t> delete_models;
    std::unordered_map<std::uint64_t, std::shared_ptr<Data3d>> updates;
    std::unordered_map<std::uint64_t, std::shared_ptr<Model>> models;
    float fov_v;
    float fov_h;

    void run();

public:
    Receiver();
    ~Receiver();
    static void init();
    void start();
    void stop();
    void draw();

    void translate(const std::string& name, const double x, const double y, const double z);
    void rotate(const std::string &name, const double rad, const double x, const double y, const double z);
    void reset_position(const std::string& name);
    void save_view() const;

    inline void getFOV(float& _fov_h, float& _fov_v) {_fov_v = fov_v; _fov_h = fov_h;}
};
