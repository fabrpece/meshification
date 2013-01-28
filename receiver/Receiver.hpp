#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

class Model;
struct Data3d;

class Receiver
{
    std::thread t;
    bool is_running = false;

    std::mutex m;
    std::unordered_set<std::uint64_t> new_models;
    std::unordered_set<std::uint64_t> delete_models;
    std::unordered_map<std::uint64_t, std::shared_ptr<Data3d>> updates;

    std::unordered_map<std::uint64_t, std::shared_ptr<Model>> models;

    void run();

public:
    Receiver();
    ~Receiver();
    void start();
    void stop();
    void draw();
};
