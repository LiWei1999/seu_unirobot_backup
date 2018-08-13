#ifndef SEU_UNIROBOT_ACTUATOR_OPTIONS_HPP
#define SEU_UNIROBOT_ACTUATOR_OPTIONS_HPP

#include <boost/program_options.hpp>
#include "singleton.hpp"

class options: public singleton<options>
{
public:
    options();

    bool init(int argc, char *argv[]);
    int id() const { return id_; }
    bool use_debug() const { return use_debug_; }
    bool use_gc() const { return use_gc_; }
    bool use_camera() const { return use_camera_; }
    bool use_robot() const { return use_robot_; }
    bool use_comm() const { return use_comm_; }
    bool use_remote() const { return use_remote_; }

private:
    template<typename T> T arg(const std::string &opt) const { return var_map_[opt].as<T>(); }
    boost::program_options::options_description opts_desc_;
    boost::program_options::variables_map var_map_;
    int id_;
    bool use_debug_;
    bool use_gc_;
    bool use_camera_;
    bool use_robot_;
    bool use_comm_;
    bool use_remote_;
};

#define OPTS options::get_singleton()

#endif
