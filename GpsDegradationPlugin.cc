#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/navsat.pb.h>

#include <random>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <mutex>

namespace gps_degradation {

class GpsDegradationPlugin : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPostUpdate
{
public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override
    {
        this->sensorEntity = _entity;
        this->linkEntity = _ecm.ParentEntity(_entity);

        if (this->linkEntity != gz::sim::kNullEntity) {
            _ecm.CreateComponent(this->linkEntity, gz::sim::components::WorldLinearVelocity());
        }

        std::string sub_topic = "/world/dereks_world/model/x500_0/link/base_link/sensor/navsat_sensor/navsat_raw";
        std::string pub_topic = "/world/dereks_world/model/x500_0/link/base_link/sensor/navsat_sensor/navsat";

        this->pub = this->node.Advertise<gz::msgs::NavSat>(pub_topic);
        this->node.Subscribe(sub_topic, &GpsDegradationPlugin::OnNavSatMsg, this);

        this->rng.seed(1337);

        gzmsg << "[GpsDegradationPlugin] INITIALIZED\n";
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused || this->linkEntity == gz::sim::kNullEntity) {
            return;
        }

        auto velComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->linkEntity);
        if (!velComp) {
            return;
        }

        double current_speed = velComp->Data().Length();
        double current_sim_time = std::chrono::duration<double>(_info.simTime).count();

        if (!this->launched.load() && current_speed > this->launch_speed_threshold) {
            this->launched.store(true);
            this->launch_time_sec = current_sim_time;
            this->launch_factor = this->degradation_factor.load();
            gzerr << "[GpsDegradationPlugin] LAUNCH DETECTED at " << this->launch_time_sec << "s\n";
        }

        if (!this->launched.load()) {
            double factor = (current_sim_time >= this->pristine_hold_sec) ? 1.0 : 0.0;
            this->degradation_factor.store(factor);
        } else {
            double elapsed = current_sim_time - this->launch_time_sec;
            double recovery_progress = elapsed / this->recovery_duration_sec;
            double factor = std::clamp(this->launch_factor * (1.0 - recovery_progress), 0.0, 1.0);
            this->degradation_factor.store(factor);
        }
    }

private:
    void OnNavSatMsg(const gz::msgs::NavSat &_msg)
    {
        gz::msgs::NavSat degradedMsg = _msg;
        double current_factor = this->degradation_factor.load();

        if (current_factor > 0.001) {
            std::lock_guard<std::mutex> lock(this->rng_mutex);

            // Scale noise and bias by the current degradation factor
            std::normal_distribution<double> dist(0.0, this->max_pos_noise_stddev_deg * current_factor);
            double noise_lat = dist(this->rng);
            double noise_lon = dist(this->rng);
            double bias = this->max_bias_deg * current_factor;

            // Modify valid fields on gz::msgs::NavSat
            degradedMsg.set_latitude_deg(degradedMsg.latitude_deg() + bias + noise_lat);
            degradedMsg.set_longitude_deg(degradedMsg.longitude_deg() - bias + noise_lon);
            degradedMsg.set_altitude(degradedMsg.altitude() + (dist(this->rng) * this->alt_noise_scale));
        }

        this->pub.Publish(degradedMsg);
    }

    gz::sim::Entity sensorEntity{gz::sim::kNullEntity};
    gz::sim::Entity linkEntity{gz::sim::kNullEntity};
    gz::sim::Entity modelEntity{gz::sim::kNullEntity};

    gz::transport::Node node;
    gz::transport::Node::Publisher pub;

    std::atomic<bool> launched{false};
    double launch_time_sec{0.0};
    double launch_factor{0.0};

    double launch_speed_threshold{5.0};
    double pristine_hold_sec{0.0};
    double recovery_duration_sec{30.0};

    std::atomic<double> degradation_factor{0.0};

    double max_pos_noise_stddev_deg{0.0002};
    double max_bias_deg{0.0003};
    double alt_noise_scale{100.0};

    std::mutex rng_mutex;
    std::mt19937 rng;
};

}

GZ_ADD_PLUGIN(
    gps_degradation::GpsDegradationPlugin,
    gz::sim::System,
    gps_degradation::GpsDegradationPlugin::ISystemConfigure,
    gps_degradation::GpsDegradationPlugin::ISystemPostUpdate
)