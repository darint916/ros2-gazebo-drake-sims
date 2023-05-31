#ifndef __VELOCITY_CONTROL_H__
#define __VELOCITY_CONTROL_H__

#include <memory>
#include <optional>

#include <ignition/gazebo/System.hh>

// Inline bracket to help doxygen filtering.
// inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
using namespace ignition;
using namespace gazebo;
namespace velocity_control
{
  // Forward declaration
  class VelocityControlPrivate
{
  /// \brief Callback for model velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Twist &_msg);

  /// \brief Callback for link velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnLinkCmdVel(const msgs::Twist &_msg,
    const transport::MessageInfo &_info);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update link velocity.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateLinkVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Angular velocity of a model
  public: math::Vector3d angularVelocity{0, 0, 0};

  /// \brief Linear velocity of a model
  public: math::Vector3d linearVelocity{0, 0, 0};

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the model velocity command.
  public: std::mutex mutex;

  /// \brief Link names
  public: std::vector<std::string> linkNames;

  /// \brief Link entities in a model
  public: std::unordered_map<std::string, Entity> links;

  /// \brief Angular velocities of links
  public: std::unordered_map<std::string, math::Vector3d> angularVelocities;

  /// \brief Linear velocities of links
  public: std::unordered_map<std::string, math::Vector3d> linearVelocities;

  /// \brief All link velocites
  public: std::unordered_map<std::string, msgs::Twist> linkVels;

    // bool initial_step = True;
};


  /// \brief Linear and angular velocity controller
  /// which is directly set on a model.
  ///
  /// ## System Parameters
  ///
  /// `<topic>` Topic to receive commands in. Defaults to
  ///     `/model/<model_name>/cmd_vel`.
  ///
  /// `<initial_linear>` Linear velocity to start with.
  ///
  /// `<initial_angular>` Linear velocity to start with.
  class VelocityControl
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: VelocityControl();

    /// \brief Destructor
    public: ~VelocityControl() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &/*_info*/,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<VelocityControlPrivate> dataPtr;
  };
}

#endif