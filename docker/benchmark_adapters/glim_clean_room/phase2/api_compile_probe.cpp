#include <memory>
#include <type_traits>
#include <utility>

#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/global_mapping_base.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/mapping/sub_mapping_base.hpp>
#include <glim/mapping/sub_mapping_passthrough.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/odometry/odometry_estimation_imu.hpp>
#include <glim/preprocess/callbacks.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>
#include <glim/util/callback_slot.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/trajectory_manager.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module();
extern "C" glim::SubMappingBase* create_sub_mapping_module();
extern "C" glim::GlobalMappingBase* create_global_mapping_module();

namespace {

template <typename Expected, typename Actual>
constexpr bool same_signature() {
  return std::is_same<Expected, Actual>::value;
}

using OdomInsertImu = void (glim::OdometryEstimationBase::*)
  (double, const Eigen::Vector3d&, const Eigen::Vector3d&);
using OdomInsertFrame = glim::EstimationFrame::ConstPtr
  (glim::OdometryEstimationBase::*)
  (const glim::PreprocessedFrame::Ptr&, std::vector<glim::EstimationFrame::ConstPtr>&);
using OdomRemaining = std::vector<glim::EstimationFrame::ConstPtr>
  (glim::OdometryEstimationBase::*)();
static_assert(same_signature<OdomInsertImu,
              decltype(static_cast<OdomInsertImu>(&glim::OdometryEstimationBase::insert_imu))>(),
              "odometry IMU signature drift");
static_assert(same_signature<OdomInsertFrame,
              decltype(static_cast<OdomInsertFrame>(&glim::OdometryEstimationBase::insert_frame))>(),
              "odometry frame signature drift");
static_assert(same_signature<OdomRemaining,
              decltype(static_cast<OdomRemaining>(&glim::OdometryEstimationBase::get_remaining_frames))>(),
              "odometry drain signature drift");

using Preprocess = glim::PreprocessedFrame::Ptr (glim::CloudPreprocessor::*)
  (const glim::RawPoints::ConstPtr&);
static_assert(same_signature<Preprocess,
              decltype(static_cast<Preprocess>(&glim::CloudPreprocessor::preprocess))>(),
              "preprocessor signature drift");
using TimeProcess = bool (glim::TimeKeeper::*)(const glim::RawPoints::Ptr&);
static_assert(same_signature<TimeProcess,
              decltype(static_cast<TimeProcess>(&glim::TimeKeeper::process))>(),
              "timestamp process signature drift");
using ImuIntegrate = void (glim::IMUIntegration::*)
  (double, const Eigen::Vector3d&, const Eigen::Vector3d&);
static_assert(same_signature<ImuIntegrate,
              decltype(static_cast<ImuIntegrate>(&glim::IMUIntegration::insert_imu))>(),
              "IMU integration signature drift");
using Deskew = std::vector<Eigen::Vector4d> (glim::CloudDeskewing::*)
  (const Eigen::Isometry3d&, const Eigen::Vector3d&, const Eigen::Vector3d&,
   const std::vector<double>&, const std::vector<Eigen::Vector4d>&);
static_assert(same_signature<Deskew,
              decltype(static_cast<Deskew>(&glim::CloudDeskewing::deskew))>(),
              "deskew signature drift");

using SubInsertImu = void (glim::SubMappingBase::*)
  (double, const Eigen::Vector3d&, const Eigen::Vector3d&);
using SubInsertFrame = void (glim::SubMappingBase::*)
  (const glim::EstimationFrame::ConstPtr&);
using SubResults = std::vector<glim::SubMap::Ptr> (glim::SubMappingBase::*)();
static_assert(same_signature<SubInsertImu,
              decltype(static_cast<SubInsertImu>(&glim::SubMappingBase::insert_imu))>(),
              "submapping IMU signature drift");
static_assert(same_signature<SubInsertFrame,
              decltype(static_cast<SubInsertFrame>(&glim::SubMappingBase::insert_frame))>(),
              "submapping frame signature drift");
static_assert(same_signature<SubResults,
              decltype(static_cast<SubResults>(&glim::SubMappingBase::get_submaps))>(),
              "submapping result signature drift");
static_assert(same_signature<SubResults,
              decltype(static_cast<SubResults>(&glim::SubMappingBase::submit_end_of_sequence))>(),
              "submapping drain signature drift");

using GlobalInsertImu = void (glim::GlobalMappingBase::*)
  (double, const Eigen::Vector3d&, const Eigen::Vector3d&);
using GlobalInsertSubmap = void (glim::GlobalMappingBase::*)
  (const glim::SubMap::Ptr&);
using GlobalVoidDouble = void (glim::GlobalMappingBase::*)(double);
using GlobalVoid = void (glim::GlobalMappingBase::*)();
using GlobalSave = void (glim::GlobalMappingBase::*)(const std::string&);
using GlobalExport = gtsam_points::PointCloud::Ptr (glim::GlobalMappingBase::*)();
static_assert(same_signature<GlobalInsertImu,
              decltype(static_cast<GlobalInsertImu>(&glim::GlobalMappingBase::insert_imu))>(),
              "global mapping IMU signature drift");
static_assert(same_signature<GlobalInsertSubmap,
              decltype(static_cast<GlobalInsertSubmap>(&glim::GlobalMappingBase::insert_submap))>(),
              "global mapping input signature drift");
static_assert(same_signature<GlobalVoidDouble,
              decltype(static_cast<GlobalVoidDouble>(&glim::GlobalMappingBase::find_overlapping_submaps))>(),
              "global overlap request signature drift");
static_assert(same_signature<GlobalVoid,
              decltype(static_cast<GlobalVoid>(&glim::GlobalMappingBase::optimize))>(),
              "global optimize signature drift");
static_assert(same_signature<GlobalVoid,
              decltype(static_cast<GlobalVoid>(&glim::GlobalMappingBase::recover_graph))>(),
              "global recover signature drift");
static_assert(same_signature<GlobalSave,
              decltype(static_cast<GlobalSave>(&glim::GlobalMappingBase::save))>(),
              "global save signature drift");
static_assert(same_signature<GlobalExport,
              decltype(static_cast<GlobalExport>(&glim::GlobalMappingBase::export_points))>(),
              "global export signature drift");

using AddOdom = void (glim::TrajectoryManager::*)
  (double, const Eigen::Isometry3d&, int);
using UpdateAnchor = void (glim::TrajectoryManager::*)
  (double, const Eigen::Isometry3d&);
static_assert(same_signature<AddOdom,
              decltype(static_cast<AddOdom>(&glim::TrajectoryManager::add_odom))>(),
              "trajectory append signature drift");
static_assert(same_signature<UpdateAnchor,
              decltype(static_cast<UpdateAnchor>(&glim::TrajectoryManager::update_anchor))>(),
              "trajectory anchor signature drift");

using RawCallback = CallbackSlot<void(const glim::RawPoints::ConstPtr&)>;
using FrameCallback = CallbackSlot<void(const glim::EstimationFrame::ConstPtr&)>;
static_assert(std::is_same<decltype(glim::PreprocessCallbacks::on_raw_points_received),
                           RawCallback>::value,
              "raw callback type drift");
static_assert(std::is_same<decltype(glim::OdometryEstimationCallbacks::on_new_frame),
                           FrameCallback>::value,
              "odometry callback type drift");

static_assert(std::is_default_constructible<glim::RawPoints>::value,
              "RawPoints must remain aggregate-constructible");
static_assert(std::is_default_constructible<glim::PreprocessedFrame>::value,
              "PreprocessedFrame must remain constructible");
static_assert(std::is_default_constructible<glim::CloudPreprocessorParams>::value,
              "preprocessor params must remain constructible");
static_assert(std::is_constructible<glim::CloudPreprocessor,
                                    const glim::CloudPreprocessorParams&>::value,
              "preprocessor construction contract drift");
static_assert(std::is_constructible<glim::OdometryEstimationCPU,
                                    const glim::OdometryEstimationCPUParams&>::value,
              "CPU odometry construction contract drift");
static_assert(std::is_constructible<glim::SubMapping,
                                    const glim::SubMappingParams&>::value,
              "submapping construction contract drift");
static_assert(std::is_constructible<glim::SubMappingPassthrough,
                                    const glim::SubMappingPassthroughParams&>::value,
              "passthrough construction contract drift");
static_assert(std::is_constructible<glim::GlobalMapping,
                                    const glim::GlobalMappingParams&>::value,
              "global mapping construction contract drift");
static_assert(std::is_constructible<glim::AsyncSubMapping,
                                    const std::shared_ptr<glim::SubMappingBase>&>::value,
              "async submapping construction contract drift");
static_assert(std::is_constructible<glim::AsyncGlobalMapping,
                                    const std::shared_ptr<glim::GlobalMappingBase>&,
                                    int>::value,
              "async global mapping construction contract drift");

using ExtensionNeedsWait = bool (glim::ExtensionModule::*)() const;
using ExtensionExit = void (glim::ExtensionModule::*)(const std::string&);
static_assert(same_signature<ExtensionNeedsWait,
              decltype(static_cast<ExtensionNeedsWait>(&glim::ExtensionModule::needs_wait))>(),
              "extension lifecycle signature drift");
static_assert(same_signature<ExtensionNeedsWait,
              decltype(static_cast<ExtensionNeedsWait>(&glim::ExtensionModule::ok))>(),
              "extension health signature drift");
static_assert(same_signature<ExtensionExit,
              decltype(static_cast<ExtensionExit>(&glim::ExtensionModule::at_exit))>(),
              "extension shutdown signature drift");

using ConfigSave = void (glim::Config::*)(const std::string&) const;
using GlobalConfigInstance = glim::GlobalConfig* (*)(const std::string&, bool);
using GlobalConfigPath = std::string (*)(const std::string&);
static_assert(same_signature<ConfigSave,
              decltype(static_cast<ConfigSave>(&glim::Config::save))>(),
              "config save signature drift");
static_assert(std::is_same<decltype(&glim::GlobalConfig::instance),
                           GlobalConfigInstance>::value,
              "global config bootstrap signature drift");
static_assert(std::is_same<decltype(&glim::GlobalConfig::get_config_path),
                           GlobalConfigPath>::value,
              "global config path signature drift");

using OdomFactory = glim::OdometryEstimationBase* (*)();
using SubFactory = glim::SubMappingBase* (*)();
using GlobalFactory = glim::GlobalMappingBase* (*)();
static_assert(std::is_same<decltype(&create_odometry_estimation_module), OdomFactory>::value,
              "odometry factory ABI signature drift");
static_assert(std::is_same<decltype(&create_sub_mapping_module), SubFactory>::value,
              "submapping factory ABI signature drift");
static_assert(std::is_same<decltype(&create_global_mapping_module), GlobalFactory>::value,
              "global factory ABI signature drift");

}  // namespace

int main() {
  // Taking the addresses forces the exported factory symbols to be resolved by
  // the link step, while no constructor, sensor input, or map operation runs.
  volatile OdomFactory odometry_factory = &create_odometry_estimation_module;
  volatile SubFactory submapping_factory = &create_sub_mapping_module;
  volatile GlobalFactory global_factory = &create_global_mapping_module;
  (void)odometry_factory;
  (void)submapping_factory;
  (void)global_factory;
  return 0;
}
