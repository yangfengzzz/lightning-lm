#include "core/backend/loop_registration/multi_resolution_ndt_registration.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include "utils/pointcloud_utils.h"

namespace lightning::backend {

void MultiResolutionNDTLoopRegistration::Compute(LoopCandidate& c,
                                                 const std::vector<Keyframe::Ptr>& all_keyframes) const {
    auto build_submap = [this, &all_keyframes](int given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -options_.submap_idx_range; idx < options_.submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= static_cast<int>(all_keyframes.size())) {
                continue;
            }

            auto kf = all_keyframes[id];
            CloudPtr cloud = kf->GetCloud();
            if (cloud->empty()) {
                continue;
            }

            SE3 Twb = kf->GetLIOPose();
            if (!build_in_world) {
                Twb = all_keyframes.at(given_id)->GetLIOPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());
            *submap += *cloud_trans;
        }
        return submap;
    };

    auto kf1 = all_keyframes.at(c.idx1_), kf2 = all_keyframes.at(c.idx2_);
    auto submap_kf1 = build_submap(kf1->GetID(), true);
    CloudPtr submap_kf2 = kf2->GetCloud();
    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    Mat4f Tw2 = kf2->GetLIOPose().matrix().cast<float>();
    CloudPtr output(new PointCloudType);
    for (const auto& r : options_.ndt_resolutions) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(options_.transformation_epsilon);
        ndt.setStepSize(options_.step_size);
        ndt.setMaximumIterations(options_.max_iterations);
        ndt.setResolution(r);

        auto target = VoxelGrid(submap_kf1, r * options_.voxel_size_ratio);
        auto source = VoxelGrid(submap_kf2, r * options_.voxel_size_ratio);
        ndt.setInputTarget(target);
        ndt.setInputSource(source);
        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();
        c.ndt_score_ = ndt.getTransformationProbability();
    }

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);
    c.Tij_ = kf1->GetLIOPose().inverse() * SE3(q, t);
}

}  // namespace lightning::backend
