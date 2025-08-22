#pragma once
#include <Eigen/Core>

namespace ORB_SLAM3 {
    class GravityManager {
    private:
        static float mGravityValue;
    public:
        static void SetGravity(float g) { mGravityValue = g; }
        static float GetGravity() { return mGravityValue; }
        
        static void UpdateFromReference(const Eigen::Vector3f& imuRef, 
                                      const Eigen::Vector3f& imuEmbarked,
                                      float threshold = 0.3f) {
            float delta = (imuRef - imuEmbarked).norm();
            if(delta > threshold) {
                mGravityValue = imuRef.norm();
            }
        }
    };
}
