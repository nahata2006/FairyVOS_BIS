// Dans ORB-SLAM3/src/ImuPreintegration.cpp

void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f& acceleration, 
                                          const Eigen::Vector3f& angVel, 
                                          const float& dt)
{
    // 1. Récupérer la référence fixe (avion)
    static Eigen::Vector3f ref_accel = GetFixedReferenceAccel(); // À implémenter
    
    // 2. Calcul différentiel
    Eigen::Vector3f adjusted_accel = acceleration - ref_accel;
    
    // 3. Modification pour gravité variable (original : const Vector3f &g = mpImuCalib->getGravity())
    Eigen::Vector3f g = Eigen::Vector3f(0, 0, -Config::Get().gravity_value); 
    
    // 4. Préintégration modifiée
    Eigen::Vector3f accel_unbiased = adjusted_accel - mb.bacc;
    Eigen::Vector3f accel_gravity = accel_unbiased - g;
    
    // ... (reste du code original ORB-SLAM3)
}
