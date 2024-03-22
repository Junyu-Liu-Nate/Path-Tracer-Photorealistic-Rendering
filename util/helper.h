#ifndef HELPER_H
#define HELPER_H

#include <tuple>
#include <Eigen>
#include "scene/scene.h"

using namespace Eigen;

class Helper
{
    public:
        Helper();

        std::tuple<Eigen::Vector3f, float> uniformSampleHemisphere(const Eigen::Vector3f &normal);
        std::tuple<Eigen::Vector3f, float> cosineWeightedSampleHemisphere(const Eigen::Vector3f& normal);
        std::tuple<Eigen::Vector3f, float> glossySpecularSampleHemisphere(const Eigen::Vector3f& normal, const Eigen::Vector3f& omegaIn, float n);

        Vector3f calRefractDirection(float refractFactor, Vector3f omegaIn, Vector3f normal);
        float calRefractAttenuation(Vector3f position, Vector3f direction, const Scene& scene);
        float fresnelProbability(float refractFactor, Vector3f omegaIn, Vector3f normal);

        std::vector<Vector3f> calDirectLighting(Vector3f p, Vector3f p_normal, const std::vector<Triangle*>&, const Scene& scene, int numSample);
        std::vector<Eigen::Vector3f> samplePointsInTriangle(const Eigen::Vector3<Eigen::Vector3f>& vertices, int n);
        float triangleArea(const Eigen::Vector3<Eigen::Vector3f>& vertices);

        Vector3f calLensPoint(float aperture);
        Vector3f calDirection(Vector3f lensPoint, Vector3f d, float focalLength);

        float haltonSequence(int index, int base);
};

#endif // HELPER_H
