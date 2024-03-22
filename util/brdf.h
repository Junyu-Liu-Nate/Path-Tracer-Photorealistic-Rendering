#ifndef BRDF_H
#define BRDF_H

#include <Eigen>

using namespace Eigen;

class BRDF
{
public:
    BRDF();

    Vector3f idealDiffuse(Vector3f matDiffuse);
    Vector3f GlossyReflect(Vector3f omegaIn, Vector3f omegaOut, Vector3f normal, Vector3f matSpecular, float shininess);
    Vector3f MirrorRefelct(Vector3f omegaIn, Vector3f omegaOut);
    Vector3f Refraction(Vector3f omegaIn, Vector3f omegaOut);

    //TODO: More advanced BRDFs
    //      1. Ward anisotropic BRDF
    Vector3f wardAnisotropicSpecular(
        const Eigen::Vector3f& omegaIn,  // Incoming light direction
        const Eigen::Vector3f& omegaOut, // Viewing direction
        const Eigen::Vector3f& normal,   // Surface normal
        const Eigen::Vector3f& matSpecular, // Material specular color
        float alphaX, // Roughness parameter in X direction
        float alphaY, // Roughness parameter in Y direction
        const Eigen::Vector3f& X, // Tangent vector in X direction
        const Eigen::Vector3f& Y  // Tangent vector in Y direction
        );
    void calculateTangentBitangent(
        const Eigen::Vector3f& normal,
        Eigen::Vector3<Eigen::Vector3f> vertices,
        Eigen::Vector3f& tangent,
        Eigen::Vector3f& bitangent
        );

    //      2. Cook-Torrance microfacet model
    float beckmannDistribution(const Eigen::Vector3f& normal, const Eigen::Vector3f& H, float alpha);
    float geometricAttenuation(const Eigen::Vector3f& normal, const Eigen::Vector3f& V, const Eigen::Vector3f& L, const Eigen::Vector3f& H);
    Eigen::Vector3f cookTorranceSpecularBRDF(
        const Eigen::Vector3f& omegaIn,
        const Eigen::Vector3f& omegaOut,
        const Eigen::Vector3f& normal,
        const Eigen::Vector3f& matSpecular,
        float alpha,
        float refractFactor
        );
};

#endif // BRDF_H
