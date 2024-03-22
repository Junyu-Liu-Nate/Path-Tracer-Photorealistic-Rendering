#include "brdf.h"
#include "util/helper.h"
#include <iostream>

BRDF::BRDF()
{

}

Vector3f BRDF::idealDiffuse(Vector3f matDiffuse) {
    Vector3f diffuseBRDF = matDiffuse / M_PI;
    return diffuseBRDF;
}

Vector3f BRDF::GlossyReflect(Vector3f omegaIn, Vector3f omegaOut, Vector3f normal, Vector3f matSpecular, float shininess) {
    Vector3f omegaInFlip = omegaIn - 2 * omegaIn.dot(normal) * normal;
    omegaInFlip.normalize();
    float dotProduct = omegaInFlip.dot(omegaOut);
    float clampedValue = std::max(0.0f, std::min(dotProduct, 1.0f));
    Vector3f glossyBRDF = matSpecular * ((shininess+2) / (2*M_PI)) * pow(dotProduct, shininess);
    return glossyBRDF;
}

// Ward anisotropic BRDF
Vector3f BRDF::wardAnisotropicSpecular(
    const Eigen::Vector3f& omegaIn,  // Incoming light direction
    const Eigen::Vector3f& omegaOut, // Viewing direction
    const Eigen::Vector3f& normal,   // Surface normal
    const Eigen::Vector3f& matSpecular, // Material specular color
    float alphaX, // Roughness parameter in X direction
    float alphaY, // Roughness parameter in Y direction
    const Eigen::Vector3f& X, // Tangent vector in X direction
    const Eigen::Vector3f& Y  // Tangent vector in Y direction
    ) {
    // Calculate the half-angle vector
    Eigen::Vector3f H = (omegaIn + omegaOut).normalized();

    // Dot products
    float NdotL = std::max(normal.dot(omegaIn), 0.0f);
    float NdotV = std::max(normal.dot(omegaOut), 0.0f);
    float NdotH = std::max(normal.dot(H), 0.0f);
    float HdotX = H.dot(X);
    float HdotY = H.dot(Y);

    // Check for early exit conditions
    if (NdotL <= 0.0f || NdotV <= 0.0f) {
        return Eigen::Vector3f(0, 0, 0); // Specular term is zero
    }

    // Ward anisotropic specular term
    float exponent = -2.0f * (pow(HdotX / alphaX, 2) + pow(HdotY / alphaY, 2)) / (1.0f + NdotH);
    float specTerm = (NdotL / (4.0f * M_PI * alphaX * alphaY * std::sqrt(NdotL * NdotV))) * std::exp(exponent);

    return matSpecular * specTerm;
}

void BRDF::calculateTangentBitangent(
    const Eigen::Vector3f& normal,
    Eigen::Vector3<Eigen::Vector3f> vertices,
    Eigen::Vector3f& tangent,
    Eigen::Vector3f& bitangent
    ) {
    Vector3f vertex0 = vertices[0];
    Vector3f vertex1 = vertices[1];
    Vector3f vertex2 = vertices[2];

    // Edge vectors
    Eigen::Vector3f edge1 = vertex1 - vertex0;
    Eigen::Vector3f edge2 = vertex2 - vertex0;

    // Normalize the normal vector to ensure it's a unit vector
    Eigen::Vector3f N = normal.normalized();

    // Calculate a vector that's orthogonal to the normal
    // Using cross product of edge1 and edge2 gives a vector in the plane of the triangle
    Eigen::Vector3f tmpTangent = edge1.cross(edge2);
    tmpTangent.normalize();

    // Ensure that the calculated tangent is orthogonal to the given normal
    // Project tmpTangent onto the plane defined by the normal
    tangent = (tmpTangent - N * N.dot(tmpTangent)).normalized();

    // Calculate bitangent orthogonal to both tangent and normal
    bitangent = N.cross(tangent).normalized();

    // Adjust bitangent to ensure it is in the plane of the triangle and orthogonal to tangent
    // This step may not be necessary as bitangent is already orthogonal due to cross product
    // but included here for completeness and potential numerical stability
    bitangent = (bitangent - tangent * tangent.dot(bitangent)).normalized();
}

// Cook-Torrance BRDF
float BRDF::beckmannDistribution(const Eigen::Vector3f& normal, const Eigen::Vector3f& H, float alpha) {
    float NdotH = normal.dot(H);
    float alphaSquared = alpha * alpha;
    float NdotHSquared = NdotH * NdotH;
    float exponent = (NdotHSquared - 1.0f) / (alphaSquared * NdotHSquared);
    float D = std::exp(exponent) / (M_PI * alphaSquared * NdotHSquared * NdotHSquared);
    return D;
}

float BRDF::geometricAttenuation(const Eigen::Vector3f& normal, const Eigen::Vector3f& V, const Eigen::Vector3f& L, const Eigen::Vector3f& H) {
    float NdotH = normal.dot(H);
    float VdotN = std::max(normal.dot(V), 0.0f);
    float LdotN = std::max(normal.dot(L), 0.0f);
    float VdotH = std::max(V.dot(H), 0.0f);
    float G = std::min({1.0f, (2.0f * NdotH * VdotN / VdotH), (2.0f * NdotH * LdotN / VdotH)});
    return G;
}

float fresnelProbability(float refractFactor, Vector3f omegaIn, Vector3f normal) {
    float cosThetaIn = normal.dot(-omegaIn);

    float n_i = refractFactor;
    float n_t = 1;
    float r_0 = pow((n_i-n_t) / (n_i+n_t), 2);

    float r_theta = r_0 + (1-r_0)*pow(1-cosThetaIn, 5);
    return r_theta;
}

float fresnelConductor(Vector3f omegaIn, Vector3f normal, float n, float k) {
    float cosTheta = normal.dot(-omegaIn);

    float F0 = ((n - 1) * (n - 1) + k * k) / ((n + 1) * (n + 1) + k * k);
    return F0 + (1 - F0) * pow(1 - cosTheta, 5);
}

Eigen::Vector3f BRDF::cookTorranceSpecularBRDF(
    const Eigen::Vector3f& omegaIn,
    const Eigen::Vector3f& omegaOut,
    const Eigen::Vector3f& normal,
    const Eigen::Vector3f& matSpecular,
    float alpha,
    float refractFactor
    ) {
    Eigen::Vector3f V = omegaOut.normalized(); // To the camera
    Eigen::Vector3f L = -omegaIn.normalized(); // To the light source
    Eigen::Vector3f H = (V + L).normalized(); // Half-angle vector

    float D = beckmannDistribution(normal, H, alpha);
    // float F = fresnelProbability(refractFactor, omegaIn, normal);
    float F = fresnelConductor(omegaIn, normal, 1.5, 3.0);
    float G = geometricAttenuation(normal, V, L, H);

    float VdotN = std::max(normal.dot(V), 0.0f);
    float LdotN = std::max(normal.dot(L), 0.0f);

    // Calculate BRDF scalar value
    float brdfScalar = (D * F * G) / std::max(4.0f * VdotN * LdotN, 0.0001f);

    // Apply BRDF scalar to the material specular color
    Eigen::Vector3f k_spec = matSpecular * brdfScalar; // Corrected line
    return k_spec;
}
