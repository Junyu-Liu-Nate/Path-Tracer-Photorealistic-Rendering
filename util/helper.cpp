#include "helper.h"
#include <iostream>

Helper::Helper()
{

}

/***** Functions for sampling *****/
std::tuple<Eigen::Vector3f, float> Helper::uniformSampleHemisphere(const Eigen::Vector3f &normal) {
    float rand1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float rand2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    float phi = 2 * M_PI * rand1;
    float theta = acos(1 - rand2);

    float x = sin(theta) * cos(phi);
    float z = sin(theta) * sin(phi);
    float y = cos(theta);
    Eigen::Vector3f omegaLocal(x, y, z);

    // Create a coordinate system around the normal
    Eigen::Vector3f up = std::abs(normal.z()) < 0.999 ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f tangent = up.cross(normal).normalized();
    Eigen::Vector3f bitangent = normal.cross(tangent);

    // Transform the sampled direction into the coordinate system defined by the normal
    Eigen::Vector3f omega = tangent * omegaLocal.x() + bitangent * omegaLocal.z() + normal * omegaLocal.y();

    return std::make_tuple(omega.normalized(), 1 / (2 * M_PI));
}

std::tuple<Eigen::Vector3f, float> Helper::cosineWeightedSampleHemisphere(const Eigen::Vector3f& normal) {
    float rand1 = static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX);
    float rand2 = static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX);

    // Use cosine-weighted hemisphere sampling
    float phi = 2 * M_PI * rand1;
    float theta = std::acos(std::sqrt(1 - rand2));

    float x = std::cos(phi) * std::sin(theta);
    float y = std::cos(theta); // cos(theta) is proportional to the desired cosine-weighted distribution
    float z = std::sin(phi) * std::sin(theta);

    Eigen::Vector3f omegaLocal(x, y, z);

    // Create a coordinate system around the normal as before
    Eigen::Vector3f up = std::abs(normal.z()) < 0.999 ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f tangent = up.cross(normal).normalized();
    Eigen::Vector3f bitangent = normal.cross(tangent);

    // Transform the sampled direction into the coordinate system defined by the normal
    Eigen::Vector3f omega = tangent * omegaLocal.x() + bitangent * omegaLocal.z() + normal * omegaLocal.y();

    // The probability density function for a cosine-weighted hemisphere is cos(theta)/pi
    float pdf = std::cos(theta) / M_PI;

    return std::make_tuple(omega.normalized(), pdf);
}

std::tuple<Eigen::Vector3f, float> Helper::glossySpecularSampleHemisphere(const Eigen::Vector3f& normal, const Eigen::Vector3f& omegaIn, float n) {
    // Calculate reflected direction
    Eigen::Vector3f omegaInFlip = omegaIn - 2 * omegaIn.dot(normal) * normal;

    // Generate random numbers for sampling
    float rand1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float rand2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    // Convert to spherical coordinates (r, theta, phi)
    float phi = 2 * M_PI * rand1;
    float cosTheta = std::pow(rand2, 1.0f / (n + 1));
    float sinTheta = std::sqrt(1 - cosTheta * cosTheta);

    // Convert spherical coordinates back to Cartesian, align with omegaInFlip
    float x = sinTheta * std::cos(phi);
    float y = sinTheta * std::sin(phi);
    float z = cosTheta;

    Eigen::Vector3f sampleDirectionLocal(x, y, z);

    // Create a coordinate system around omegaInFlip
    Eigen::Vector3f upVector = std::abs(omegaInFlip.z()) < 0.999 ? Eigen::Vector3f(0, 0, 1) : Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f tangent = upVector.cross(omegaInFlip).normalized();
    Eigen::Vector3f bitangent = omegaInFlip.cross(tangent);

    // Transform the sampled direction into the coordinate system defined by omegaInFlip
    Eigen::Vector3f sampleDirection = tangent * sampleDirectionLocal.x() + bitangent * sampleDirectionLocal.y() + omegaInFlip * sampleDirectionLocal.z();

    // Calculate the PDF for the sampled direction
    float pdf = std::pow(cosTheta, n) * (n + 1) / (2 * M_PI);

    return std::make_tuple(sampleDirection.normalized(), pdf);
}

/***** Functions for refraction *****/
Vector3f Helper::calRefractDirection(float refractFactor, Vector3f omegaIn, Vector3f normal) {    
    float cosThetaIn = normal.dot(-omegaIn);

    if (1 - refractFactor * refractFactor * (1 - cosThetaIn * cosThetaIn) < 0) {
        // Internal reflection
        Vector3f refractDirection = omegaIn - 2 * omegaIn.dot(normal) * normal;
        return refractDirection.normalized();
    }
    else{
        // Refraction
        float cosThetaOut = sqrt(1 - refractFactor * refractFactor * (1 - cosThetaIn * cosThetaIn));
        Vector3f refractDirection = refractFactor*omegaIn + (refractFactor*cosThetaIn - cosThetaOut) * normal;
        return refractDirection.normalized();
    }
}

float Helper::calRefractAttenuation(Vector3f position, Vector3f direction, const Scene& scene) {
    Ray ray(position, direction);
    IntersectionInfo i;
    if (scene.getIntersection(ray, &i)) {
        Vector3f hitPosition = i.hit;
        float distance = (position - hitPosition).norm();
        //TODO: which attenuation function to use?
        float attenuation = 1.0 / (1 + 1.0*distance + 1.0*distance*distance);
        return attenuation;
    }
    else {
        return 1.0f;
    }
}

float Helper::fresnelProbability(float refractFactor, Vector3f omegaIn, Vector3f normal) {
    float cosThetaIn = normal.dot(-omegaIn);

    float n_i = refractFactor;
    float n_t = 1;
    float r_0 = pow((n_i-n_t) / (n_i+n_t), 2);

    float r_theta = r_0 + (1-r_0)*pow(1-cosThetaIn, 5);
    return r_theta;
}

/***** Functions for direct lighting *****/
std::vector<Vector3f> Helper::calDirectLighting(Vector3f p, Vector3f p_normal, const std::vector<Triangle*>& triLights, const Scene& scene, int numSample) {
    Vector3f totalDirectRadian = Vector3f(0, 0, 0);
    Vector3f totalDirection = Vector3f(0, 0, 0);
    for (Triangle* triLight:triLights) {
        Eigen::Vector3<Eigen::Vector3f> vertices = triLight->getVertices();
        std::vector<Eigen::Vector3f> sampledPoints = samplePointsInTriangle(vertices, numSample);
        float triArea = triangleArea(vertices);
        const tinyobj::material_t& mat = triLight->getMaterial();
        Vector3f triEmission = Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);

        Vector3f totalTriRadians = Vector3f(0, 0, 0);
        Vector3f totalTriDirection = Vector3f(0, 0, 0);
        for (Vector3f samplePoint:sampledPoints) {
            Vector3f omega = (samplePoint - p).normalized();
            Vector3f omegaPrime = (p - samplePoint).normalized();

            if (omegaPrime.y() >= 0) {
                continue;
            }

            Ray ray(p, omega);
            IntersectionInfo i;
            if (scene.getIntersection(ray, &i)) {
                const Triangle *firstIntersectTri = static_cast<const Triangle *>(i.data);
                if (firstIntersectTri->getIndex() != triLight->getIndex()){
                    continue;
                }
                else {
                    float cosTheta = omega.dot(p_normal);
                    float cosThetaPrime = omegaPrime.dot(triLight->getNormal(i));
                    float factor = cosTheta * cosThetaPrime / (p - samplePoint).squaredNorm();

                    totalTriRadians += triEmission * factor;
                    totalTriDirection += omega;
                }
            }
            else {
                continue;
            }
        }

        Vector3f triRadian = (triArea / numSample) * totalTriRadians;
        totalDirectRadian += triRadian;
        totalDirection += totalTriDirection / numSample;
    }

    Vector3f directRadian = totalDirectRadian / triLights.size();
    Vector3f direction = totalDirection / triLights.size();

    std::vector<Vector3f> result;
    result.push_back(directRadian);
    result.push_back(direction);

    return result;
}

std::vector<Eigen::Vector3f> Helper::samplePointsInTriangle(const Eigen::Vector3<Eigen::Vector3f>& vertices, int n) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(n);

    for (int i = 0; i < n; ++i) {
        // Generate two random numbers
        float u = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float v = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // Adjust to ensure they are inside the triangle
        if (u + v > 1.0f) {
            u = 1.0f - u;
            v = 1.0f - v;
        }

        // Convert to Cartesian coordinates
        float w = 1.0f - u - v;
        Eigen::Vector3f point = u * vertices[0] + v * vertices[1] + w * vertices[2];

        points.push_back(point);
    }

    return points;
}

float Helper::triangleArea(const Eigen::Vector3<Eigen::Vector3f>& vertices) {
    // Compute the vectors representing two sides of the triangle
    Eigen::Vector3f vector1 = vertices[1] - vertices[0];
    Eigen::Vector3f vector2 = vertices[2] - vertices[0];

    // Calculate the cross product of these two vectors
    Eigen::Vector3f crossProduct = vector1.cross(vector2);

    // The area of the triangle is half the magnitude of the cross product
    return 0.5f * crossProduct.norm();
}

/***** Functions for depth of field *****/
Vector3f Helper::calLensPoint(float aperture) {
    // generate a random point on the lens (aperture)
    float r = sqrt(drand48());
    float theta = 2 * M_PI * drand48();
    Vector3f randomPointOnLens = Vector3f(r * cos(theta), r * sin(theta), 0);

    Vector3f lensPoint = aperture * randomPointOnLens;
    return lensPoint;
}

Vector3f Helper::calDirection(Vector3f lensPoint, Vector3f d, float focalLength) {
    Vector3f focalPoint = d * focalLength;
    Vector3f direction = (focalPoint - lensPoint).normalized();

    return direction;
}

/***** Functions for quasi-monte carlo *****/
// Halton sequence generator for a given index and base
float Helper::haltonSequence(int index, int base) {
    float f = 1.0f, r = 0.0f;
    while (index > 0) {
        f /= base;
        r += f * (index % base);
        index /= base;
    }
    return r;
}
