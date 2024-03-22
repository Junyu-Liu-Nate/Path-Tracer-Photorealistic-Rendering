#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        #pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            if (!settings.enableStratifiedSampling && !settings.enableLowDiscrepancySampling) {
                intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
            }
            if (settings.enableStratifiedSampling && !settings.enableLowDiscrepancySampling) {
                intensityValues[offset] = tracePixelStratify(x, y, scene, invViewMat);
            }
            if (!settings.enableStratifiedSampling && settings.enableLowDiscrepancySampling) {
                intensityValues[offset] = tracePixelLowDiscrepency(x, y, scene, invViewMat);
            }
        }
    }

    // Save as PFM image for debug/HDR visualization use
    std::string pfmPath = "student_outputs/final/" + pfmFilename + ".pfm";
    outputPFM(pfmPath, m_width, m_height, intensityValues);

    //TODO - extra credit: Fixed Function Denoising, wavelet transform methods for Monte Carlo Rendering
    toneMap(imageData, intensityValues);
}

/***** The Monte Carlo version *****/
Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix) {
    Vector3f accumulateRay = Vector3f(0,0,0);
    int sampleCount = settings.samplesPerPixel;

    for (int i = 0; i < sampleCount; i++) {
        // Generate a random point within the pixel boundaries
        float u = ((float)x + static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX)) / m_width;
        float v = ((float)y + static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX)) / m_height;

        // Convert to screen space coordinates
        Vector3f d((2.f * u) - 1, 1 - (2.f * v), -1);
        d.normalize();

        accumulateRay += traceOneSample(d, scene, invViewMatrix);
    }

    return accumulateRay / sampleCount; // Average the accumulated radiance
}

/***** The Monte Carlo integration with stratified sampling *****/
float get_random_float() {
    static std::uniform_real_distribution<float> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

Vector3f PathTracer::tracePixelStratify(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    Vector3f accumulateRay = Vector3f(0,0,0);
    int numSamples = settings.samplesPerPixel;
    int sqrtSamples = int(sqrt(numSamples)); // use grid.
    int totalSamples = sqrtSamples * sqrtSamples; // Total samples per pixel
    float invSqrtSamples = 1.0f / sqrtSamples;

    for (int i = 0; i < sqrtSamples; ++i) {
        for (int j = 0; j < sqrtSamples; ++j) {
            for (int k = 0; k < 1; ++k) { // Loop to sample 10 points within each grid cell
                // Compute the offset within the stratum for each of the 10 samples
                float xOffset = (i + get_random_float()) * invSqrtSamples;
                float yOffset = (j + get_random_float()) * invSqrtSamples;

                // Calculate the direction for this sample.
                Vector3f d((2.f * (x + xOffset) / m_width) - 1, 1 - (2.f * (y + yOffset) / m_height), -1);
                d.normalize();

                accumulateRay += traceOneSample(d, scene, invViewMatrix);
            }
        }
    }
    return accumulateRay / totalSamples;
}

// Halton sequence generator for a given index and base
float haltonSequence(int index, int base) {
    float f = 1.0f, r = 0.0f;
    while (index > 0) {
        f /= base;
        r += f * (index % base);
        index /= base;
    }
    return r;
}

Vector3f PathTracer::tracePixelLowDiscrepency(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix) {
    Vector3f accumulateRay = Vector3f(0,0,0);
    int sampleCount = settings.samplesPerPixel;

    for (int i = 0; i < sampleCount; i++) {
        // Generate a random point within the pixel boundaries using Halton sequence
        float u = ((float)x + haltonSequence(i + 1, 2)) / m_width; // +1 to avoid zero
        float v = ((float)y + haltonSequence(i + 1, 3)) / m_height; // Use different bases

        // Convert to screen space coordinates
        Vector3f d((2.f * u) - 1, 1 - (2.f * v), -1);
        d.normalize();

        accumulateRay += traceOneSample(d, scene, invViewMatrix);
    }

    return accumulateRay / sampleCount; // Average the accumulated radiance
}

Vector3f PathTracer::traceOneSample(Vector3f d, const Scene& scene, const Matrix4f &invViewMatrix) {
    if (settings.enableDepthOfField) {
        float aperture = 0.1f;
        float focalLength = 6.0f;
        Vector3f lensPoint = helperFunc.calLensPoint(aperture);
        Vector3f direction = helperFunc.calDirection(lensPoint, d, focalLength);

        Ray r(lensPoint, direction);
        r = r.transform(invViewMatrix);

        // Accumulate the radiance estimated for this sample
        return traceRay(r, scene, true);
    }
    else {
        Ray r(Vector3f(0, 0, 0), d);
        r = r.transform(invViewMatrix);

        // Accumulate the radiance estimated for this sample
        return traceRay(r, scene, true);
    }
}


Vector3f PathTracer::traceRay(const Ray& r, const Scene& scene, bool countEmitted)
{
    IntersectionInfo i;
    Ray ray(r);

    if(scene.getIntersection(ray, &i)) {
        //** Example code for accessing materials provided by a .mtl file **
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = t->getMaterial();//Get the material of the triangle from the mesh
        const tinyobj::real_t *d = mat.diffuse; //Diffuse color as array of floats
        Vector3f dVec = Vector3f(d[0], d[1], d[2]);
        const tinyobj::real_t *s = mat.specular; //Specular colors
        Vector3f sVec = Vector3f(s[0], s[1], s[2]);
//        // For the milestone submission, hard-coded the diffusion to be [0.5,0.5,0.5] if the material is not diffuse
//        // For sphere only (NEED TO REAMOVE THIS FOR LATER IMPLEMENTATION)
//        if (dVec.x() < 0.05f && dVec.y() < 0.05f && dVec.z() < 0.05f) {
//            dVec = Vector3f(1, 1, 1);
//        }
//        const std::string diffuseTex = mat.diffuse_texname;//Diffuse texture name

        // Event splitting - separately account direact and indirect lighting
        // Do not double account light sources
        // Direct lighting
        Vector3f L = Vector3f(0,0,0);
        std::vector<Vector3f> directLighting = helperFunc.calDirectLighting(i.hit, t->getNormal(i), scene.getEmissives(), scene, settings.numDirectLightingSamples);
        Vector3f directRadiance = directLighting.at(0);
        Vector3f directDirection = directLighting.at(1);
        if (mat.shininess < 500 && mat.ior <= 2) {
            Vector3f totalBRDF;
            if (sVec.x() > 0.5) {
                totalBRDF = brdf.GlossyReflect(-directDirection, -r.d, t->getNormal(i), sVec, mat.shininess);
            }
            else {
                totalBRDF = brdf.idealDiffuse(dVec);
            }
            L.x() = directRadiance.x() * totalBRDF.x();
            L.y() = directRadiance.y() * totalBRDF.y();
            L.z() = directRadiance.z() * totalBRDF.z();
        }

        // Indirect Lighting - recursively trace rays on randomly sampled directions
        if (!settings.directLightingOnly) {
            float pdf_rr = settings.pathContinuationProb;

            float random_val = static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX);
            if (random_val < pdf_rr) {
                // Total diffuse BRDF
                if (mat.shininess < 500 && mat.ior <= 2 && sVec.x() < 0.5) {
                    Vector3f sampleOmega;
                    float samplePDF;
                    if (settings.enableImportanceSampling) {
                        auto result = helperFunc.cosineWeightedSampleHemisphere(t->getNormal(i));
                        sampleOmega = std::get<0>(result);
                        samplePDF = std::get<1>(result);
                    } else {
                        auto result = helperFunc.uniformSampleHemisphere(t->getNormal(i));
                        sampleOmega = std::get<0>(result);
                        samplePDF = std::get<1>(result);
                    }
                    sampleOmega.normalize();
                    Ray rNext(i.hit, sampleOmega);
                    Vector3f L_next = traceRay(rNext, scene, false);

                    Vector3f totalBRDF = brdf.idealDiffuse(dVec);

                    L.x() += L_next.x() * totalBRDF.x() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                    L.y() += L_next.y() * totalBRDF.y() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                    L.z() += L_next.z() * totalBRDF.z() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                }

                // Specular glossy BRDF
                if (mat.shininess < 500 && mat.ior <= 2 && sVec.x() >= 0.5) {
                    Vector3f sampleOmega;
                    float samplePDF;
                    if (settings.enableImportanceSampling) {
                        auto result = helperFunc.glossySpecularSampleHemisphere(t->getNormal(i), r.d, mat.shininess);
                        sampleOmega = std::get<0>(result);
                        samplePDF = std::get<1>(result);
                    } else {
                        auto result = helperFunc.uniformSampleHemisphere(t->getNormal(i));
                        sampleOmega = std::get<0>(result);
                        samplePDF = std::get<1>(result);
                    }
                    sampleOmega.normalize();
                    Ray rNext(i.hit, sampleOmega);
                    Vector3f L_next = traceRay(rNext, scene, false);

                    Vector3f totalBRDF;
                    if (settings.enableWardAnisotropicBRDF && !settings.enableCookTorranceBRDF) {
                        Vector3f tangent;
                        Vector3f bitangent;
                        brdf.calculateTangentBitangent(t->getNormal(i), t->getVertices(), tangent, bitangent);
                        totalBRDF = brdf.wardAnisotropicSpecular(rNext.d, -r.d, t->getNormal(i), sVec, 0.4, 1.0/mat.shininess, tangent, bitangent);
                    }
                    if (settings.enableCookTorranceBRDF && !settings.enableWardAnisotropicBRDF) {
                        totalBRDF = brdf.cookTorranceSpecularBRDF(-rNext.d, -r.d, t->getNormal(i), sVec, 1.0/mat.shininess, mat.ior);
                    }
                    if (!settings.enableCookTorranceBRDF && !settings.enableWardAnisotropicBRDF) {
                        totalBRDF = brdf.GlossyReflect(-rNext.d, -r.d, t->getNormal(i), sVec, mat.shininess);
                    }

                    L.x() += L_next.x() * totalBRDF.x() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                    L.y() += L_next.y() * totalBRDF.y() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                    L.z() += L_next.z() * totalBRDF.z() * sampleOmega.dot(t->getNormal(i)) / (pdf_rr*samplePDF);
                }

                // Total reflection BRDF
                if (mat.shininess >= 500) {
                    Vector3f mirrorOmega = r.d - 2 * r.d.dot(t->getNormal(i)) * t->getNormal(i);
                    mirrorOmega.normalize();
                    Ray rNext(i.hit, mirrorOmega);
                    Vector3f L_next = traceRay(rNext, scene, true);

                    L += L_next / pdf_rr;
                }

                // Refraction BRDF
                if (mat.ior > 2) {
                    Vector3f refractOmega;
                    float refractFactor;
                    bool isReflect = false; // Fresnel
                    float random_fresnel_val = 0.0;
                    float r_theta = 0.0;

                    if (r.d.dot(t->getNormal(i)) < 0) {
                        // Light from air to denser objects
                        refractFactor = 1 / mat.ior;
                        random_fresnel_val = static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX);
                        r_theta = helperFunc.fresnelProbability(refractFactor, r.d, t->getNormal(i));
                        if (random_fresnel_val < r_theta) {
                            refractOmega = r.d - 2 * r.d.dot(t->getNormal(i)) * t->getNormal(i);
                        }
                        else {
                            isReflect = true;
                            refractOmega = helperFunc.calRefractDirection(refractFactor, r.d, t->getNormal(i));
                        }
                    }
                    else {
                        // Light from denser objects to air
                        refractFactor = mat.ior;
                        random_fresnel_val = static_cast<float>(arc4random()) / static_cast<float>(UINT32_MAX);
                        r_theta = helperFunc.fresnelProbability(refractFactor, r.d, -t->getNormal(i));
                        if (random_fresnel_val < r_theta) {
                            refractOmega = r.d - 2 * r.d.dot(-t->getNormal(i)) * -t->getNormal(i);
                        }
                        else {
                            refractOmega = helperFunc.calRefractDirection(refractFactor, r.d, -t->getNormal(i));
                        }
                    }

                    Ray rNext(i.hit, refractOmega);
                    Vector3f L_next;

                    //Extra credit: Attenuate refracted paths
                    if (settings.enableAttenuateRefraction) {
                        float attenuation = 1.0;
                        if (refractOmega.dot(t->getNormal(i)) < 0) {
                            attenuation = helperFunc.calRefractAttenuation(i.hit, refractOmega, scene);
                        }
                        L_next = attenuation * traceRay(rNext, scene, true);
                    }
                    else {
                        L_next = traceRay(rNext, scene, true);
                    }


                    L += L_next / pdf_rr;
                }
            }
        }

        if (countEmitted) {
            L += Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
        }

        return L;
    } else {
        return Vector3f(0, 0, 0);
    }
}


float reinhardExtended(float v, float maxWhite) {
//    float extendedValue = v * (1 + v/(maxWhite*maxWhite)) / (1 + v);
    float extendedValue = v / (1.0f + v);
    return extendedValue;
}

void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
    float maxWhite = 0.8; // Set this to your chosen maximum white value

    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f &intensity = intensityValues[offset];

            // Calculate luminance
            float luminance = 0.2126 * intensity.x() + 0.7152 * intensity.y() + 0.0722 * intensity.z();

            // Apply the Extended Reinhard tone mapping to the luminance
            float mappedLuminance = reinhardExtended(luminance, maxWhite);

            // Scale the RGB channels to match the new luminance
            if (luminance > 0) {
                float scale = mappedLuminance / luminance;
                intensity.x() *= scale;
                intensity.y() *= scale;
                intensity.z() *= scale;
            }

            // Apply gamma correction and scale the intensity values to the range [0, 255]
            int r = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.x(), 1 / 2.2) * 255)));
            int g = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.y(), 1 / 2.2) * 255)));
            int b = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.z(), 1 / 2.2) * 255)));

            imageData[offset] = qRgb(r, g, b);
        }
    }
}

/**************************** Obsolete code (older versions) ****************************/

/***** The non-Monte Carlo version (from stencil) *****/
//Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
//{
//    //TODO - extra credit: Depth of field - scatter the starting location over a lens
//    Vector3f p(0, 0, 0);
//    Vector3f d((2.f * x / m_width) - 1, 1 - (2.f * y / m_height), -1);
//    d.normalize();

//    Ray r(p, d);
//    r = r.transform(invViewMatrix);
////    return traceRay(r, scene);

//    //TODO - extra credit: Stratified sampling - uniform sampling in on pixel
//    //TODO: What should the samling num be? (Review Monte Carlo Integration)
//    Vector3f accumulateRay = Vector3f(0,0,0);
//    int accumulateNum = 100; // Num of sample per pixel
//    for (int i=0; i<accumulateNum; i++) {
//        accumulateRay += traceRay(r, scene);
//    }
//    return accumulateRay / accumulateNum;
//}

/***** Separately map enery channel *****/
//void PathTracer::toneMap(QRgb *imageData, std::vector<Vector3f> &intensityValues) {
//    float maxWhite = 0.8; // Set this to your chosen maximum white value

//    for(int y = 0; y < m_height; ++y) {
//        for(int x = 0; x < m_width; ++x) {
//            int offset = x + (y * m_width);

//            Vector3f &intensity = intensityValues[offset];

//            // Apply the Extended Reinhard tone mapping
//            intensity.x() = reinhardExtended(intensity.x(), maxWhite);
//            intensity.y() = reinhardExtended(intensity.y(), maxWhite);
//            intensity.z() = reinhardExtended(intensity.z(), maxWhite);

//            // Apply gamma correction and scale the intensity values to the range [0, 255]
//            int r = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.x(), 1/2.2) * 255)));
//            int g = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.y(), 1/2.2) * 255)));
//            int b = std::min(255, std::max(0, static_cast<int>(std::pow(intensity.z(), 1/2.2) * 255)));

//            imageData[offset] = qRgb(r, g, b);
//        }
//    }
//}
