#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>
#include <random>

#include "scene/scene.h"
#include "util/helper.h"
#include "util/brdf.h"
#include "scene/shape/Sphere.h"

struct Settings {
    int samplesPerPixel;
    bool directLightingOnly; // if true, ignore indirect lighting
    int numDirectLightingSamples; // number of shadow rays to trace from each intersection point
    float pathContinuationProb; // probability of spawning a new secondary ray == (1-pathTerminationProb)

    bool enableDepthOfField = false;
    bool enableStratifiedSampling = false;
    bool enableLowDiscrepancySampling = false;
    bool enableAttenuateRefraction = false;
    bool enableImportanceSampling = false;
    bool enableWardAnisotropicBRDF = false;
    bool enableCookTorranceBRDF = false;
};

class PathTracer
{
public:
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);
    Settings settings;
    std::string pfmFilename;

private:
    Helper helperFunc = Helper();
    BRDF brdf = BRDF();

    int m_width, m_height;

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);

    Eigen::Vector3f tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
    Vector3f tracePixelStratify(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix);
    Vector3f tracePixelLowDiscrepency(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix);
    Vector3f traceOneSample(Vector3f d, const Scene& scene, const Matrix4f &invViewMatrix);

    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, bool);
};

#endif // PATHTRACER_H
